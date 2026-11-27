/* App-Logik: WebSocket-Rohdaten, Worker-Berechnung, 2D/3D-Viewer, Steuerung. */
'use strict';

const $ = (id) => document.getElementById(id);
const rawFetch = (path, opts) => fetch(path, { headers: { 'Content-Type': 'application/json' }, ...opts });

// Meldung unten rechts einblenden (Fehler des Servers, Hinweise)
function toast(text, kind = 'error') {
  const box = $('toast');
  if (!box) { console.warn(text); return; }
  const el = document.createElement('div');
  el.className = 'toast ' + kind;
  el.textContent = text;
  box.appendChild(el);
  setTimeout(() => el.remove(), kind === 'error' ? 9000 : 4000);
}

// Wie fetch, zeigt aber Serverfehler (409 „Beschäftigt“, Position unbekannt …) an
async function api(path, opts) {
  let r;
  try { r = await rawFetch(path, opts); }
  catch (e) { if (opts && opts.method && opts.method !== 'GET') toast('Pi nicht erreichbar: ' + e.message); throw e; }
  if (!r.ok && opts && opts.method && opts.method !== 'GET') {
    let msg = `${r.status} ${r.statusText}`;
    try { const j = await r.clone().json(); if (j.error) msg = j.error; } catch (e) {}
    toast(msg);
  }
  return r;
}

const v2d = new Viewer2D($('view2d'));
const v3d = new Viewer3D($('view3d'));
const ASSET_VERSION = '20261128a';  // wie ?v= in index.html – erzwingt neue Skripte nach Updates
const worker = new Worker(`js/scan.worker.js?v=${ASSET_VERSION}`);

let ws = null;
let viewMode = '2d';
let totalPoints = 0;
let lidarResDeg = 0;
let lastState = null;  // für das automatische Nachladen der Scan-Liste
const edgeFilterOn = () => { const e = document.getElementById('edgeChk'); return !!(e && e.checked); };
let loadedScans = [];   // überlagerte Scans (Index = src = Farbe im Modus „Scan-Nummer“)
let poses = [];         // Lage je Scan: {yaw [rad], tx, ty, tz [m]}  p' = Rz(yaw)·p + t
let alignQ = [];        // Güte der Ausrichtung je Scan {overlap, rmse}
let seamView = null;    // Naht-Ansicht aktiv: {id, buf}
const rawWaiters = new Map();   // src -> resolve, wenn der Worker einen Scan fertig gerechnet hat

// Geometrie-Parameter für den Worker aus der aktuellen Konfiguration
async function refreshWorkerCfg() {
  try {
    const c = await (await rawFetch('/api/config')).json();
    worker.postMessage({ type: 'init', angle_offset: c.LIDAR.ANGLE_OFFSET,
      model_y: c.GEOMETRY.MODEL_Y_OFFSET, model_z: c.GEOMETRY.MODEL_Z_OFFSET,
      dist_min: c.LIDAR.DISTANCE_MIN_MM, dist_max: c.LIDAR.DISTANCE_MAX_MM, edge_filter: edgeFilterOn(),
      z_scale: c.GEOMETRY.Z_SCALE ?? 1, half_yaw: c.GEOMETRY.HALF_YAW ?? 0 });
  } catch (e) { /* offline – Init kommt mit dem WebSocket */ }
}
refreshWorkerCfg();   // vertikale Winkelauflösung [°], live aus Frames gemessen

v2d.start();
// Nachleuchtzeit der 2D-Ansicht (je Browser gemerkt)
(() => {
  const sel = $('persist2d');
  if (!sel) return;
  try { const v = localStorage.getItem('pilidar.persist2d'); if (v) sel.value = v; } catch (e) {}
  v2d.setPersistence(sel.value);
  sel.onchange = () => {
    v2d.setPersistence(sel.value);
    try { localStorage.setItem('pilidar.persist2d', sel.value); } catch (e) {}
  };
})();
applyViewMode();  // 3D-Labels-Overlay beim Start korrekt ausblenden

// --- Worker-Rückgaben -------------------------------------------------
worker.onmessage = (ev) => {
  const m = ev.data;
  if (m.type === 'points') {
    v3d.addPoints(m.xyz, m.inten, m.za, m.src || 0);
    totalPoints = m.total;
    $('stPts').textContent = m.progress !== undefined && m.progress < 1
      ? `${totalPoints.toLocaleString('de-DE')} (${Math.round(m.progress * 100)} %)`
      : totalPoints.toLocaleString('de-DE');
  } else if (m.type === 'rawDone') {
    $('stPts').textContent = m.total.toLocaleString('de-DE') + (m.edgeRemoved ? ` (${m.edgeRemoved.toLocaleString('de-DE')} Kantenpunkte gefiltert)` : '');
    v3d._flush();
    const w = rawWaiters.get(m.src);
    if (w) { rawWaiters.delete(m.src); w(); }
  } else if (m.type === 'exportData') {
    exportPLY(m.xyz, m.inten, m.total);
  }
};

// --- WebSocket --------------------------------------------------------
function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws/scan`);
  ws.binaryType = 'arraybuffer';
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') {           // Init (JSON)
      const m = JSON.parse(ev.data);
      if (m.type === 'init') worker.postMessage({ type: 'init', ...m });
      return;
    }
    // Binär-Bündel: uint32 n | f32 z[n] | f32 s[n] | f32 a[n*12] | u16 d[n*12] | u8 i[n*12]
    const buf = ev.data;
    const n = new Uint32Array(buf, 0, 1)[0];
    if (!n) return;
    const a = new Float32Array(buf, 4 + n * 8, n * 12);
    const d = new Uint16Array(buf, 4 + n * 56, n * 12);
    // Winkelauflösung aus benachbarten Punkten eines Pakets (Wrap ausschließen)
    const step = Math.abs(a[1] - a[0]);
    if (step > 0 && step < 5) lidarResDeg = step;
    for (let f = 0; f < n; f++)                   // 2D: Nachleuchten je Winkelsegment
      v2d.addFrame(a.subarray(f * 12, f * 12 + 12), d.subarray(f * 12, f * 12 + 12));
    // gespeicherte Scans in 3D: Live-Punkte nicht hineinmischen (erst „Leeren“ zeigt wieder live)
    if (!loadedScans.length && !seamView) worker.postMessage({ type: 'batch', buf }, [buf]);
  };
  ws.onclose = () => setTimeout(connectWS, 1500);
}
connectWS();

// --- Steuerung --------------------------------------------------------
$('btnLidar').onclick = () => api('/api/lidar/start', { method: 'POST' });
$('btnLidarStop').onclick = () => api('/api/lidar/stop', { method: 'POST' });

$('btnScan').onclick = () => {
  clearView();
  api('/api/scan/start', { method: 'POST', body: JSON.stringify({ mode: $('mode').value, name: $('name').value }) });
};
$('btnStop').onclick = () => api('/api/scan/stop', { method: 'POST' });

$('btnCalRot').onclick = async () => {
  $('calOut').textContent = 'Rotationskalibrierung läuft (360°-Lauf)…';
  const resp = await api('/api/calibrate/rotation', { method: 'POST' });
  if (!resp.ok) { $('calOut').textContent = 'nicht gestartet'; return; }
  const r = await resp.json();
  $('calOut').textContent = JSON.stringify(r, null, 1);
  if (r.ok && confirm(`Getriebeverhältnis ${r.gear_ratio.toFixed(4)} übernehmen?`))
    await api('/api/config/apply', { method: 'POST', body: JSON.stringify({ gear_ratio: r.gear_ratio }) });
  refreshWorkerCfg();
};
$('btnCalOff').onclick = async () => {
  $('calOut').textContent = 'Offset-Kalibrierung läuft (>180°-Lauf)…';
  const resp = await api('/api/calibrate/offset', { method: 'POST' });
  if (!resp.ok) { $('calOut').textContent = 'nicht gestartet'; return; }
  const r = await resp.json();
  $('calOut').textContent = JSON.stringify(r, null, 1);
  if (r.ok && confirm(`Winkel ${r.angle_offset.toFixed(2)}° übernehmen (Rest ${r.residual_mm.toFixed(1)} mm)? y und z bleiben.`))
    await api('/api/config/apply', { method: 'POST', body: JSON.stringify({ angle_offset: r.angle_offset }) });
  refreshWorkerCfg();
};

function applyViewMode() {
  viewMode = document.querySelector('input[name=view]:checked').value;
  $('view2d').classList.toggle('hidden', viewMode !== '2d');
  $('view3d').classList.toggle('hidden', viewMode !== '3d');
  const ov = document.querySelector('.v3d-labels');
  if (ov) ov.classList.toggle('hidden', viewMode !== '3d');
  if (viewMode === '3d') {
    v2d.stop();        // 2D-Loop pausieren, spart Rechenzeit
    v3d._resize();     // Canvas war evtl. 0×0 während hidden
  } else {
    v2d.start();       // 2D-Loop wieder aufnehmen (idempotent)
  }
}
document.querySelectorAll('input[name=view]').forEach(r => r.onchange = applyViewMode);
applyViewMode();  // Anfangszustand setzen (3D-Achsen-Overlay korrekt aus-/einblenden)

function clearView() {
  v2d.clear(); v3d.clear(); worker.postMessage({ type: 'clear' });
  totalPoints = 0; $('stPts').textContent = '0';
  loadedScans = []; poses = []; alignQ = []; $('alignInfo').textContent = '';
  seamView = null;
  if ($('manualBox')) $('manualBox').classList.add('hidden');
}
$('btnClear').onclick = clearView;
// Punkte für den Export: sichtbar (Höhenschnitt), optional ohne Ausreißer, optional ausgedünnt
function exportPoints() {
  let v = v3d.visiblePoints();
  if ($('outlierChk').checked && v.total) {
    v = removeOutliers(v);
    toast(`${v.removed.toLocaleString('de-DE')} Ausreißer entfernt.`, 'info');
  }
  return thinPoints(v, +$('thinCell').value);
}
// PLY: sichtbare Punkte (Höhenschnitt beachtet) direkt aus dem Viewer
$('btnPly').onclick = () => {
  const v = exportPoints();
  if (!v.total) { toast('Keine Punkte in der 3D-Ansicht.'); return; }
  exportPLY(v.xyz, v.inten, v.total);
};

$('btnLas').onclick = () => {
  const v = exportPoints();
  if (!v.total) { toast('Keine Punkte in der 3D-Ansicht.'); return; }
  exportLAS(v.xyz, v.inten, v.total, `${currentScanId || 'pilidar'}.las`);
};
$('btnPng').onclick = () => {
  const a = document.createElement('a');
  a.href = $('view3d').toDataURL('image/png');
  a.download = `${currentScanId || 'pilidar'}_ansicht.png`; a.click();
};

// --- Schnittansicht --------------------------------------------------------
let secRes = null;
function secCompute() {
  try {
    const t0 = performance.now();
    secRes = Section.compute(v3d, { x0: +$('secX').value || 0, y0: +$('secY').value || 0,
                                    dirDeg: +$('secDir').value || 0, thick: +$('secThick').value });
    Section.draw($('secCanvas'), secRes);
    $('secInfo').textContent = `${secRes.n.toLocaleString('de-DE')} Punkte · Länge ${(secRes.sMax - secRes.sMin).toFixed(2)} m · ` +
      `Höhe ${secRes.zMin.toFixed(2)} … ${secRes.zMax.toFixed(2)} m (${(secRes.zMax - secRes.zMin).toFixed(2)} m) · ` +
      `${Math.round(performance.now() - t0)} ms. Raster 0,5 m, rot gestrichelt = Bezugspunkt.`;
  } catch (e) { toast('Schnitt: ' + e.message); }
}
$('btnSection').onclick = () => {
  if (!v3d.total) { toast('Erst einen Scan in 3D laden.'); return; }
  $('secDlg').classList.remove('hidden'); secCompute();
};
$('secClose').onclick = () => $('secDlg').classList.add('hidden');
$('secRedo').onclick = secCompute;
for (const id of ['secX', 'secY', 'secDir', 'secThick']) $(id).onchange = secCompute;
$('secTurn').onclick = () => { $('secDir').value = (((+$('secDir').value || 0) + 90) % 360).toFixed(1); secCompute(); };
$('secFromPicks').onclick = () => {
  if (!v3d.picks || v3d.picks.length < 2) { toast('Erst mit „Messen“ zwei Punkte in der 3D-Ansicht anklicken.'); return; }
  const [a, b] = v3d.picks;
  $('secX').value = a[0].toFixed(2); $('secY').value = a[1].toFixed(2);
  $('secDir').value = (Math.atan2(b[1] - a[1], b[0] - a[0]) * 180 / Math.PI).toFixed(1); secCompute();
};
$('secWalls').onclick = () => {
  try {
    const f = FloorPlan.compute(v3d, { orthogonal: true });
    if (typeof f.m.baseDeg !== 'number') throw new Error('keine Wände erkannt');
    $('secDir').value = f.m.baseDeg.toFixed(1); secCompute();
  } catch (e) { toast('Ausrichten: ' + e.message); }
};
const secName = ext => `schnitt_${(currentScanId || 'live')}.${ext}`;
$('secDxf').onclick = () => secRes && FloorPlan.download(secName('dxf'), Section.toDXF(secRes), 'application/dxf');
$('secSvg').onclick = () => secRes && FloorPlan.download(secName('svg'), Section.toSVG(secRes), 'image/svg+xml');
$('secPng').onclick = () => { const a = document.createElement('a'); a.href = $('secCanvas').toDataURL('image/png'); a.download = secName('png'); a.click(); };

// --- 3D-Darstellung ----------------------------------------------------
$('colorMode').onchange = () => {
  const m = +$('colorMode').value;
  if (m !== 6) { v3d.setColorMode(m); return; }
  try {                                               // Abweichung zwischen Scan 0 und den dazugeladenen
    const t0 = performance.now(), r = v3d.computeDiff(0.10);
    toast(`Abweichung berechnet (${Math.round(performance.now() - t0)} ms): ${(r.share * 100).toFixed(1)} % der Punkte ` +
          'liegen 5 cm oder mehr vom anderen Scan entfernt (rot = ab 10 cm). Vorher „Scans ausrichten“.', 'info');
  } catch (e) { toast(e.message); $('colorMode').value = '5'; v3d.setColorMode(5); }
};
// Kantenfilter: Einstellung merken, geladene Scans mit gleicher Lage neu rechnen
try { $('edgeChk').checked = localStorage.getItem('pilidar.edge') === '1'; } catch (e) {}
$('edgeChk').onchange = async () => {
  try { localStorage.setItem('pilidar.edge', $('edgeChk').checked ? '1' : '0'); } catch (e) {}
  worker.postMessage({ type: 'init', edge_filter: edgeFilterOn() });
  if (!loadedScans.length) return;
  const ids = loadedScans.slice(), ps = poses.slice(), qs = alignQ.slice(), cm = $('colorMode').value;
  clearView();
  for (let k = 0; k < ids.length; k++) {
    await loadScanInto3D(ids[k], k > 0, true);
    const p = ps[k];
    if (p && (p.yaw || p.tx || p.ty || p.tz)) v3d.transformSource(k, p.yaw, p.tx, p.ty, p.tz);
    poses[k] = p || poses[k]; alignQ[k] = qs[k];
  }
  $('colorMode').value = cm; v3d.setColorMode(+cm); showAlignInfo();
};
$('ptSize').oninput = () => v3d.setPointSize(+$('ptSize').value);
function applyClip() {
  const lo = $('clipLo').value === '' ? -1e9 : +$('clipLo').value;
  const hi = $('clipHi').value === '' ? 1e9 : +$('clipHi').value;
  v3d.setClip(lo, hi);
}
$('clipLo').oninput = applyClip; $('clipHi').oninput = applyClip;
$('btnClipReset').onclick = () => { $('clipLo').value = ''; $('clipHi').value = ''; applyClip(); };
$('btnNoCeiling').onclick = () => {
  const c = FloorPlan.estimateCeiling(v3d.chunks, v3d.bbox);
  if (c === null) { toast('Keine Punktwolke geladen.'); return; }
  $('clipHi').value = (c - 0.10).toFixed(2); applyClip();
  toast(`Decke bei ${c.toFixed(2)} m ausgeblendet.`, 'info');
};
document.querySelectorAll('.view-btn').forEach(b => b.onclick = () => v3d.setView(b.dataset.view));
$('btnMeasure').onclick = () => {
  const on = !v3d.measuring; v3d.setMeasuring(on);
  $('btnMeasure').classList.toggle('active', on);
  if (on) toast('Messen: zwei Punkte in der 3D-Ansicht anklicken.', 'info');
};

// --- Grundriss -----------------------------------------------------------
let floorRes = null;
function floorCompute(opts) {
  try {
    const t0 = performance.now();
    if (loadedScans.length > 1) opts = { ...opts, stations: poses.slice(0, loadedScans.length).map(p => [p.tx, p.ty]) };
    floorRes = FloorPlan.compute(v3d, opts);
    FloorPlan.draw($('floorCanvas'), floorRes);
    $('floorLo').value = floorRes.zLo.toFixed(2); $('floorHi').value = floorRes.zHi.toFixed(2);
    $('floorInfo').textContent = `Decke bei ${floorRes.ceiling.toFixed(2)} m · Streifen ${floorRes.zLo.toFixed(2)}…${floorRes.zHi.toFixed(2)} m · ` +
      `${floorRes.segs.length} Wandstücke, ${floorRes.wallLength.toFixed(1)} m Wandlänge · ${Math.round(performance.now() - t0)} ms. ` +
      FloorPlan.describe(floorRes).join(' · ') + '. ' +
      `Nur senkrechte Flächen (Dachschrägen/Möbel ausgefiltert). Roter Punkt = Scannerstandort.`;
  } catch (e) { toast('Grundriss: ' + e.message); }
}
$('btnFloor').onclick = () => {
  if (!v3d.total) { toast('Erst einen Scan in 3D laden.'); return; }
  $('floorDlg').classList.remove('hidden'); floorCompute({ orthogonal: $('floorOrtho').checked });
};
$('floorClose').onclick = () => $('floorDlg').classList.add('hidden');
$('floorRedo').onclick = () => floorCompute({ zLo: +$('floorLo').value, zHi: +$('floorHi').value,
                                              orthogonal: $('floorOrtho').checked });
const floorName = ext => `grundriss_${(currentScanId || 'live')}.${ext}`;
$('floorDxf').onclick = () => floorRes && FloorPlan.download(floorName('dxf'), FloorPlan.toDXF(floorRes, false), 'application/dxf');
$('floorDxfPts').onclick = () => floorRes && FloorPlan.download(floorName('dxf'), FloorPlan.toDXF(floorRes, true), 'application/dxf');
$('floorSvg').onclick = () => floorRes && FloorPlan.download(floorName('svg'), FloorPlan.toSVG(floorRes), 'image/svg+xml');

// --- System-Steuerung -------------------------------------------------
$('btnReboot').onclick = () => {
  if (confirm('Raspberry Pi wirklich neu starten?'))
    api('/api/system/reboot', { method: 'POST' });
};
$('btnPoweroff').onclick = () => {
  if (confirm('Raspberry Pi wirklich ausschalten?'))
    api('/api/system/poweroff', { method: 'POST' });
};

// --- LiDAR-Health-Indikator -------------------------------------------
function updateLidarHealth(s) {
  const dot = $('lidarHealth');
  const txt = $('lidarHealthTxt');
  const rate = s.stats.packet_rate;
  const crc  = s.stats.crc_error_rate;
  dot.className = 'dot';
  if (!s.lidar_running) {
    dot.classList.add('dot-off'); txt.textContent = 'aus';
  } else if (crc > 0.02) {
    dot.classList.add('error'); txt.textContent = `CRC-Fehler ${(crc * 100).toFixed(1)} %`;
  } else if (rate >= 1500) {
    dot.classList.add('ok'); txt.textContent = `OK (${Math.round(rate)} pkt/s)`;
  } else if (rate >= 1000) {
    dot.classList.add('warn'); txt.textContent = `Degradiert (${Math.round(rate)} pkt/s)`;
  } else {
    dot.classList.add('error'); txt.textContent = `Fehler (${Math.round(rate)} pkt/s)`;
  }
}

// --- Status-Polling ---------------------------------------------------
async function poll() {
  try {
    const s = await (await rawFetch('/api/status')).json();
    const b = $('state'); b.textContent = s.state;
    $('posBanner').classList.toggle('hidden', !s.position_lost);
    const running = s.state === 'scanning' || s.state === 'calibrating';
    $('progress').classList.toggle('hidden', !running && s.state !== 'processing');
    if (running && s.scan_angle) {
      const pct = Math.max(0, Math.min(1, s.angle / s.scan_angle));
      const rest = Math.max(0, (s.scan_angle - s.angle) / (s.speed_dps || 1));
      $('progressBar').style.width = `${(pct * 100).toFixed(0)}%`;
      $('progressTxt').textContent = `${(pct * 100).toFixed(0)} % · noch ca. ${Math.ceil(rest)} s`;
    } else if (s.state === 'processing') {
      $('progressBar').style.width = '100%'; $('progressTxt').textContent = 'Auswertung …';
    }
    // Scan-Liste nachladen, sobald ein Lauf (Scan/Kalibrierung) fertig ist
    if (s.state === 'idle' && lastState && lastState !== 'idle' && lastState !== 'lidar_only') refreshScans();
    lastState = s.state;
    b.className = 'badge ' + (s.state === 'idle' ? 'ok' : s.state === 'scanning' ? 'warn' : '');
    $('stAngle').textContent = s.angle;
    $('stRate').textContent = s.lidar_running ? Math.round(s.stats.packet_rate) : 0;
    $('stCrc').textContent = (s.stats.crc_error_rate * 100).toFixed(2);
    // Optimale Plattform-Geschwindigkeit für gleichen H/V-Punktabstand:
    //   SPEED_DPS = Auflösung[°] × Umdrehungsfrequenz[Hz] = res × (spiegel_dps / 360)
    if (s.stats.last_speed_dps > 0 && lidarResDeg > 0) {
      const freqHz = s.stats.last_speed_dps / 360;
      $('stOptSpeed').textContent = (lidarResDeg * freqHz).toFixed(2);
    }
    updateLidarHealth(s);
  } catch (e) { /* offline */ }
  try {
    const sys = await (await rawFetch('/api/system/stats')).json();
    if (sys.available) {
      $('stCpu').textContent = sys.cpu_percent.toFixed(1);
      $('stRam').textContent = sys.ram_percent.toFixed(1);
      if (sys.disk) $('stDisk').textContent = `${sys.disk.free_gb} GB (${(100 - sys.disk.percent).toFixed(0)} %)`;
      const eth = sys.net.eth0 || sys.net.end0 || sys.net.wlan0 || {};
      const bt  = sys.net.bnep0 || {};
      const parts = [];
      if (eth.rx_bps !== undefined)
        parts.push(`LAN ↓${_kb(eth.rx_bps)} ↑${_kb(eth.tx_bps)}`);
      if (bt.rx_bps !== undefined && (bt.rx_bps + bt.tx_bps) > 0)
        parts.push(`BT ↓${_kb(bt.rx_bps)} ↑${_kb(bt.tx_bps)}`);
      $('stNet').textContent = parts.length ? parts.join(' · ') : 'Netzwerk: –';
    }
  } catch (e) { /* psutil nicht verfügbar */ }
}
function _kb(bps) { return bps < 1024 ? `${bps} B/s` : `${(bps/1024).toFixed(0)} KB/s`; }

setInterval(poll, 700); poll();

// --- Scan-Liste -------------------------------------------------------
async function refreshScans() {
  const scans = await (await rawFetch('/api/scans')).json();
  const ul = $('scanList'); ul.innerHTML = '';
  for (const s of scans) {
    const qa  = (s.qa && s.qa.status) || '';
    const ann = (s.annotation || '').replace(/"/g, '&quot;');
    const li  = document.createElement('li');
    li.innerHTML = `
      <div class="row">
        <span><span class="dot ${qa}"></span><b>${s.id}</b></span>
        <span style="display:flex;gap:6px;align-items:center">
          <button class="btn-view3d" data-id="${s.id}" title="In 3D anzeigen">&#128065; 3D</button>
          <button class="btn-add3d" data-id="${s.id}" title="Zur 3D-Ansicht hinzufügen (überlagern)">+</button>
          <button class="btn-calscan" data-id="${s.id}" title="Offsets aus diesem Scan kalibrieren (ohne Motor)">&#9881;</button>
          <a href="/api/scans/${s.id}/download">ZIP</a>
          <button class="btn-del" data-id="${s.id}">&#128465;</button>
        </span>
      </div>
      <div style="color:#7d8794;margin-top:2px">Modus ${s.mode || '?'} &middot; ${(s.n_points || 0).toLocaleString('de-DE')} Pkt &middot; QA ${qa || '–'}${
        s.qa && typeof s.qa.loop_closure_mm === 'number' ? ` &middot; Überlappung ${s.qa.loop_closure_mm.toFixed(0)} mm` : ''}${
        typeof s.size_mb === 'number' ? ` &middot; ${s.size_mb} MB` : ''}${
        s.qa && s.qa.messages && s.qa.messages.length ? `<br><span style="color:#e0b050">${s.qa.messages.join('; ')}</span>` : ''}</div>
      <div class="ann-row">
        <input class="ann-input" type="text" placeholder="Anmerkung…" value="${ann}" />
        <button class="ann-save" data-id="${s.id}">&#10003;</button>
      </div>`;
    ul.appendChild(li);
  }
  ul.querySelectorAll('.btn-view3d').forEach(btn => {
    btn.onclick = () => loadScanInto3D(btn.dataset.id);
  });
  ul.querySelectorAll('.btn-add3d').forEach(btn => {
    btn.onclick = () => loadScanInto3D(btn.dataset.id, true);
  });
  ul.querySelectorAll('.btn-calscan').forEach(btn => {
    btn.onclick = () => calibrateFromScan(btn.dataset.id);
  });
  ul.querySelectorAll('.btn-del').forEach(btn => {
    btn.onclick = async () => {
      if (!confirm(`Scan "${btn.dataset.id}" wirklich löschen?`)) return;
      await api(`/api/scans/${btn.dataset.id}`, { method: 'DELETE' });
      refreshScans();
    };
  });
  ul.querySelectorAll('.ann-save').forEach(btn => {
    btn.onclick = async () => {
      const input = btn.closest('li').querySelector('.ann-input');
      await api(`/api/scans/${btn.dataset.id}/annotation`, {
        method: 'POST',
        body: JSON.stringify({ text: input.value }),
      });
    };
  });
}
// Gespeicherten Scan in die 3D-Ansicht laden
let currentScanId = null;   // zuletzt in 3D geladener Scan (für Dateinamen)
async function loadScanInto3D(scanId, add = false, quiet = false) {
  currentScanId = scanId;
  if (!add || !loadedScans.length) clearView();
  const src = loadedScans.length;
  loadedScans.push(scanId); poses[src] = { yaw: 0, tx: 0, ty: 0, tz: 0 }; alignQ[src] = null;
  if (add) setTimeout(updateManualBox, 0);
  if (add && !quiet) toast(`Scan ${scanId} überlagert (Nr. ${src}). Mit „Scans ausrichten“ passend zueinander legen.`, 'info');
  // auf 3D-Ansicht umschalten
  const radio3d = document.querySelector('input[name="view"][value="3d"]');
  radio3d.checked = true;
  applyViewMode();
  $('stPts').textContent = 'lädt …';
  try {
    await refreshWorkerCfg();                      // aktuelle Offsets verwenden
    const raw = await rawFetch(`/api/scans/${scanId}/raw`);
    if (raw.ok) {                                  // Browser rechnet aus den Rohdaten
      const buf = await raw.arrayBuffer();
      await new Promise(res => { rawWaiters.set(src, res); worker.postMessage({ type: 'rawScan', buf, scanId, src }, [buf]); });
      return;
    }
    // Fallback: auf dem Pi gespeicherte Punktwolke (ältere Scans ohne Rohdaten)
    const r = await rawFetch(`/api/scans/${scanId}/pointcloud`);
    if (!r.ok) { toast('Für diesen Scan gibt es weder Rohdaten noch Punktwolke.'); $('stPts').textContent = '0'; return; }
    // Kompakter Binär-Buffer: uint32 n | float32[n*3] xyz | uint8[n] Intensität
    const buf = await r.arrayBuffer();
    const n = new Uint32Array(buf, 0, 1)[0];
    const xyz = new Float32Array(buf, 4, n * 3);
    const inten = Float32Array.from(new Uint8Array(buf, 4 + n * 12, n));
    // in Blöcken einspeisen, damit sehr große Wolken den GPU-Upload nicht blockieren
    const CHUNK = 200000;  // Punkte pro Block
    for (let i = 0; i < n; i += CHUNK) {
      const m = Math.min(CHUNK, n - i);
      v3d.addPoints(xyz.slice(i * 3, (i + m) * 3), inten.slice(i, i + m), null, src);
    }
    v3d._flush();
    totalPoints = n;
    $('stPts').textContent = totalPoints.toLocaleString('de-DE');
  } catch (e) {
    console.error(e);
    $('stPts').textContent = '0';
    toast('Fehler beim Laden der Punktwolke: ' + e);
  }
}

// Offset-Kalibrierung aus einem gespeicherten Scan (rechnet auf dem Pi, ~3 s, kein Motor)
async function calibrateFromScan(scanId) {
  $('calOut').textContent = `Offset-Kalibrierung aus Scan ${scanId} …`;
  const resp = await api('/api/calibrate/offset_from_scan', { method: 'POST', body: JSON.stringify({ scan_id: scanId }) });
  if (!resp.ok) { $('calOut').textContent = 'nicht möglich'; return; }
  const r = await resp.json();
  $('calOut').textContent = JSON.stringify(r, null, 1);
  if (r.ok && confirm(`Aus Scan ${scanId}: Winkel ${r.angle_offset.toFixed(2)}° ` +
                      `(Rest ${r.residual_mm.toFixed(1)} mm). Übernehmen? Die Höhe bleibt unverändert.`)) {
    await api('/api/config/apply', { method: 'POST', body: JSON.stringify({
      angle_offset: r.angle_offset }) });
    refreshWorkerCfg();
    toast('Offsets übernommen – gespeicherte Scans werden beim Öffnen neu gerechnet.', 'info');
  }
}

async function runSelftest(quiet) {
  try {
    const r = await (await rawFetch('/api/selftest')).json();
    const bad = r.checks.filter(c => !c.ok);
    if (!quiet || bad.length)
      toast(bad.length ? 'Selbsttest: ' + bad.map(c => `${c.name}: ${c.detail}`).join(' · ')
                       : 'Selbsttest: alles in Ordnung', bad.some(c => c.level === 'error') ? 'error' : 'info');
  } catch (e) { /* offline */ }
}
$('btnSelftest').onclick = () => runSelftest(false);
setTimeout(() => runSelftest(true), 1500);   // beim Laden nur Probleme melden

// --- Mehrere Scans: Ausrichtung und Projekte -------------------------------
const regWorker = new Worker(`js/registration.worker.js?v=${ASSET_VERSION}`);
let regId = 0;
function regRun(ref, mov, opts, label) {
  return new Promise((resolve, reject) => {
    const id = ++regId;
    regWorker.onmessage = (ev) => {
      const m = ev.data; if (m.id !== id) return;
      if (m.type === 'progress') $('alignInfo').textContent = `${label}: ${m.phase} … ${Math.round(m.f * 100)} %`;
      else if (m.type === 'result') resolve(m.res);
      else reject(new Error(m.error));
    };
    regWorker.postMessage({ type: 'align', id, ref, mov, opts }, [ref.buffer, mov.buffer]);
  });
}
// delta ∘ pose
function composePose(d, p) {
  const c = Math.cos(d.yaw), s = Math.sin(d.yaw);
  return { yaw: d.yaw + p.yaw, tx: c * p.tx - s * p.ty + d.tx, ty: s * p.tx + c * p.ty + d.ty, tz: p.tz + d.tz };
}
function describePose(k) {
  const p = poses[k], q = alignQ[k], deg = ((p.yaw * 180 / Math.PI) % 360 + 540) % 360 - 180;
  let t = `Nr. ${k}: ${deg.toFixed(1)}°, (${p.tx.toFixed(2)} | ${p.ty.toFixed(2)} | ${p.tz.toFixed(2)}) m`;
  if (q && q.manual) t += ' (von Hand)';
  else if (q) {
    t += `, Überdeckung ${Math.round(q.overlap * 100)} %` + (q.rmse ? `, Abweichung ${(q.rmse * 100).toFixed(1)} cm` : '');
    if (q.overlap < 0.3) t += ' – unsicher';
    else if (q.second && q.overlap - q.second < 0.05) t += ' – mehrdeutig, bitte prüfen';
  }
  return t;
}
function showAlignInfo() {
  $('alignInfo').innerHTML = loadedScans.length > 1
    ? loadedScans.map((id, k) => k ? describePose(k) : `Nr. 0: ${id} (Bezug)`).join('<br>') : '';
  updateManualBox();
}

// --- Lage von Hand ------------------------------------------------------------
function updateManualBox() {
  const n = loadedScans.length;
  $('manualBox').classList.toggle('hidden', n < 2);
  if (n < 2) return;
  const sel = $('mvScan'), cur = +sel.value || 1;
  sel.innerHTML = '';
  for (let k = 1; k < n; k++) {
    const o = document.createElement('option'); o.value = k; o.textContent = `Scan Nr. ${k}`; sel.appendChild(o);
  }
  sel.value = Math.min(cur, n - 1);
  fillManualInputs();
}
function fillManualInputs() {
  const p = poses[+$('mvScan').value]; if (!p) return;
  $('mvYaw').value = (((p.yaw * 180 / Math.PI) % 360 + 540) % 360 - 180).toFixed(1);
  $('mvX').value = p.tx.toFixed(3); $('mvY').value = p.ty.toFixed(3); $('mvZ').value = p.tz.toFixed(3);
}
// Lage eines Scans auf einen Zielwert setzen (Punkte: erst zurück auf 0, dann Ziel)
function setPose(k, target) {
  const p = poses[k], c = Math.cos(-p.yaw), s = Math.sin(-p.yaw);
  v3d.transformSource(k, -p.yaw, -(c * p.tx - s * p.ty), -(s * p.tx + c * p.ty), -p.tz);
  v3d.transformSource(k, target.yaw, target.tx, target.ty, target.tz);
  poses[k] = { ...target };
  alignQ[k] = { manual: true };
  $('alignInfo').innerHTML = loadedScans.map((id, j) => j ? describePose(j) : `Nr. 0: ${id} (Bezug)`).join('<br>');
  fillManualInputs();
}
$('mvScan').onchange = fillManualInputs;
document.querySelectorAll('.mv').forEach(b => b.onclick = () => {
  const k = +$('mvScan').value, p = poses[k]; if (!p) return;
  const [axis, sign] = b.dataset.mv.split(','), [dm, ddeg] = $('mvStep').value.split(',').map(Number);
  const t = { ...p };
  if (axis === 'x') t.tx += sign * dm;
  else if (axis === 'y') t.ty += sign * dm;
  else if (axis === 'z') t.tz += sign * dm;
  else t.yaw += sign * ddeg * Math.PI / 180;      // Drehung um den eigenen Standort (tx, ty bleibt)
  setPose(k, t);
});
for (const id of ['mvYaw', 'mvX', 'mvY', 'mvZ']) $(id).onchange = () => {
  const k = +$('mvScan').value; if (!poses[k]) return;
  const v = id => +$(id).value || 0;
  setPose(k, { yaw: v('mvYaw') * Math.PI / 180, tx: v('mvX'), ty: v('mvY'), tz: v('mvZ') });
};
$('btnMvReset').onclick = () => { const k = +$('mvScan').value; if (poses[k]) setPose(k, { yaw: 0, tx: 0, ty: 0, tz: 0 }); };
$('btnMvIcp').onclick = async () => {
  const k = +$('mvScan').value; if (!poses[k]) return;
  const btn = $('btnMvIcp'); btn.disabled = true;
  try {
    const ref = Registration.voxelSample(v3d.chunks, s => s !== k, 0.05, 250000);
    const mov = Registration.voxelSample(v3d.chunks, s => s === k, 0.05);
    const r = await regRun(ref, mov, { init: { yaw: 0, tx: 0, ty: 0, tz: 0 } }, `Scan ${k}`);
    v3d.transformSource(k, r.yaw, r.tx, r.ty, r.tz);
    poses[k] = composePose(r, poses[k]);
    alignQ[k] = { overlap: r.overlap, rmse: r.rmse };
    showAlignInfo();
    toast(r.overlap < 0.3 ? 'Feinausrichtung: kaum Überdeckung – näher hinschieben.' : 'Feinausrichtung fertig.', r.overlap < 0.3 ? 'error' : 'info');
  } catch (e) { toast('Feinausrichtung fehlgeschlagen: ' + e.message); }
  finally { btn.disabled = false; }
};
$('btnAlign').onclick = async () => {
  if (loadedScans.length < 2) { toast('Erst mindestens zwei Scans laden (weitere mit „+“ in der Scanliste).'); return; }
  const btn = $('btnAlign'); btn.disabled = true;
  try {
    for (let k = 1; k < loadedScans.length; k++) {
      const ref = Registration.voxelSample(v3d.chunks, s => s < k, 0.05, 250000);
      const mov = Registration.voxelSample(v3d.chunks, s => s === k, 0.05);
      const r = await regRun(ref, mov, {}, `Scan ${k}/${loadedScans.length - 1}`);
      v3d.transformSource(k, r.yaw, r.tx, r.ty, r.tz);
      poses[k] = composePose(r, poses[k]);
      alignQ[k] = { overlap: r.overlap, rmse: r.rmse, second: r.second };
      console.info('Ausrichtung', k, r);
    }
    $('colorMode').value = '5'; v3d.setColorMode(5); v3d.setView('top');
    showAlignInfo();
    const weak = alignQ.slice(1).some(q => q && q.overlap < 0.3);
    toast(weak ? 'Ausgerichtet, aber mindestens ein Scan überdeckt sich kaum – Ergebnis prüfen.' : 'Scans ausgerichtet.', weak ? 'error' : 'info');
  } catch (e) {
    toast('Ausrichtung fehlgeschlagen: ' + e.message); $('alignInfo').textContent = '';
  } finally { btn.disabled = false; }
};

async function refreshProjects() {
  try {
    const list = await (await rawFetch('/api/projects')).json();
    const sel = $('projSel'), cur = sel.value;
    sel.innerHTML = '<option value="">– Projekt –</option>';
    for (const p of list) {
      const o = document.createElement('option');
      o.value = p.name; o.textContent = `${p.name} (${p.scans.length})`; sel.appendChild(o);
    }
    if (list.some(p => p.name === cur)) sel.value = cur;
  } catch (e) { /* offline */ }
}
$('btnProjSave').onclick = async () => {
  const name = $('projName').value.trim();
  if (!loadedScans.length) { toast('Keine gespeicherten Scans geladen.'); return; }
  if (!name) { toast('Bitte einen Projektnamen eingeben.'); return; }
  const body = { scans: loadedScans.map((id, k) => ({ id,
    pose: { yaw_deg: poses[k].yaw * 180 / Math.PI, tx: poses[k].tx, ty: poses[k].ty, tz: poses[k].tz },
    ...(alignQ[k] && !alignQ[k].manual ? { rmse: alignQ[k].rmse, overlap: alignQ[k].overlap } : {}) })) };
  const r = await api(`/api/projects/${encodeURIComponent(name)}`, { method: 'PUT', body: JSON.stringify(body) });
  if (r.ok) { toast(`Projekt „${name}“ gespeichert.`, 'info'); await refreshProjects(); $('projSel').value = name; }
};
$('btnProjLoad').onclick = async () => {
  const name = $('projSel').value; if (!name) return;
  const r = await api(`/api/projects/${encodeURIComponent(name)}`); if (!r.ok) return;
  const pr = await r.json();
  clearView();
  for (let k = 0; k < pr.scans.length; k++) {
    const s = pr.scans[k];
    $('alignInfo').textContent = `Lade ${k + 1}/${pr.scans.length}: ${s.id} …`;
    await loadScanInto3D(s.id, k > 0, true);
    const p = { yaw: (s.pose.yaw_deg || 0) * Math.PI / 180, tx: s.pose.tx || 0, ty: s.pose.ty || 0, tz: s.pose.tz || 0 };
    if (p.yaw || p.tx || p.ty || p.tz) v3d.transformSource(k, p.yaw, p.tx, p.ty, p.tz);
    poses[k] = p;
    alignQ[k] = typeof s.overlap === 'number' ? { overlap: s.overlap, rmse: s.rmse } : null;
  }
  currentScanId = name.replace(/\s+/g, '_');
  $('projName').value = name;
  if (pr.scans.length > 1) { $('colorMode').value = '5'; v3d.setColorMode(5); }
  v3d.setView('top'); showAlignInfo();
};
$('btnProjDel').onclick = async () => {
  const name = $('projSel').value; if (!name) return;
  if (!confirm(`Projekt „${name}“ löschen? Die Scans selbst bleiben erhalten.`)) return;
  const r = await api(`/api/projects/${encodeURIComponent(name)}`, { method: 'DELETE' });
  if (r.ok) refreshProjects();
};
refreshProjects();

// --- Offsets von Hand ----------------------------------------------------------
async function fillOffsetBox() {
  try {
    const c = await (await rawFetch('/api/config')).json();
    $('offAng').value = (+c.LIDAR.ANGLE_OFFSET).toFixed(3);
    $('offY').value = (+c.GEOMETRY.MODEL_Y_OFFSET).toFixed(1);
    $('offZ').value = (+c.GEOMETRY.MODEL_Z_OFFSET).toFixed(1);
    $('offK').value = (+(c.GEOMETRY.Z_SCALE ?? 1)).toFixed(4);
    $('offH').value = (+(c.GEOMETRY.HALF_YAW ?? 0)).toFixed(2);
    const scans = await (await rawFetch('/api/scans')).json();
    const sel = $('offScan'), cur = sel.value; sel.innerHTML = '';
    for (const s of scans.filter(s => (s.scan_angle || 0) > 180 || s.mode === 'B')) {
      const o = document.createElement('option'); o.value = s.id; o.textContent = s.id; sel.appendChild(o);
    }
    if (cur) sel.value = cur;
  } catch (e) { /* offline */ }
}
$('offBox').addEventListener('toggle', () => { if ($('offBox').open) fillOffsetBox(); });
const offVals = () => ({ angle_offset: +$('offAng').value, model_y_offset: +$('offY').value || 0,
                         model_z_offset: +$('offZ').value || 0, z_scale: +$('offK').value || 1,
                         half_yaw: +$('offH').value || 0 });
// Naht-Ansicht: Anfang (orange) und Ende (türkis) des Scans mit den Werten aus den Feldern
let seamMode = 'seam';   // 'seam' = Anfang/Ende, 'halves' = ganzer Scan, vordere/hintere LiDAR-Hälfte
async function showSeam(reuse, mode) {
  if (mode) seamMode = mode;
  const id = $('offScan').value; if (!id) { toast('Kein Scan mit Überlappung vorhanden.'); return; }
  const meta = (await (await rawFetch('/api/scans')).json()).find(s => s.id === id) || {};
  const sa = meta.scan_angle || 184, ov = meta.overlap_deg || 4;
  let buf = reuse && seamView && seamView.id === id ? seamView.buf : null;
  if (!buf) {
    const r = await rawFetch(`/api/scans/${id}/raw`);
    if (!r.ok) { toast('Keine Rohdaten für diesen Scan.'); return; }
    buf = await r.arrayBuffer();
  }
  const keepView = reuse && seamView;
  clearView();
  seamView = { id, buf };
  const v = offVals();
  worker.postMessage({ type: 'init', angle_offset: v.angle_offset, model_y: v.model_y_offset,
                       model_z: v.model_z_offset, z_scale: v.z_scale, half_yaw: v.half_yaw });
  document.querySelector('input[name="view"][value="3d"]').checked = true; applyViewMode();
  await new Promise(res => { rawWaiters.set(0, res);
    worker.postMessage({ type: 'rawScan', buf: buf.slice(0), scanId: id, src: 0,
      ...(seamMode === 'halves' ? { halves: { stride: 2 } } : { seam: { ov, sa } }) }); });
  $('colorMode').value = '5'; v3d.setColorMode(5);
  if (!keepView) v3d.setView('top');
  if (seamMode === 'halves') {
    $('offOut').innerHTML = `Ganzer Scan ${id}: orange = vordere, türkis = hintere Hälfte des LiDAR. ` +
      (sa >= 350 ? 'Beide Hälften sehen den ganzen Raum – Wände und Kanten müssen überall deckungsgleich sein. '
                 : `Achtung: bei ${sa}° sehen die Hälften verschiedene Raumteile und überdecken sich nur an der Naht. `) +
      'Werte ändern, die Ansicht rechnet sofort neu; „Mit Scan prüfen“ gibt die Abweichung als Zahl.';
    return;
  }
  $('offOut').innerHTML = `Naht von ${id}: orange = Anfang (0…${ov}°), türkis = Ende (${sa - ov}…${sa}°). ` +
    'Werte ändern, die Ansicht rechnet sofort neu; passt es, „Übernehmen“. „Ansicht leeren“ beendet die Naht-Ansicht.';
}
let seamTimer = null;
for (const id of ['offAng', 'offY', 'offZ', 'offK', 'offH']) $(id).addEventListener('input', () => {
  if (!seamView) return;
  clearTimeout(seamTimer); seamTimer = setTimeout(() => showSeam(true), 350);
});
$('btnSeam').onclick = () => showSeam(false, 'seam');
$('btnHalves').onclick = () => showSeam(false, 'halves');
$('btnOffCheck').onclick = async () => {
  const id = $('offScan').value; if (!id) { toast('Kein Scan mit Überlappung vorhanden.'); return; }
  $('offOut').textContent = 'rechnet …';
  const r = await api('/api/calibrate/check', { method: 'POST', body: JSON.stringify({ scan_id: id, ...offVals() }) });
  if (!r.ok) { $('offOut').textContent = ''; return; }
  const j = await r.json(), t = j.test.loop_closure_mm, c = j.current.loop_closure_mm;
  const verdict = t === null ? '' : t <= 15 ? ' – Winkel gut' : t <= 30 ? ' – Winkel mäßig' : ' – Winkel passt nicht';
  $('offOut').innerHTML = `Überlappung mit diesen Werten: <b>${t ?? '–'} mm</b>${verdict}<br>` +
    `mit dem aktuellen Winkel (${(+j.current.angle_offset).toFixed(2)}°): ${c ?? '–'} mm<br>` +
    'y und z lassen sich an der Naht nicht prüfen.' +
    (j.halves_test_mm != null ? `<br><b>Ganzer Scan, vordere gegen hintere Hälfte: ${j.halves_test_mm} mm</b> ` +
      `(gespeicherte Werte: ${j.halves_current_mm} mm) – kleiner ist besser, bewertet auch y und Hälften-Verdrehung.` : '');
};
$('btnOffApply').onclick = async () => {
  const v = offVals();
  if (!isFinite(v.angle_offset)) { toast('Winkel fehlt.'); return; }
  if (!confirm(`Offsets übernehmen? Winkel ${v.angle_offset}°, y ${v.model_y_offset} mm, z ${v.model_z_offset} mm, Drehfaktor ${v.z_scale}, Hälften-Verdrehung ${v.half_yaw}°.\n` +
               'Gespeicherte Scans werden beim nächsten Öffnen damit gerechnet.')) return;
  const r = await api('/api/config/apply', { method: 'POST', body: JSON.stringify(v) });
  if (r.ok) { await refreshWorkerCfg(); toast('Offsets übernommen – Scan neu öffnen, um sie zu sehen.', 'info'); }
};

$('btnRefresh').onclick = refreshScans;
$('btnConfirmHome').onclick = async () => {
  if (!confirm('Steht der Drehkopf in der Ausgangslage (Startposition des Scans)?')) return;
  const r = await api('/api/stepper/confirm_home', { method: 'POST' });
  if (r.ok) toast('Ausgangslage übernommen.', 'info');
};
refreshScans();
