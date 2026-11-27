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
const ASSET_VERSION = '20261127m';  // wie ?v= in index.html – erzwingt neue Skripte nach Updates
const worker = new Worker(`js/scan.worker.js?v=${ASSET_VERSION}`);

let ws = null;
let viewMode = '2d';
let totalPoints = 0;
let lidarResDeg = 0;
let lastState = null;  // für das automatische Nachladen der Scan-Liste

// Geometrie-Parameter für den Worker aus der aktuellen Konfiguration
async function refreshWorkerCfg() {
  try {
    const c = await (await rawFetch('/api/config')).json();
    worker.postMessage({ type: 'init', angle_offset: c.LIDAR.ANGLE_OFFSET,
      model_y: c.GEOMETRY.MODEL_Y_OFFSET, model_z: c.GEOMETRY.MODEL_Z_OFFSET,
      dist_min: c.LIDAR.DISTANCE_MIN_MM, dist_max: c.LIDAR.DISTANCE_MAX_MM });
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
    $('stPts').textContent = m.total.toLocaleString('de-DE');
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
    worker.postMessage({ type: 'batch', buf }, [buf]);
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
  if (r.ok && confirm(`Offsets übernehmen (Residuum ${r.residual_mm.toFixed(1)} mm)?`))
    await api('/api/config/apply', { method: 'POST', body: JSON.stringify({
      angle_offset: r.angle_offset, model_y_offset: r.model_y_offset, model_z_offset: r.model_z_offset }) });
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
}
$('btnClear').onclick = clearView;
// PLY: sichtbare Punkte (Höhenschnitt beachtet) direkt aus dem Viewer
$('btnPly').onclick = () => {
  const v = thinPoints(v3d.visiblePoints(), +$('thinCell').value);
  if (!v.total) { toast('Keine Punkte in der 3D-Ansicht.'); return; }
  exportPLY(v.xyz, v.inten, v.total);
};

$('btnLas').onclick = () => {
  const v = thinPoints(v3d.visiblePoints(), +$('thinCell').value);
  if (!v.total) { toast('Keine Punkte in der 3D-Ansicht.'); return; }
  exportLAS(v.xyz, v.inten, v.total, `${currentScanId || 'pilidar'}.las`);
};
$('btnPng').onclick = () => {
  const a = document.createElement('a');
  a.href = $('view3d').toDataURL('image/png');
  a.download = `${currentScanId || 'pilidar'}_ansicht.png`; a.click();
};

// --- 3D-Darstellung ----------------------------------------------------
$('colorMode').onchange = () => v3d.setColorMode(+$('colorMode').value);
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
let loadedScans = [];       // überlagerte Scans (Index = Farbe im Modus „Scan-Nummer“)
async function loadScanInto3D(scanId, add = false) {
  currentScanId = scanId;
  if (!add || !loadedScans.length) { clearView(); loadedScans = []; }
  const src = loadedScans.length;
  loadedScans.push(scanId);
  if (add) toast(`Scan ${scanId} überlagert (Nr. ${src}). Farbe „Scan-Nummer“ trennt die Scans.`, 'info');
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
      worker.postMessage({ type: 'rawScan', buf, scanId, src }, [buf]);
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
      v3d.addPoints(xyz.slice(i * 3, (i + m) * 3), inten.slice(i, i + m));
    }
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
  if (r.ok && confirm(`Aus Scan ${scanId}: Winkel ${r.angle_offset.toFixed(2)}°, Achsversatz ${r.model_y_offset.toFixed(1)} mm ` +
                      `(Rest ${r.residual_mm.toFixed(1)} mm). Übernehmen? Die Höhe bleibt unverändert.`)) {
    await api('/api/config/apply', { method: 'POST', body: JSON.stringify({
      angle_offset: r.angle_offset, model_y_offset: r.model_y_offset }) });
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

$('btnRefresh').onclick = refreshScans;
$('btnConfirmHome').onclick = async () => {
  if (!confirm('Steht der Drehkopf in der Ausgangslage (Startposition des Scans)?')) return;
  const r = await api('/api/stepper/confirm_home', { method: 'POST' });
  if (r.ok) toast('Ausgangslage übernommen.', 'info');
};
refreshScans();
