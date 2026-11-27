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
const ASSET_VERSION = '20261127b';  // wie ?v= in index.html – erzwingt neue Skripte nach Updates
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
    v3d.addPoints(m.xyz, m.inten);
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
$('btnPly').onclick = () => worker.postMessage({ type: 'export' });

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
          <a href="/api/scans/${s.id}/download">ZIP</a>
          <button class="btn-del" data-id="${s.id}">&#128465;</button>
        </span>
      </div>
      <div style="color:#7d8794;margin-top:2px">Modus ${s.mode || '?'} &middot; ${(s.n_points || 0).toLocaleString('de-DE')} Pkt &middot; QA ${qa || '–'}</div>
      <div class="ann-row">
        <input class="ann-input" type="text" placeholder="Anmerkung…" value="${ann}" />
        <button class="ann-save" data-id="${s.id}">&#10003;</button>
      </div>`;
    ul.appendChild(li);
  }
  ul.querySelectorAll('.btn-view3d').forEach(btn => {
    btn.onclick = () => loadScanInto3D(btn.dataset.id);
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
async function loadScanInto3D(scanId) {
  clearView();
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
      worker.postMessage({ type: 'rawScan', buf, scanId }, [buf]);
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

$('btnRefresh').onclick = refreshScans;
$('btnConfirmHome').onclick = async () => {
  if (!confirm('Steht der Drehkopf in der Ausgangslage (Startposition des Scans)?')) return;
  const r = await api('/api/stepper/confirm_home', { method: 'POST' });
  if (r.ok) toast('Ausgangslage übernommen.', 'info');
};
refreshScans();
