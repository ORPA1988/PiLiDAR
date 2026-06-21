/* App-Logik: WebSocket-Rohdaten, Worker-Berechnung, 2D/3D-Viewer, Steuerung. */
'use strict';

const $ = (id) => document.getElementById(id);
const api = (path, opts) => fetch(path, { headers: { 'Content-Type': 'application/json' }, ...opts });

const v2d = new Viewer2D($('view2d'));
const v3d = new Viewer3D($('view3d'));
const worker = new Worker('js/scan.worker.js');

let ws = null;
let viewMode = '2d';
let totalPoints = 0;

v2d.start();

// --- Worker-Rückgaben -------------------------------------------------
worker.onmessage = (ev) => {
  const m = ev.data;
  if (m.type === 'points') {
    v3d.addPoints(m.xyz, m.inten);
    totalPoints = m.total;
    $('stPts').textContent = totalPoints.toLocaleString('de-DE');
  } else if (m.type === 'exportData') {
    exportPLY(m.xyz, m.inten, m.total);
  }
};

// --- WebSocket --------------------------------------------------------
function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws/scan`);
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'init') {
      worker.postMessage({ type: 'init', ...m });
    } else if (m.type === 'frame') {
      if (viewMode === '2d') v2d.addFrame(m.a, m.d);
      worker.postMessage({ type: 'frame', a: m.a, d: m.d, i: m.i, z: m.z });
    }
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
  const r = await (await api('/api/calibrate/rotation', { method: 'POST' })).json();
  $('calOut').textContent = JSON.stringify(r, null, 1);
  if (r.ok && confirm(`Getriebeverhältnis ${r.gear_ratio.toFixed(4)} übernehmen?`))
    await api('/api/config/apply', { method: 'POST', body: JSON.stringify({ gear_ratio: r.gear_ratio }) });
};
$('btnCalOff').onclick = async () => {
  $('calOut').textContent = 'Offset-Kalibrierung läuft (>180°-Lauf)…';
  const r = await (await api('/api/calibrate/offset', { method: 'POST' })).json();
  $('calOut').textContent = JSON.stringify(r, null, 1);
  if (r.ok && confirm(`Offsets übernehmen (Residuum ${r.residual_mm.toFixed(1)} mm)?`))
    await api('/api/config/apply', { method: 'POST', body: JSON.stringify({
      angle_offset: r.angle_offset, model_y_offset: r.model_y_offset, model_z_offset: r.model_z_offset }) });
};

document.querySelectorAll('input[name=view]').forEach(r => r.onchange = () => {
  viewMode = document.querySelector('input[name=view]:checked').value;
  $('view2d').classList.toggle('hidden', viewMode !== '2d');
  $('view3d').classList.toggle('hidden', viewMode !== '3d');
});

function clearView() {
  v2d.clear(); v3d.clear(); worker.postMessage({ type: 'clear' });
  totalPoints = 0; $('stPts').textContent = '0';
}
$('btnClear').onclick = clearView;
$('btnPly').onclick = () => worker.postMessage({ type: 'export' });

// --- Status-Polling ---------------------------------------------------
async function poll() {
  try {
    const s = await (await api('/api/status')).json();
    const b = $('state'); b.textContent = s.state;
    b.className = 'badge ' + (s.state === 'idle' ? 'ok' : s.state === 'scanning' ? 'warn' : '');
    $('stAngle').textContent = s.angle;
    $('stRate').textContent = Math.round(s.stats.packet_rate);
    $('stCrc').textContent = (s.stats.crc_error_rate * 100).toFixed(2);
  } catch (e) { /* offline */ }
}
setInterval(poll, 700); poll();

// --- Scan-Liste -------------------------------------------------------
async function refreshScans() {
  const scans = await (await api('/api/scans')).json();
  const ul = $('scanList'); ul.innerHTML = '';
  for (const s of scans) {
    const qa = (s.qa && s.qa.status) || '';
    const li = document.createElement('li');
    li.innerHTML = `<div class="row"><span><span class="dot ${qa}"></span><b>${s.id}</b></span>
      <a href="/api/scans/${s.id}/download">ZIP</a></div>
      <div style="color:#7d8794">Modus ${s.mode || '?'} · ${s.n_points || 0} Pkt · QA ${qa || '–'}</div>`;
    ul.appendChild(li);
  }
}
$('btnRefresh').onclick = refreshScans;
refreshScans();
