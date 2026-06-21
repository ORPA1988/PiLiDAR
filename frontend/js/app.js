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
let lidarResDeg = 0;   // vertikale Winkelauflösung [°], live aus Frames gemessen

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
      // Winkelauflösung aus benachbarten Punkten eines Pakets messen (Wrap ausschließen)
      if (m.a && m.a.length >= 2) {
        const step = Math.abs(m.a[1] - m.a[0]);
        if (step > 0 && step < 5) lidarResDeg = step;
      }
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
  if (viewMode === '3d') v3d._resize();
});

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
    const s = await (await api('/api/status')).json();
    const b = $('state'); b.textContent = s.state;
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
    const sys = await (await api('/api/system/stats')).json();
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
  const scans = await (await api('/api/scans')).json();
  const ul = $('scanList'); ul.innerHTML = '';
  for (const s of scans) {
    const qa  = (s.qa && s.qa.status) || '';
    const ann = (s.annotation || '').replace(/"/g, '&quot;');
    const li  = document.createElement('li');
    li.innerHTML = `
      <div class="row">
        <span><span class="dot ${qa}"></span><b>${s.id}</b></span>
        <span style="display:flex;gap:6px;align-items:center">
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
$('btnRefresh').onclick = refreshScans;
refreshScans();
