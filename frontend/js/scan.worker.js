/* Web Worker: rechnet Roh-Frames -> 3D-Punktwolke (mm -> m).
 * Identische Geometrie wie backend/pointcloud.py:
 *   Polar -> X-Z-Ebene (y=0) -> Y-Rotation(angle_offset) -> +position_offset
 *   -> Z-Rotation(-z_angle).
 * Quellen: Live-Bündel vom WebSocket (Binärformat, siehe backend/app.py _pack_frames)
 * und gespeicherte Scans als Rohdaten (/api/scans/<id>/raw). Gespeicherte Scans
 * werden damit immer mit den AKTUELLEN Offsets dargestellt.
 * Hält alle Punkte für den PLY-Export und sendet Blöcke an den 3D-Viewer.
 */
'use strict';

let cfg = { angle_offset: 0, model_y: 0, model_z: 0, dist_min: 30, dist_max: 25000 };

let allXYZ = [];   // Float32Array-Blöcke für den Export
let allINT = [];
let totalPoints = 0;

const D2R = Math.PI / 180;

/* Frames [f0, f0+n) aus getypten Arrays transformieren.
 * z: Float32Array(N), a: Float32Array(N*12), d: Uint16Array(N*12), it: Uint8Array(N*12) */
function transformBlock(z, a, d, it, f0, n) {
  const ao = cfg.angle_offset * D2R, caO = Math.cos(ao), saO = Math.sin(ao);
  const oy = cfg.model_y, oz = cfg.model_z, dmin = cfg.dist_min, dmax = cfg.dist_max;
  const xyz = new Float32Array(n * 12 * 3);
  const inten = new Float32Array(n * 12);
  const za = new Float32Array(n * 12);              // Drehwinkel je Punkt (Diagnose-Farbe)
  let k = 0;
  for (let f = f0; f < f0 + n; f++) {
    const zr = -z[f] * D2R, cz = Math.cos(zr), sz = Math.sin(zr);
    for (let j = f * 12; j < f * 12 + 12; j++) {
      const dd = d[j];
      if (dd < dmin || dd > dmax) continue;
      const ang = a[j] * D2R;
      const px0 = dd * Math.cos(ang), pz0 = -dd * Math.sin(ang);   // 270° -> +Z (oben)
      const px = px0 * caO + pz0 * saO;                            // Y-Rotation
      const pz = -px0 * saO + pz0 * caO + oz;
      const py = oy;
      xyz[k * 3] = (px * cz - py * sz) / 1000;                     // Z-Rotation, mm -> m
      xyz[k * 3 + 1] = (px * sz + py * cz) / 1000;
      xyz[k * 3 + 2] = pz / 1000;
      inten[k] = it[j];
      za[k] = z[f];
      k++;
    }
  }
  return { xyz: xyz.subarray(0, k * 3), inten: inten.subarray(0, k), za: za.subarray(0, k) };
}

function emit(res, extra) {
  if (res.inten.length === 0) return;
  const xyz = res.xyz.slice(), inten = res.inten.slice();   // eigene Kopien (kompakt)
  allXYZ.push(xyz); allINT.push(inten);
  totalPoints += inten.length;
  // Kopie an den Viewer; Master bleibt für den Export
  self.postMessage({ type: 'points', xyz: xyz.slice(), inten: inten.slice(), za: res.za.slice(),
                     total: totalPoints, ...extra });
}

/* Puffer mit Kopf uint32 n und danach Feldern in fester Reihenfolge zerlegen. */
function unpack(buf, withSpeed) {
  const n = new Uint32Array(buf, 0, 1)[0];
  let o = 4;
  const z = new Float32Array(buf, o, n); o += n * 4;
  if (withSpeed) o += n * 4;                       // Drehzahl (für die 3D-Rechnung nicht nötig)
  const a = new Float32Array(buf, o, n * 12); o += n * 48;
  const d = new Uint16Array(buf, o, n * 12); o += n * 24;
  const it = new Uint8Array(buf, o, n * 12);
  return { n, z, a, d, it };
}

self.onmessage = (ev) => {
  const msg = ev.data;
  if (msg.type === 'init') {
    for (const k of ['angle_offset', 'model_y', 'model_z', 'dist_min', 'dist_max'])
      if (typeof msg[k] === 'number') cfg[k] = msg[k];
  } else if (msg.type === 'batch') {               // Live: Bündel vom WebSocket
    const r = unpack(msg.buf, true);
    emit(transformBlock(r.z, r.a, r.d, r.it, 0, r.n), {});
  } else if (msg.type === 'rawScan') {             // gespeicherter Scan (Rohdaten)
    const r = unpack(msg.buf, false);
    const STEP = 20000;                            // Frames je Block
    for (let f = 0; f < r.n; f += STEP) {
      const m = Math.min(STEP, r.n - f);
      emit(transformBlock(r.z, r.a, r.d, r.it, f, m), { progress: (f + m) / r.n, scanId: msg.scanId, src: msg.src || 0 });
    }
    self.postMessage({ type: 'rawDone', scanId: msg.scanId, src: msg.src || 0, total: totalPoints });
  } else if (msg.type === 'clear') {
    allXYZ = []; allINT = []; totalPoints = 0;
  } else if (msg.type === 'export') {
    const xyz = new Float32Array(totalPoints * 3);
    const it = new Float32Array(totalPoints);
    let o = 0, io = 0;
    for (let b = 0; b < allXYZ.length; b++) {
      xyz.set(allXYZ[b], o); o += allXYZ[b].length;
      it.set(allINT[b], io); io += allINT[b].length;
    }
    self.postMessage({ type: 'exportData', xyz, inten: it, total: totalPoints }, [xyz.buffer, it.buffer]);
  }
};
