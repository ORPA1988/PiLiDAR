/* Web Worker: rechnet Roh-Frames -> 3D-Punktwolke (mm -> m).
 * Identische Geometrie wie backend/pointcloud.py:
 *   Polar -> X-Z-Ebene (y=0) -> Y-Rotation(angle_offset) -> +position_offset
 *   -> Z-Rotation(-z_angle).
 * Hält alle Punkte für den PLY-Export und sendet Batches an den 3D-Viewer.
 */
'use strict';

let cfg = { angle_offset: 0, model_y: 0, model_z: 0, dist_min: 30, dist_max: 25000 };

// wachsende Speicher für Export
let allXYZ = [];   // Array von Float32Array-Batches
let allINT = [];
let totalPoints = 0;

function deg2rad(d) { return d * Math.PI / 180; }

function transformFrame(angles, dists, inten, zAngle) {
  const ao = deg2rad(cfg.angle_offset);
  const caO = Math.cos(ao), saO = Math.sin(ao);
  const za = deg2rad(-zAngle);
  const cz = Math.cos(za), sz = Math.sin(za);
  const oy = cfg.model_y, oz = cfg.model_z;

  const xs = [], ints = [];
  for (let i = 0; i < dists.length; i++) {
    const d = dists[i];
    if (d < cfg.dist_min || d > cfg.dist_max) continue;
    const a = deg2rad(angles[i]);
    // Punkt in Scan-Ebene (X-Z, y=0)
    let px = d * Math.cos(a);
    let py = 0.0;
    let pz = d * Math.sin(a);
    // Y-Rotation (angle_offset): x' = x*c + z*s ; z' = -x*s + z*c
    let rx = px * caO + pz * saO;
    let rz = -px * saO + pz * caO;
    px = rx; pz = rz;
    // + position_offset (x=0, y=model_y, z=model_z)
    py += oy; pz += oz;
    // Z-Rotation(-z_angle): x'' = x*cz - y*sz ; y'' = x*sz + y*cz
    const fx = px * cz - py * sz;
    const fy = px * sz + py * cz;
    const fz = pz;
    xs.push(fx, fy, fz);
    ints.push(inten[i]);
  }
  return { xyz: Float32Array.from(xs.map(v => v / 1000.0)), inten: Float32Array.from(ints) };
}

self.onmessage = (ev) => {
  const msg = ev.data;
  if (msg.type === 'init') {
    cfg = { ...cfg, ...msg };
  } else if (msg.type === 'frame') {
    const { xyz, inten } = transformFrame(msg.a, msg.d, msg.i, msg.z);
    if (xyz.length === 0) return;
    // Master-Kopie für den Export behalten
    allXYZ.push(xyz); allINT.push(inten);
    totalPoints += xyz.length / 3;
    // KOPIE an den Viewer senden (kein Transfer -> Master bleibt gültig)
    self.postMessage({ type: 'points', xyz: xyz.slice(), inten: inten.slice(), total: totalPoints });
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
