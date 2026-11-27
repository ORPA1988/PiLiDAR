/* PLY-Export im Browser aus den Worker-Daten (binär, little-endian). */
'use strict';

function exportPLY(xyz, inten, total) {
  const header = [
    'ply', 'format binary_little_endian 1.0', `element vertex ${total}`,
    'property float x', 'property float y', 'property float z',
    'property uchar red', 'property uchar green', 'property uchar blue',
    'end_header', ''
  ].join('\n');
  const body = new ArrayBuffer(total * 15);
  const dv = new DataView(body);
  for (let i = 0, o = 0; i < total; i++, o += 15) {
    dv.setFloat32(o, xyz[i * 3], true);
    dv.setFloat32(o + 4, xyz[i * 3 + 1], true);
    dv.setFloat32(o + 8, xyz[i * 3 + 2], true);
    const g = Math.max(0, Math.min(255, Math.round(inten[i] || 0)));
    dv.setUint8(o + 12, g); dv.setUint8(o + 13, g); dv.setUint8(o + 14, g);
  }
  const blob = new Blob([header, body], { type: 'application/octet-stream' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `pilidar_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.ply`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}

/* LAS 1.2, Punktformat 0 (x, y, z als skalierte Ganzzahlen in mm, Intensität) –
 * Standardformat für CloudCompare, QGIS, PDAL usw. */
function exportLAS(xyz, inten, total, name) {
  let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity;
  for (let i = 0; i < total; i++) {
    const x = xyz[i * 3], y = xyz[i * 3 + 1], z = xyz[i * 3 + 2];
    if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; if (z < z0) z0 = z; if (z > z1) z1 = z;
  }
  const HDR = 227, REC = 20, scale = 0.001;
  const buf = new ArrayBuffer(HDR + total * REC), dv = new DataView(buf), u8 = new Uint8Array(buf);
  const str = (o, s, n) => { for (let i = 0; i < n; i++) u8[o + i] = i < s.length ? s.charCodeAt(i) : 0; };
  str(0, 'LASF', 4);
  u8[24] = 1; u8[25] = 2;                                   // Version 1.2
  str(26, 'PiLiDAR', 32); str(58, 'PiLiDAR 2.0 Web', 32);
  const now = new Date(), doy = Math.floor((now - new Date(now.getFullYear(), 0, 0)) / 864e5);
  dv.setUint16(90, doy, true); dv.setUint16(92, now.getFullYear(), true);
  dv.setUint16(94, HDR, true); dv.setUint32(96, HDR, true); // Kopfgröße, Offset zu den Punkten
  dv.setUint32(100, 0, true); u8[104] = 0; dv.setUint16(105, REC, true);
  dv.setUint32(107, total, true); dv.setUint32(111, total, true);   // Punkte gesamt / Rückläufer 1
  for (let k = 0; k < 3; k++) dv.setFloat64(131 + k * 8, scale, true);   // Skalierung
  for (let k = 0; k < 3; k++) dv.setFloat64(155 + k * 8, 0, true);       // Offset
  dv.setFloat64(179, x1, true); dv.setFloat64(187, x0, true); dv.setFloat64(195, y1, true);
  dv.setFloat64(203, y0, true); dv.setFloat64(211, z1, true); dv.setFloat64(219, z0, true);
  for (let i = 0, o = HDR; i < total; i++, o += REC) {
    dv.setInt32(o, Math.round(xyz[i * 3] / scale), true);
    dv.setInt32(o + 4, Math.round(xyz[i * 3 + 1] / scale), true);
    dv.setInt32(o + 8, Math.round(xyz[i * 3 + 2] / scale), true);
    dv.setUint16(o + 12, Math.max(0, Math.min(65535, Math.round((inten[i] || 0) * 257))), true);
    u8[o + 14] = 0x09;                                      // Rückläufer 1 von 1
  }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([buf], { type: 'application/octet-stream' }));
  a.download = name || `pilidar_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.las`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}

/* Voxel-Ausdünnung: je Würfel (Kantenlänge cell [m]) ein Punkt (Mittelwert). */
function thinPoints(v, cell) {
  if (!(cell > 0)) return v;
  const B = 131072, map = new Map();
  for (let i = 0; i < v.total; i++) {
    const x = v.xyz[i * 3], y = v.xyz[i * 3 + 1], z = v.xyz[i * 3 + 2];
    const key = (Math.floor(x / cell) + B) * 17179869184 + (Math.floor(y / cell) + B) * 131072 + (Math.floor(z / cell) + B);
    const a = map.get(key);
    if (a) { a[0] += x; a[1] += y; a[2] += z; a[3] += v.inten[i]; a[4]++; }
    else map.set(key, [x, y, z, v.inten[i], 1]);
  }
  const n = map.size, xyz = new Float32Array(n * 3), inten = new Float32Array(n);
  let k = 0;
  for (const a of map.values()) {
    xyz[k * 3] = a[0] / a[4]; xyz[k * 3 + 1] = a[1] / a[4]; xyz[k * 3 + 2] = a[2] / a[4]; inten[k] = a[3] / a[4]; k++;
  }
  return { xyz, inten, total: n };
}

/* Ausreißer entfernen: Punkt bleibt, wenn in seinem Würfel und den 26 Nachbarn
 * (Kante = cell) mindestens minN Punkte liegen. Entfernt Streupunkte (Staub, Spiegelungen,
 * Kantentreffer „in der Luft“), lässt dünn abgetastete ferne Wände stehen. */
function removeOutliers(v, cell = 0.05, minN = 4) {
  const B = 131072, cnt = new Map(), keys = new Float64Array(v.total);
  const K1 = 17179869184, K2 = 131072;
  for (let i = 0; i < v.total; i++) {
    const k = (Math.floor(v.xyz[i * 3] / cell) + B) * K1 + (Math.floor(v.xyz[i * 3 + 1] / cell) + B) * K2 + (Math.floor(v.xyz[i * 3 + 2] / cell) + B);
    keys[i] = k; cnt.set(k, (cnt.get(k) || 0) + 1);
  }
  const ok = new Map();                                   // Ergebnis je Würfel zwischenspeichern
  const keep = new Uint8Array(v.total); let n = 0;
  for (let i = 0; i < v.total; i++) {
    const k = keys[i];
    let r = ok.get(k);
    if (r === undefined) {
      let s = 0;
      for (let dx = -1; dx <= 1 && s < minN; dx++) for (let dy = -1; dy <= 1 && s < minN; dy++) for (let dz = -1; dz <= 1; dz++) {
        s += cnt.get(k + dx * K1 + dy * K2 + dz) || 0; if (s >= minN) break;
      }
      r = s >= minN; ok.set(k, r);
    }
    if (r) { keep[i] = 1; n++; }
  }
  const xyz = new Float32Array(n * 3), inten = new Float32Array(n);
  for (let i = 0, k = 0; i < v.total; i++) if (keep[i]) {
    xyz[k * 3] = v.xyz[i * 3]; xyz[k * 3 + 1] = v.xyz[i * 3 + 1]; xyz[k * 3 + 2] = v.xyz[i * 3 + 2]; inten[k] = v.inten[i]; k++;
  }
  return { xyz, inten, total: n, removed: v.total - n };
}

window.removeOutliers = removeOutliers;
window.exportPLY = exportPLY;
window.thinPoints = thinPoints;
window.exportLAS = exportLAS;
