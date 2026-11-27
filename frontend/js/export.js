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

window.exportPLY = exportPLY;
