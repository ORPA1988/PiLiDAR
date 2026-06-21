/* PLY-Export (ASCII) im Browser aus den Worker-Daten. */
'use strict';

function exportPLY(xyz, inten, total) {
  const lines = [
    'ply', 'format ascii 1.0', `element vertex ${total}`,
    'property float x', 'property float y', 'property float z',
    'property uchar red', 'property uchar green', 'property uchar blue',
    'end_header'
  ];
  const parts = [lines.join('\n') + '\n'];
  const chunk = [];
  for (let i = 0; i < total; i++) {
    const g = Math.max(0, Math.min(255, Math.round(inten[i] || 0)));
    chunk.push(`${xyz[i * 3].toFixed(4)} ${xyz[i * 3 + 1].toFixed(4)} ${xyz[i * 3 + 2].toFixed(4)} ${g} ${g} ${g}`);
    if (chunk.length >= 5000) { parts.push(chunk.join('\n') + '\n'); chunk.length = 0; }
  }
  if (chunk.length) parts.push(chunk.join('\n') + '\n');
  const blob = new Blob(parts, { type: 'application/octet-stream' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `pilidar_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.ply`;
  a.click();
  URL.revokeObjectURL(a.href);
}

window.exportPLY = exportPLY;
