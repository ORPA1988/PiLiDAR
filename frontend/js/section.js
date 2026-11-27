/* Schnittansicht: senkrechter Schnitt durch die Punktwolke.
 * Schnittebene: durch den Punkt (x0, y0), Richtung dirDeg (0° = +x, 90° = +y), Dicke thick [m].
 * Ergebnis: Punkte als (s, z) – s = Strecke entlang der Schnittlinie, z = Höhe.
 * Dazu Vorschau (Canvas, 0,5-m-Raster), SVG und DXF (Schicht SCHNITT, Meter).
 */
'use strict';

const Section = (() => {

  function compute(viewer, opts = {}) {
    const chunks = viewer.chunks;
    if (!chunks.length) throw new Error('Keine Punktwolke geladen');
    const x0 = opts.x0 || 0, y0 = opts.y0 || 0, thick = opts.thick || 0.05;
    const a = (opts.dirDeg || 0) * Math.PI / 180, ux = Math.cos(a), uy = Math.sin(a);
    const half = thick / 2, out = [];
    for (const c of chunks) {
      const p = c.pos;
      for (let i = 0; i < p.length; i += 3) {
        const dx = p[i] - x0, dy = p[i + 1] - y0;
        if (Math.abs(-dx * uy + dy * ux) > half) continue;      // Abstand quer zur Schnittlinie
        out.push(dx * ux + dy * uy, p[i + 2]);
      }
    }
    if (out.length < 4) throw new Error('Im Schnitt liegen keine Punkte – Lage oder Dicke ändern');
    const pts = Float32Array.from(out);
    let sMin = Infinity, sMax = -Infinity, zMin = Infinity, zMax = -Infinity;
    for (let i = 0; i < pts.length; i += 2) {
      if (pts[i] < sMin) sMin = pts[i]; if (pts[i] > sMax) sMax = pts[i];
      if (pts[i + 1] < zMin) zMin = pts[i + 1]; if (pts[i + 1] > zMax) zMax = pts[i + 1];
    }
    return { pts, n: pts.length / 2, sMin, sMax, zMin, zMax, x0, y0, dirDeg: opts.dirDeg || 0, thick };
  }

  /* je Rasterzelle ein Punkt (für kompakte Exporte) */
  function thin(res, cell = 0.01) {
    const seen = new Set(), out = [];
    for (let i = 0; i < res.pts.length; i += 2) {
      const key = Math.floor(res.pts[i] / cell) * 100003 + Math.floor(res.pts[i + 1] / cell);
      if (seen.has(key)) continue;
      seen.add(key); out.push(res.pts[i], res.pts[i + 1]);
    }
    return out;
  }

  function draw(canvas, res) {
    const ctx = canvas.getContext('2d'), W = canvas.width, H = canvas.height, m = 36;
    ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, W, H);
    const s = Math.min((W - 2 * m) / (res.sMax - res.sMin || 1), (H - 2 * m) / (res.zMax - res.zMin || 1));
    const X = v => m + (v - res.sMin) * s, Y = v => H - m - (v - res.zMin) * s;
    ctx.strokeStyle = '#e3e3e3'; ctx.fillStyle = '#888'; ctx.font = '11px system-ui'; ctx.lineWidth = 1;
    for (let g = Math.ceil(res.sMin / 0.5) * 0.5; g <= res.sMax; g += 0.5) {
      ctx.beginPath(); ctx.moveTo(X(g), m - 6); ctx.lineTo(X(g), H - m + 6); ctx.stroke();
      if (Math.abs(g - Math.round(g)) < 1e-6) ctx.fillText(g.toFixed(0) + ' m', X(g) - 10, H - m + 20);
    }
    for (let g = Math.ceil(res.zMin / 0.5) * 0.5; g <= res.zMax; g += 0.5) {
      ctx.beginPath(); ctx.moveTo(m - 6, Y(g)); ctx.lineTo(W - m + 6, Y(g)); ctx.stroke();
      ctx.fillText(g.toFixed(1), 4, Y(g) + 4);
    }
    ctx.fillStyle = '#111';
    const px = Math.max(1, Math.min(2, s * 0.004));
    for (let i = 0; i < res.pts.length; i += 2) ctx.fillRect(X(res.pts[i]), Y(res.pts[i + 1]), px, px);
    if (res.sMin <= 0 && res.sMax >= 0) {                      // Bezugspunkt (s = 0)
      ctx.strokeStyle = '#c0392b'; ctx.setLineDash([4, 4]);
      ctx.beginPath(); ctx.moveTo(X(0), m - 6); ctx.lineTo(X(0), H - m + 6); ctx.stroke(); ctx.setLineDash([]);
    }
  }

  function toDXF(res) {
    const out = ['0', 'SECTION', '2', 'HEADER', '9', '$INSUNITS', '70', '6', '0', 'ENDSEC',
                 '0', 'SECTION', '2', 'TABLES', '0', 'TABLE', '2', 'LAYER', '70', '1',
                 '0', 'LAYER', '2', 'SCHNITT', '70', '0', '62', '7', '6', 'CONTINUOUS',
                 '0', 'ENDTAB', '0', 'ENDSEC', '0', 'SECTION', '2', 'ENTITIES'];
    const t = thin(res, 0.01);
    for (let i = 0; i < t.length; i += 2)
      out.push('0', 'POINT', '8', 'SCHNITT', '10', t[i].toFixed(3), '20', t[i + 1].toFixed(3), '30', '0.0');
    out.push('0', 'TEXT', '8', 'SCHNITT', '10', res.sMin.toFixed(3), '20', (res.zMin - 0.25).toFixed(3), '30', '0.0', '40', '0.10',
             '1', `Schnitt durch (${res.x0.toFixed(2)} / ${res.y0.toFixed(2)}) m, Richtung ${res.dirDeg.toFixed(1)} Grad, Dicke ${(res.thick * 100).toFixed(0)} cm`);
    out.push('0', 'ENDSEC', '0', 'EOF');
    return out.join('\r\n') + '\r\n';
  }

  /* SVG im Maßstab 1:10 (1 Einheit = 1 mm) */
  function toSVG(res) {
    const m = 0.2, W = (res.sMax - res.sMin + 2 * m) * 1000, H = (res.zMax - res.zMin + 2 * m) * 1000;
    const t = thin(res, 0.01), r = [];
    for (let i = 0; i < t.length; i += 2)
      r.push(`<rect x="${((t[i] - res.sMin + m) * 1000).toFixed(0)}" y="${((res.zMax + m - t[i + 1]) * 1000).toFixed(0)}" width="10" height="10"/>`);
    return `<svg xmlns="http://www.w3.org/2000/svg" width="${(W / 10).toFixed(0)}mm" height="${(H / 10).toFixed(0)}mm" ` +
      `viewBox="0 0 ${W.toFixed(0)} ${H.toFixed(0)}"><title>PiLiDAR Schnitt (Maßstab 1:10)</title><g fill="#000">${r.join('')}</g></svg>`;
  }

  return { compute, draw, toDXF, toSVG, thin };
})();

window.Section = Section;
