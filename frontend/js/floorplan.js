/* Automatischer Grundriss aus der Punktwolke (läuft im Browser).
 *
 * 1. Höhenstreifen wählen: automatisch knapp unter der Decke (dort stehen selten Möbel) oder
 *    vorgegeben. Die Decke wird als dichteste Höhe im oberen Bereich gesucht.
 * 2. Punkte des Streifens auf die XY-Ebene projizieren und auf ein Raster (Standard 2 cm) legen.
 * 3. Wände per RANSAC als Geraden finden, Punkte je Gerade in zusammenhängende Abschnitte teilen
 *    (Lücken > 25 cm trennen: Türen, Fenster), kurze Stücke verwerfen.
 * 4. Ausgabe: Liniensegmente in Metern -> Vorschau, DXF (für CAD) und SVG.
 */
'use strict';

const FloorPlan = (() => {

  /* Deckenhöhe schätzen: Histogramm der z-Werte (2 cm), dichteste Klasse in der oberen Hälfte */
  function estimateCeiling(chunks, bbox) {
    if (!bbox) return null;
    const lo = (bbox.min[2] + bbox.max[2]) / 2, hi = bbox.max[2], bin = 0.02;
    const n = Math.max(1, Math.ceil((hi - lo) / bin) + 1), h = new Uint32Array(n);
    for (const c of chunks) for (let i = 2; i < c.pos.length; i += 3) {
      const z = c.pos[i]; if (z >= lo) h[Math.min(n - 1, Math.floor((z - lo) / bin))]++;
    }
    let best = 0; for (let i = 1; i < n; i++) if (h[i] > h[best]) best = i;
    return lo + (best + 0.5) * bin;
  }

  /* Belegte Rasterzellen im Höhenstreifen -> Liste von [x, y] (Zellmitten) */
  function occupancy(chunks, zLo, zHi, cell, minHits) {
    const map = new Map();
    for (const c of chunks) {
      const p = c.pos;
      for (let i = 0; i < p.length; i += 3) {
        const z = p[i + 2]; if (z < zLo || z > zHi) continue;
        const key = Math.floor(p[i] / cell) + ',' + Math.floor(p[i + 1] / cell);
        map.set(key, (map.get(key) || 0) + 1);
      }
    }
    const pts = [];
    for (const [key, cnt] of map) {
      if (cnt < minHits) continue;
      const [gx, gy] = key.split(',').map(Number);
      pts.push([(gx + 0.5) * cell, (gy + 0.5) * cell]);
    }
    return pts;
  }

  /* RANSAC: wiederholt die Gerade mit den meisten Punkten (Abstand < tol) suchen */
  function ransacLines(pts, { tol = 0.03, minPts = 40, maxLines = 40, iters = 600 } = {}) {
    let rest = pts.slice();
    const lines = [];
    let rnd = 12345;                                          // reproduzierbarer Zufall
    const rand = n => { rnd = (rnd * 1103515245 + 12345) & 0x7fffffff; return rnd % n; };
    while (rest.length >= minPts && lines.length < maxLines) {
      // Kandidaten an einer Stichprobe bewerten (schnell), Sieger an allen Punkten prüfen
      const S = Math.min(rest.length, 1500);
      const sample = Array.from({ length: S }, () => rest[rand(rest.length)]);
      let best = null, bestCount = 0;
      for (let k = 0; k < iters; k++) {
        const a = rest[rand(rest.length)], b = rest[rand(rest.length)];
        const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy);
        if (L < 0.2) continue;
        const nx = -dy / L, ny = dx / L, c = nx * a[0] + ny * a[1];
        let cnt = 0;
        for (const p of sample) if (Math.abs(nx * p[0] + ny * p[1] - c) < tol) cnt++;
        if (cnt > bestCount) { bestCount = cnt; best = { nx, ny, c }; }
      }
      if (best) {
        bestCount = 0;
        for (const p of rest) if (Math.abs(best.nx * p[0] + best.ny * p[1] - best.c) < tol) bestCount++;
      }
      if (!best || bestCount < minPts) break;
      // Gerade mit allen Inliern verfeinern (Hauptachse)
      const inl = rest.filter(p => Math.abs(best.nx * p[0] + best.ny * p[1] - best.c) < tol);
      let mx = 0, my = 0; for (const p of inl) { mx += p[0]; my += p[1]; } mx /= inl.length; my /= inl.length;
      let sxx = 0, sxy = 0, syy = 0;
      for (const p of inl) { const u = p[0] - mx, v = p[1] - my; sxx += u * u; sxy += u * v; syy += v * v; }
      const ang = 0.5 * Math.atan2(2 * sxy, sxx - syy), dx = Math.cos(ang), dy = Math.sin(ang);
      lines.push({ mx, my, dx, dy, pts: inl });
      rest = rest.filter(p => Math.abs(-dy * (p[0] - mx) + dx * (p[1] - my)) >= tol * 1.5);
    }
    return lines;
  }

  /* Punkte einer Geraden in Abschnitte teilen (Lücke > gap trennt), kurze verwerfen */
  function segments(lines, { gap = 0.25, minLen = 0.3 } = {}) {
    const segs = [];
    for (const l of lines) {
      const t = l.pts.map(p => (p[0] - l.mx) * l.dx + (p[1] - l.my) * l.dy).sort((a, b) => a - b);
      let s0 = t[0], prev = t[0];
      const push = (a, b) => {
        if (b - a >= minLen) segs.push([l.mx + a * l.dx, l.my + a * l.dy, l.mx + b * l.dx, l.my + b * l.dy]);
      };
      for (let i = 1; i < t.length; i++) {
        if (t[i] - prev > gap) { push(s0, prev); s0 = t[i]; }
        prev = t[i];
      }
      push(s0, prev);
    }
    return segs;
  }

  /* Gesamtablauf. opts: {zLo, zHi} (absolut, m) oder automatisch unter der Decke */
  function compute(viewer, opts = {}) {
    const chunks = viewer.chunks, bbox = viewer.bbox;
    if (!chunks.length || !bbox) throw new Error('Keine Punktwolke geladen');
    const ceiling = estimateCeiling(chunks, bbox);
    const zHi = opts.zHi ?? ceiling - 0.12;
    const zLo = opts.zLo ?? ceiling - 0.60;
    const cell = opts.cell ?? 0.02;
    const occ = occupancy(chunks, zLo, zHi, cell, opts.minHits ?? 3);
    const lines = ransacLines(occ, { tol: cell * 1.5 });
    const segs = segments(lines);
    let length = 0; for (const s of segs) length += Math.hypot(s[2] - s[0], s[3] - s[1]);
    return { segs, occ, zLo, zHi, ceiling, cell, wallLength: length };
  }

  /* DXF (R12, ASCII): Schicht WAENDE mit LINE-Elementen, Einheit Meter; optional Rasterpunkte */
  function toDXF(res, withPoints = false) {
    const out = ['0', 'SECTION', '2', 'HEADER', '9', '$INSUNITS', '70', '6', '0', 'ENDSEC',
                 '0', 'SECTION', '2', 'TABLES', '0', 'TABLE', '2', 'LAYER', '70', '2',
                 '0', 'LAYER', '2', 'WAENDE', '70', '0', '62', '7', '6', 'CONTINUOUS',
                 '0', 'LAYER', '2', 'SCANPUNKTE', '70', '0', '62', '8', '6', 'CONTINUOUS',
                 '0', 'ENDTAB', '0', 'ENDSEC', '0', 'SECTION', '2', 'ENTITIES'];
    for (const s of res.segs)
      out.push('0', 'LINE', '8', 'WAENDE', '10', s[0].toFixed(4), '20', s[1].toFixed(4), '30', '0.0',
               '11', s[2].toFixed(4), '21', s[3].toFixed(4), '31', '0.0');
    if (withPoints) for (const p of res.occ)
      out.push('0', 'POINT', '8', 'SCANPUNKTE', '10', p[0].toFixed(3), '20', p[1].toFixed(3), '30', '0.0');
    out.push('0', 'ENDSEC', '0', 'EOF');
    return out.join('\r\n') + '\r\n';
  }

  function bounds(res) {
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const p of res.occ) { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); }
    return { x0, y0, x1, y1 };
  }

  /* SVG in Millimetern (1 Einheit = 1 mm), y nach oben wie im Plan */
  function toSVG(res) {
    const b = bounds(res), m = 0.2;
    const W = (b.x1 - b.x0 + 2 * m) * 1000, H = (b.y1 - b.y0 + 2 * m) * 1000;
    const X = x => ((x - b.x0 + m) * 1000).toFixed(1), Y = y => ((b.y1 + m - y) * 1000).toFixed(1);
    const pts = res.occ.map(p => `<rect x="${X(p[0])}" y="${Y(p[1])}" width="${res.cell * 1000}" height="${res.cell * 1000}"/>`).join('');
    const ln = res.segs.map(s => `<line x1="${X(s[0])}" y1="${Y(s[1])}" x2="${X(s[2])}" y2="${Y(s[3])}"/>`).join('');
    return `<svg xmlns="http://www.w3.org/2000/svg" width="${(W / 10).toFixed(0)}mm" height="${(H / 10).toFixed(0)}mm" ` +
      `viewBox="0 0 ${W.toFixed(0)} ${H.toFixed(0)}"><title>PiLiDAR Grundriss (Maßstab 1:10)</title>` +
      `<g fill="#bbb">${pts}</g><g stroke="#000" stroke-width="15" stroke-linecap="round">${ln}</g></svg>`;
  }

  /* Vorschau auf ein Canvas zeichnen (Rasterpunkte grau, Wände schwarz, Maßstab) */
  function draw(canvas, res) {
    const ctx = canvas.getContext('2d'), W = canvas.width, H = canvas.height;
    ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, W, H);
    const b = bounds(res);
    const s = Math.min((W - 40) / (b.x1 - b.x0 || 1), (H - 40) / (b.y1 - b.y0 || 1));
    const X = x => 20 + (x - b.x0) * s, Y = y => H - 20 - (y - b.y0) * s;
    ctx.fillStyle = '#c8c8c8';
    for (const p of res.occ) ctx.fillRect(X(p[0]), Y(p[1]), Math.max(1, res.cell * s), Math.max(1, res.cell * s));
    ctx.strokeStyle = '#000'; ctx.lineWidth = 2.5; ctx.lineCap = 'round';
    for (const g of res.segs) { ctx.beginPath(); ctx.moveTo(X(g[0]), Y(g[1])); ctx.lineTo(X(g[2]), Y(g[3])); ctx.stroke(); }
    ctx.fillStyle = '#c0392b'; ctx.beginPath(); ctx.arc(X(0), Y(0), 5, 0, 7); ctx.fill();   // Scannerstandort
    ctx.strokeStyle = '#333'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(20, H - 8); ctx.lineTo(20 + s, H - 8); ctx.stroke();
    ctx.fillStyle = '#333'; ctx.font = '12px system-ui'; ctx.fillText('1 m', 24 + s, H - 4);
  }

  function download(name, text, type) {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([text], { type }));
    a.download = name; a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  }

  return { compute, toDXF, toSVG, draw, download, estimateCeiling };
})();

window.FloorPlan = FloorPlan;
