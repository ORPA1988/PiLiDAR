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

  /* Senkrechte Flächen: Zellen, die in mindestens `need` von `n` Teilstreifen belegt sind
   * (±1 Zelle Toleranz). Dachschrägen, Tischplatten und niedrige Möbel fallen heraus. */
  function verticalCells(chunks, zLo, zHi, cell, n = 4, need = 3, minHits = 2) {
    const h = (zHi - zLo) / n, layers = [];
    for (let k = 0; k < n; k++) {
      const set = new Set();
      for (const p of occupancy(chunks, zLo + k * h, zLo + (k + 1) * h, cell, minHits))
        set.add(Math.floor(p[0] / cell) + ',' + Math.floor(p[1] / cell));
      layers.push(set);
    }
    const all = new Set(); for (const L of layers) for (const k of L) all.add(k);
    const pts = [];
    for (const key of all) {
      const [gx, gy] = key.split(',').map(Number);
      let cnt = 0;
      for (const L of layers) {
        let hit = false;
        for (let dx = -1; dx <= 1 && !hit; dx++) for (let dy = -1; dy <= 1 && !hit; dy++)
          if (L.has((gx + dx) + ',' + (gy + dy))) hit = true;
        if (hit) cnt++;
      }
      if (cnt >= need) pts.push([(gx + 0.5) * cell, (gy + 0.5) * cell]);
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

  /* Fast parallele, nahe Geraden (gleiche Wand, z. B. doppelt erfasst) zusammenführen */
  function mergeLines(lines, { maxAngleDeg = 4, maxDist = 0.08 } = {}) {
    const out = [];
    for (const l of lines.sort((a, b) => b.pts.length - a.pts.length)) {
      const m = out.find(o => {
        const cosA = Math.abs(o.dx * l.dx + o.dy * l.dy);
        const dist = Math.abs(-o.dy * (l.mx - o.mx) + o.dx * (l.my - o.my));
        return cosA > Math.cos(maxAngleDeg * Math.PI / 180) && dist < maxDist;
      });
      if (m) m.pts = m.pts.concat(l.pts); else out.push({ ...l });
    }
    return out;
  }

  /* Rechtwinklig ausrichten: Richtungen auf Hauptrichtung bzw. +90° ziehen (Abweichung < tolDeg) */
  function orthogonalize(lines, tolDeg = 6) {
    if (!lines.length) return lines;
    // Hauptrichtung: gewichteter Mittelwert der Winkel modulo 90° (über doppelte Winkel 4θ)
    let sx = 0, sy = 0;
    for (const l of lines) { const a = Math.atan2(l.dy, l.dx) * 4; sx += Math.cos(a) * l.pts.length; sy += Math.sin(a) * l.pts.length; }
    const base = Math.atan2(sy, sx) / 4;
    for (const l of lines) {
      const a = Math.atan2(l.dy, l.dx);
      const k = Math.round((a - base) / (Math.PI / 2)), snapped = base + k * Math.PI / 2;
      if (Math.abs(a - snapped) < tolDeg * Math.PI / 180) {
        l.dx = Math.cos(snapped); l.dy = Math.sin(snapped);
        // Lage neu: mittlerer Normalabstand der Punkte
        let c = 0; for (const p of l.pts) c += -l.dy * p[0] + l.dx * p[1]; c /= l.pts.length;
        const mc = -l.dy * l.mx + l.dx * l.my;
        l.mx += -l.dy * (c - mc); l.my += l.dx * (c - mc);
      }
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

  /* Bodenhöhe: dichteste Höhe in der unteren Hälfte (nur wenn genug Bodenpunkte sichtbar) */
  function estimateFloor(chunks, bbox) {
    const lo = bbox.min[2], hi = (bbox.min[2] + bbox.max[2]) / 2, bin = 0.02;
    const n = Math.max(1, Math.ceil((hi - lo) / bin) + 1), h = new Uint32Array(n);
    let tot = 0;
    for (const c of chunks) for (let i = 2; i < c.pos.length; i += 3) {
      const z = c.pos[i]; if (z <= hi) { h[Math.min(n - 1, Math.floor((z - lo) / bin))]++; tot++; }
    }
    // tiefste deutliche Fläche (nicht die dichteste – das wäre oft die Tischplatte)
    let max = 0; for (let i = 0; i < n; i++) if (h[i] > max) max = h[i];
    const need = Math.max(1500, max * 0.25);
    for (let i = 0; i < n; i++) if (h[i] >= need) return lo + (i + 0.5) * bin;
    return null;
  }

  /* Raummaße entlang der Hauptrichtungen, Fläche (Flutfüllung vom Scanner aus), Raumhöhe */
  function metrics(res, chunks, bbox) {
    const out = {};
    // Hauptrichtung aus den Wandstücken (gewichtet mit Länge, modulo 90°)
    let sx = 0, sy = 0;
    for (const s of res.segs) {
      const L = Math.hypot(s[2] - s[0], s[3] - s[1]), a = Math.atan2(s[3] - s[1], s[2] - s[0]) * 4;
      sx += Math.cos(a) * L; sy += Math.sin(a) * L;
    }
    const base = Math.atan2(sy, sx) / 4, cb = Math.cos(base), sb = Math.sin(base);
    const us = [], vs = [];
    for (const p of res.occ) { us.push(p[0] * cb + p[1] * sb); vs.push(-p[0] * sb + p[1] * cb); }
    const pct = (arr, q) => { const a = arr.slice().sort((x, y) => x - y); return a[Math.floor(q * (a.length - 1))]; };
    if (us.length > 10) {
      out.lengthA = pct(us, 0.99) - pct(us, 0.01);
      out.lengthB = pct(vs, 0.99) - pct(vs, 0.01);
      out.baseDeg = base * 180 / Math.PI;
    }
    // Fläche: Raster 5 cm, Wände (verbreitert) sperren, vom Scanner aus füllen
    const g = 0.05, b = { x0: Infinity, y0: Infinity, x1: -Infinity, y1: -Infinity };
    for (const p of res.occ) { b.x0 = Math.min(b.x0, p[0]); b.x1 = Math.max(b.x1, p[0]); b.y0 = Math.min(b.y0, p[1]); b.y1 = Math.max(b.y1, p[1]); }
    if (isFinite(b.x0)) {
      for (const st of res.stations) { b.x0 = Math.min(b.x0, st[0]); b.y0 = Math.min(b.y0, st[1]); b.x1 = Math.max(b.x1, st[0]); b.y1 = Math.max(b.y1, st[1]); }
      b.x0 -= g; b.y0 -= g; b.x1 += g; b.y1 += g;
      const W = Math.ceil((b.x1 - b.x0) / g) + 1, H = Math.ceil((b.y1 - b.y0) / g) + 1;
      if (W * H < 4e6) {
        const wall = new Uint8Array(W * H), seen = new Uint8Array(W * H);
        const mark = (x, y) => {                                 // Wandpunkt, 1 Zelle verbreitert
          const ix = Math.floor((x - b.x0) / g), iy = Math.floor((y - b.y0) / g);
          for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
            const xx = ix + dx, yy = iy + dy; if (xx >= 0 && yy >= 0 && xx < W && yy < H) wall[yy * W + xx] = 1;
          }
        };
        for (const p of res.occ) mark(p[0], p[1]);
        for (const s of res.segs) {                              // Wandlinien schließen Lücken im Raster
          const L = Math.hypot(s[2] - s[0], s[3] - s[1]), k = Math.ceil(L / (g / 2));
          for (let i = 0; i <= k; i++) mark(s[0] + (s[2] - s[0]) * i / k, s[1] + (s[3] - s[1]) * i / k);
        }
        const stack = []; let cnt = 0, open = false;         // von jedem Scannerstandort aus füllen
        for (const st of res.stations) {
          const k = Math.floor((st[1] - b.y0) / g) * W + Math.floor((st[0] - b.x0) / g);
          if (!wall[k]) stack.push(k);
        }
        while (stack.length) {
          const i = stack.pop(); if (seen[i] || wall[i]) continue;
          seen[i] = 1; cnt++;
          const x = i % W, y = (i - x) / W;
          if (x === 0 || y === 0 || x === W - 1 || y === H - 1) { open = true; continue; }
          stack.push(i - 1, i + 1, i - W, i + W);
        }
        out.area = cnt * g * g; out.areaOpen = open;
      }
    }
    // Raumhöhe
    const floor = estimateFloor(chunks, bbox);
    out.floor = floor; out.height = floor === null ? null : res.ceiling - floor;
    return out;
  }

  /* Geschlossener Umriss des vom Standort sichtbaren Raums.
   * Strahlen vom Scannerstandort (1°-Schritte) treffen jeweils das nächste Wandstück (>= 0,6 m).
   * Je Wandstück der getroffene Abschnitt (gemessen); Nachbarn werden an der Ecke verlängert,
   * wenn ihr Schnittpunkt nah liegt, sonst gerade verbunden (ergänzt, gestrichelt).
   * Liefert {pts: [[x, y, gemessenBisZumNächsten]], area, perimeter, added} oder null. */
  function outline(res) {
    if (!res.stations || res.stations.length !== 1) return null;
    const st = res.stations[0];
    const segs = res.segs.filter(s => Math.hypot(s[2] - s[0], s[3] - s[1]) >= 0.6);
    if (segs.length < 2) return null;
    const hits = [];
    for (let a = 0; a < 360; a++) {
      const dx = Math.cos(a * Math.PI / 180), dy = Math.sin(a * Math.PI / 180);
      let best = null;
      segs.forEach((s, i) => {
        const ex = s[2] - s[0], ey = s[3] - s[1], den = dx * ey - dy * ex;
        if (Math.abs(den) < 1e-9) return;
        const wx = s[0] - st[0], wy = s[1] - st[1];
        const t = (wx * ey - wy * ex) / den, u = (wx * dy - wy * dx) / den;
        if (t > 0.05 && u >= 0 && u <= 1 && (!best || t < best.t)) best = { t, seg: i, x: st[0] + t * dx, y: st[1] + t * dy };
      });
      hits.push(best);
    }
    if (hits.filter(Boolean).length < 240) return null;        // zu wenig Wände ringsum -> kein verlässlicher Umriss
    const runs = [];
    for (const h of hits) {
      if (!h) continue;
      const last = runs[runs.length - 1];
      if (last && last.seg === h.seg) last.last = h; else runs.push({ seg: h.seg, first: h, last: h });
    }
    if (runs.length > 1 && runs[0].seg === runs[runs.length - 1].seg) { runs[0].first = runs[runs.length - 1].first; runs.pop(); }
    if (runs.length < 2) return null;
    const pts = [];
    runs.forEach((r, k) => {
      const nx = runs[(k + 1) % runs.length], a = segs[r.seg], b = segs[nx.seg];
      pts.push([r.first.x, r.first.y, true]);
      // Ecke: Schnittpunkt der beiden Wandgeraden
      const ax = a[2] - a[0], ay = a[3] - a[1], bx = b[2] - b[0], by = b[3] - b[1], den = ax * by - ay * bx;
      let corner = null;
      if (Math.abs(den) > 0.2 * Math.hypot(ax, ay) * Math.hypot(bx, by)) {
        const t = ((b[0] - a[0]) * by - (b[1] - a[1]) * bx) / den;
        corner = [a[0] + t * ax, a[1] + t * ay];
        if (Math.hypot(corner[0] - r.last.x, corner[1] - r.last.y) > 0.8 ||
            Math.hypot(corner[0] - nx.first.x, corner[1] - nx.first.y) > 0.8) corner = null;
      }
      if (corner) { pts.push([r.last.x, r.last.y, true]); pts.push([corner[0], corner[1], true]); }
      else pts.push([r.last.x, r.last.y, false]);              // Lücke: gerade ergänzt
    });
    let area = 0, per = 0, added = 0;
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i], q = pts[(i + 1) % pts.length], L = Math.hypot(q[0] - p[0], q[1] - p[1]);
      area += p[0] * q[1] - q[0] * p[1]; per += L; if (!p[2]) added += L;
    }
    let inside = false;                                        // Standort muss im Umriss liegen
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      const p = pts[i], q = pts[j];
      if ((p[1] > st[1]) !== (q[1] > st[1]) && st[0] < (q[0] - p[0]) * (st[1] - p[1]) / (q[1] - p[1]) + p[0]) inside = !inside;
    }
    if (!inside || added > 0.5 * per) return null;
    return { pts, area: Math.abs(area) / 2, perimeter: per, added };
  }

  /* Gesamtablauf. opts: {zLo, zHi} (absolut, m) oder automatisch unter der Decke */
  function compute(viewer, opts = {}) {
    const chunks = viewer.chunks, bbox = viewer.bbox;
    if (!chunks.length || !bbox) throw new Error('Keine Punktwolke geladen');
    const ceiling = estimateCeiling(chunks, bbox);
    // Standard: von knapp über Scannerhöhe bis knapp unter die Decke, nur senkrechte Flächen
    const zHi = opts.zHi ?? ceiling - 0.15;
    const zLo = opts.zLo ?? Math.min(0.10, zHi - 0.4);
    const cell = opts.cell ?? 0.02;
    const occ = verticalCells(chunks, zLo, zHi, cell);
    const tol = opts.wallTol ?? 0.05;       // Wände sind durch Messrauschen/Doppelerfassung einige cm dick
    let lines = mergeLines(ransacLines(occ, { tol }), { maxDist: 0.10 });
    if (opts.orthogonal !== false) lines = orthogonalize(lines);
    // Punkte je Gerade neu sammeln (breiteres Band): sonst zerfällt eine leicht krumme oder
    // doppelt erfasste Wand in viele kurze Stücke, die als „zu kurz“ wegfallen
    for (const l of lines) {
      const band = occ.filter(q => Math.abs(-l.dy * (q[0] - l.mx) + l.dx * (q[1] - l.my)) < tol * 1.6);
      if (band.length >= l.pts.length) l.pts = band;
    }
    const segs = segments(lines, { gap: 0.35, minLen: 0.4 });
    let length = 0; for (const s of segs) length += Math.hypot(s[2] - s[0], s[3] - s[1]);
    const stations = opts.stations && opts.stations.length ? opts.stations : [[0, 0]];
    const res = { segs, occ, zLo, zHi, ceiling, cell, wallLength: length, stations };
    res.m = metrics(res, chunks, bbox);
    res.outline = outline(res);
    return res;
  }

  /* Maße als Text (für Anzeige und DXF) */
  function describe(res) {
    const m = res.m || {}, t = [];
    if (m.lengthA) t.push(`Raum ca. ${Math.max(m.lengthA, m.lengthB).toFixed(2)} × ${Math.min(m.lengthA, m.lengthB).toFixed(2)} m`);
    if (m.area && !(m.areaOpen && res.outline)) t.push(`Fläche ${m.areaOpen ? '≥ ' : 'ca. '}${m.area.toFixed(1)} m²${m.areaOpen ? ' (Raum nicht geschlossen)' : ''}`);
    if (res.outline) t.push(`Umriss (vom Standort sichtbar): ${res.outline.area.toFixed(1)} m², Umfang ${res.outline.perimeter.toFixed(1)} m, ` +
      `davon ${res.outline.added.toFixed(1)} m ergänzt`);
    t.push(m.height ? `Raumhöhe (tiefste Fläche bis Decke) ca. ${m.height.toFixed(2)} m` : 'Raumhöhe: Boden nicht sichtbar');
    return t;
  }

  /* DXF (R12, ASCII): Schicht WAENDE mit LINE-Elementen, Einheit Meter; optional Rasterpunkte */
  function toDXF(res, withPoints = false) {
    const out = ['0', 'SECTION', '2', 'HEADER', '9', '$INSUNITS', '70', '6', '0', 'ENDSEC',
                 '0', 'SECTION', '2', 'TABLES', '0', 'TABLE', '2', 'LAYER', '70', '2',
                 '0', 'LAYER', '2', 'WAENDE', '70', '0', '62', '7', '6', 'CONTINUOUS',
                 '0', 'LAYER', '2', 'SCANPUNKTE', '70', '0', '62', '8', '6', 'CONTINUOUS',
                 '0', 'LAYER', '2', 'MASSE', '70', '0', '62', '1', '6', 'CONTINUOUS',
                 '0', 'LAYER', '2', 'UMRISS', '70', '0', '62', '5', '6', 'CONTINUOUS',
                 '0', 'ENDTAB', '0', 'ENDSEC', '0', 'SECTION', '2', 'ENTITIES'];
    for (const s of res.segs)
      out.push('0', 'LINE', '8', 'WAENDE', '10', s[0].toFixed(4), '20', s[1].toFixed(4), '30', '0.0',
               '11', s[2].toFixed(4), '21', s[3].toFixed(4), '31', '0.0');
    if (withPoints) for (const p of res.occ)
      out.push('0', 'POINT', '8', 'SCANPUNKTE', '10', p[0].toFixed(3), '20', p[1].toFixed(3), '30', '0.0');
    if (res.outline) {                                        // geschlossener Umriss als Polylinie
      out.push('0', 'POLYLINE', '8', 'UMRISS', '66', '1', '70', '1');
      for (const p of res.outline.pts) out.push('0', 'VERTEX', '8', 'UMRISS', '10', p[0].toFixed(4), '20', p[1].toFixed(4), '30', '0.0');
      out.push('0', 'SEQEND');
    }
    const bb = bounds(res);                                   // Beschriftung unter dem Plan
    describe(res).forEach((line, k) => out.push('0', 'TEXT', '8', 'MASSE', '10', bb.x0.toFixed(3),
      '20', (bb.y0 - 0.25 - k * 0.15).toFixed(3), '30', '0.0', '40', '0.10', '1', line.replace(/[²×≥]/g, c => ({ '²': '2', '×': 'x', '≥': '>=' })[c])));
    for (const st of res.stations)                            // Scannerstandorte
      out.push('0', 'POINT', '8', 'MASSE', '10', st[0].toFixed(3), '20', st[1].toFixed(3), '30', '0.0');
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
    const ol = res.outline ? `<polygon fill="none" stroke="#1565c0" stroke-width="8" stroke-dasharray="40 30" points="${
      res.outline.pts.map(p => X(p[0]) + ',' + Y(p[1])).join(' ')}"/>` : '';
    return `<svg xmlns="http://www.w3.org/2000/svg" width="${(W / 10).toFixed(0)}mm" height="${(H / 10).toFixed(0)}mm" ` +
      `viewBox="0 0 ${W.toFixed(0)} ${H.toFixed(0)}"><title>PiLiDAR Grundriss (Maßstab 1:10)</title>` +
      `<g fill="#bbb">${pts}</g>${ol}<g stroke="#000" stroke-width="15" stroke-linecap="round">${ln}</g></svg>`;
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
    if (res.outline && res.showOutline !== false) {           // Umriss: gemessen durchgezogen, ergänzt gestrichelt
      const o = res.outline.pts; ctx.strokeStyle = '#1565c0'; ctx.lineWidth = 1.5;
      for (let i = 0; i < o.length; i++) {
        const p = o[i], q = o[(i + 1) % o.length];
        ctx.setLineDash(p[2] ? [] : [6, 5]);
        ctx.beginPath(); ctx.moveTo(X(p[0]), Y(p[1])); ctx.lineTo(X(q[0]), Y(q[1])); ctx.stroke();
      }
      ctx.setLineDash([]);
    }
    ctx.strokeStyle = '#000'; ctx.lineWidth = 2.5; ctx.lineCap = 'round';
    for (const g of res.segs) { ctx.beginPath(); ctx.moveTo(X(g[0]), Y(g[1])); ctx.lineTo(X(g[2]), Y(g[3])); ctx.stroke(); }
    ctx.fillStyle = '#c0392b';                                // Scannerstandorte
    for (const st of res.stations) { ctx.beginPath(); ctx.arc(X(st[0]), Y(st[1]), 5, 0, 7); ctx.fill(); }
    ctx.strokeStyle = '#333'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(20, H - 8); ctx.lineTo(20 + s, H - 8); ctx.stroke();
    ctx.fillStyle = '#333'; ctx.font = '12px system-ui'; ctx.fillText('1 m', 24 + s, H - 4);
  }

  function download(name, text, type) {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([text], { type }));
    a.download = name; a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 5000);
  }

  return { compute, toDXF, toSVG, draw, download, estimateCeiling, describe };
})();

window.FloorPlan = FloorPlan;
