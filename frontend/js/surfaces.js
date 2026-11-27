/* Flächenerkennung: Boden, Decke, Wände, Schrägen, waagerechte Möbelflächen.
 *
 * Die Wolke wird in 10-cm-Würfel geteilt; je Würfel liefert die Hauptachsenanalyse der Punkte
 * (Kovarianz, kleinster Eigenvektor) die Flächennormale und ob die Punkte überhaupt eben liegen.
 * Klassen: 0 Objekt/uneben, 1 Boden, 2 Decke, 3 Wand, 4 Schräge, 5 waagerechte Fläche (Tisch o. ä.).
 * Fläche je Würfel ≈ Zellfläche / größter Normalenanteil (Ebene durch einen Würfel).
 * Es zählen nur vom Scanner gesehene Flächen – verdeckte Bereiche fehlen.
 */
'use strict';

const Surfaces = (() => {
  const NAMES = ['Objekte', 'Boden', 'Decke', 'Wände', 'Schrägen', 'waagerechte Flächen'];

  /* kleinster Eigenwert/-vektor einer symmetrischen 3x3-Matrix (Jacobi-Verfahren) */
  function smallestEig(a) {          // a = [xx, xy, xz, yy, yz, zz]
    let A = [[a[0], a[1], a[2]], [a[1], a[3], a[4]], [a[2], a[4], a[5]]];
    let V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (let sweep = 0; sweep < 8; sweep++) {
      let off = Math.abs(A[0][1]) + Math.abs(A[0][2]) + Math.abs(A[1][2]);
      if (off < 1e-14) break;
      for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
        if (Math.abs(A[p][q]) < 1e-18) continue;
        const th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
        const c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < 3; k++) {                    // Spalten drehen
          const akp = A[k][p], akq = A[k][q];
          A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {                    // Zeilen drehen
          const apk = A[p][k], aqk = A[q][k];
          A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
        }
      }
    }
    const ev = [A[0][0], A[1][1], A[2][2]];
    let i = 0; if (ev[1] < ev[i]) i = 1; if (ev[2] < ev[i]) i = 2;
    return { val: Math.max(ev[i], 0), sum: Math.max(ev[0] + ev[1] + ev[2], 1e-18), n: [V[0][i], V[1][i], V[2][i]] };
  }

  function classify(viewer, cell = 0.10) {
    viewer._flush();
    const chunks = viewer.chunks;
    if (!chunks.length) throw new Error('Keine Punktwolke geladen');
    const K = 8192, H = K / 2;
    const key = (x, y, z) => ((Math.floor(z / cell) + H) * K + Math.floor(y / cell) + H) * K + Math.floor(x / cell) + H;
    const acc = new Map();
    for (const c of chunks) {
      const p = c.pos;
      for (let i = 0; i < p.length; i += 3) {
        const x = p[i], y = p[i + 1], z = p[i + 2], k = key(x, y, z);
        let a = acc.get(k);
        if (!a) { a = new Float64Array(10); acc.set(k, a); }
        a[0]++; a[1] += x; a[2] += y; a[3] += z;
        a[4] += x * x; a[5] += x * y; a[6] += x * z; a[7] += y * y; a[8] += y * z; a[9] += z * z;
      }
    }
    // Normale und Ebenheit je Würfel
    const info = new Map(), horizZ = [];
    for (const [k, a] of acc) {
      const n = a[0];
      if (n < 8) { info.set(k, { cls: 0 }); continue; }
      const mx = a[1] / n, my = a[2] / n, mz = a[3] / n;
      const e = smallestEig([a[4] / n - mx * mx, a[5] / n - mx * my, a[6] / n - mx * mz,
                             a[7] / n - my * my, a[8] / n - my * mz, a[9] / n - mz * mz]);
      const planar = e.val / e.sum < 0.10;   // Messrauschen ~1 cm in 10-cm-Würfeln ergibt schon ~0,06
      const nz = Math.abs(e.n[2]);
      const area = cell * cell / Math.max(Math.abs(e.n[0]), Math.abs(e.n[1]), nz, 0.5);
      let cls = 0;
      if (planar) { if (nz > 0.92) { cls = 5; horizZ.push(mz); } else if (nz < 0.25) cls = 3; else cls = 4; }
      info.set(k, { cls, mz, area });
    }
    // Boden = tiefste deutliche waagerechte Höhe, Decke = höchste
    let floorZ = null, ceilZ = null;
    if (horizZ.length > 20) {
      horizZ.sort((u, v) => u - v);
      const lo = horizZ[Math.floor(horizZ.length * 0.02)], hi = horizZ[Math.floor(horizZ.length * 0.98)];
      const near = z0 => horizZ.filter(z => Math.abs(z - z0) < 0.12).length;
      if (near(lo) > horizZ.length * 0.03) floorZ = lo;
      if (near(hi) > horizZ.length * 0.03 && hi - lo > 1.0) ceilZ = hi;
    }
    // Schräge nur, wenn genug Nachbarwürfel ebenfalls schräg sind – einzelne Schräg-Würfel
    // entstehen an Kanten (Decke/Wand), wo zwei Flächen in einem Würfel liegen.
    const lone = [];
    for (const [k, v] of info) {
      if (v.cls !== 4) continue;
      let nb = 0;
      for (let dz = -1; dz <= 1; dz++) for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        if (!dx && !dy && !dz) continue;
        const o = info.get(k + (dz * K + dy) * K + dx);
        if (o && o.cls === 4) nb++;
      }
      if (nb < 5) lone.push(v);
    }
    for (const v of lone) v.cls = 0;
    const area = [0, 0, 0, 0, 0, 0];
    for (const v of info.values()) {
      if (v.cls === 5) {
        if (floorZ !== null && Math.abs(v.mz - floorZ) < 0.12) v.cls = 1;
        else if (ceilZ !== null && Math.abs(v.mz - ceilZ) < 0.12) v.cls = 2;
      }
      if (v.cls) area[v.cls] += v.area;
    }
    const count = [0, 0, 0, 0, 0, 0];
    for (const c of chunks) {
      c.cls = new Float32Array(c.count);
      const p = c.pos;
      for (let i = 0, j = 0; i < p.length; i += 3, j++) {
        const cl = info.get(key(p[i], p[i + 1], p[i + 2])).cls;
        c.cls[j] = cl; count[cl]++;
      }
    }
    return { area, count, floorZ, ceilZ, cell,
             height: floorZ !== null && ceilZ !== null ? ceilZ - floorZ : null };
  }

  function describe(r) {
    const t = [];
    for (const i of [1, 2, 3, 4, 5]) if (r.area[i] > 0.05) t.push(`${NAMES[i]} ${r.area[i].toFixed(1)} m²`);
    if (r.height) t.push(`Höhe Boden–Decke ${r.height.toFixed(2)} m`);
    return t;
  }

  return { classify, describe, NAMES };
})();

window.Surfaces = Surfaces;
