/* Ausrichtung mehrerer Scans zueinander (Registrierung), läuft im Browser.
 *
 * Annahme: der Scanner steht waagerecht (Stativ) -> gesucht sind nur
 * Drehung um die senkrechte Achse (yaw) und Verschiebung tx, ty, tz.
 *   p' = Rz(yaw) · p + t
 *
 * 1. Grob (2D): Wandzellen (senkrecht belegte 10-cm-Zellen) beider Scans als Bild,
 *    für alle Drehwinkel (2°-Schritte) die Verschiebung per FFT-Kreuzkorrelation.
 * 2. Fein (3D): ICP Punkt-zu-Punkt mit sinkendem Fangradius (0,5 -> 0,2 -> 0,08 m)
 *    auf 5-cm-Voxeln, für die besten Grob-Kandidaten; der mit der größten
 *    Überdeckung gewinnt.
 * Nutzbar im Worker (importScripts) und im Hauptfenster (Voxel-Stichprobe).
 */
'use strict';

const Registration = (() => {

  /* --- Voxel-Stichprobe: je Würfel ein Punkt ---------------------------------
   * chunks: [{pos: Float32Array(3n), src: Float32Array(n)}], keep(src) -> bool */
  function voxelSample(chunks, keep, cell = 0.05, maxPts = Infinity) {
    const seen = new Set(), out = [];
    const K = 8192, H = K / 2;
    for (const c of chunks) {
      const p = c.pos, s = c.src;
      for (let i = 0, j = 0; i < p.length; i += 3, j++) {
        if (s && !keep(s[j])) continue;
        const ix = Math.floor(p[i] / cell) + H, iy = Math.floor(p[i + 1] / cell) + H, iz = Math.floor(p[i + 2] / cell) + H;
        const key = (iz * K + iy) * K + ix;
        if (seen.has(key)) continue;
        seen.add(key); out.push(p[i], p[i + 1], p[i + 2]);
      }
    }
    let xyz = Float32Array.from(out);
    if (xyz.length / 3 > maxPts) xyz = subsample(xyz, maxPts);
    return xyz;
  }

  function subsample(xyz, m, seed = 1) {
    const n = xyz.length / 3, out = new Float32Array(m * 3);
    let r = seed >>> 0;
    const rnd = () => ((r = (r * 1664525 + 1013904223) >>> 0) / 4294967296);
    for (let k = 0; k < m; k++) {             // Ziehen mit Zurücklegen genügt hier
      const i = Math.floor(rnd() * n) * 3;
      out[k * 3] = xyz[i]; out[k * 3 + 1] = xyz[i + 1]; out[k * 3 + 2] = xyz[i + 2];
    }
    return out;
  }

  /* --- Wandzellen: 2D-Zellen, in denen viele Höhenstufen belegt sind ---------- */
  function wallCells(xyz, cell = 0.1) {
    let z0 = Infinity, z1 = -Infinity;
    for (let i = 2; i < xyz.length; i += 3) { if (xyz[i] < z0) z0 = xyz[i]; if (xyz[i] > z1) z1 = xyz[i]; }
    const zb = Math.max(0.1, (z1 - z0) / 30);                 // höchstens 31 Höhenstufen (Bitmaske)
    const m = new Map();
    for (let i = 0; i < xyz.length; i += 3) {
      const key = (Math.floor(xyz[i] / cell) + 50000) * 100000 + Math.floor(xyz[i + 1] / cell) + 50000;
      const bit = 1 << Math.min(30, Math.floor((xyz[i + 2] - z0) / zb));
      m.set(key, (m.get(key) || 0) | bit);
    }
    const pts = [];
    for (const [key, mask] of m) {
      let b = mask, cnt = 0; while (b) { b &= b - 1; cnt++; }
      if (cnt < 5) continue;                                   // Boden/Decke/Schräge: nur 1-2 Stufen
      const ix = Math.floor(key / 100000) - 50000, iy = (key % 100000) - 50000;
      pts.push((ix + 0.5) * cell, (iy + 0.5) * cell);
    }
    return Float64Array.from(pts);
  }

  /* --- FFT (radix 2, komplex, in place) ---------------------------------------- */
  function fft1(re, im, off, stride, n, inv) {
    for (let i = 1, j = 0; i < n; i++) {                       // Bit-Umkehr
      let bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        const a = off + i * stride, b = off + j * stride;
        let t = re[a]; re[a] = re[b]; re[b] = t; t = im[a]; im[a] = im[b]; im[b] = t;
      }
    }
    for (let len = 2; len <= n; len <<= 1) {
      const ang = (inv ? 2 : -2) * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
      for (let i = 0; i < n; i += len) {
        let cr = 1, ci = 0;
        for (let k = 0; k < len / 2; k++) {
          const a = off + (i + k) * stride, b = off + (i + k + len / 2) * stride;
          const xr = re[b] * cr - im[b] * ci, xi = re[b] * ci + im[b] * cr;
          re[b] = re[a] - xr; im[b] = im[a] - xi; re[a] += xr; im[a] += xi;
          const t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
        }
      }
    }
  }
  function fft2(re, im, N, inv) {
    for (let r = 0; r < N; r++) fft1(re, im, r * N, 1, N, inv);
    for (let c = 0; c < N; c++) fft1(re, im, c, N, N, inv);
  }

  function rasterize(pts, x0, y0, c, N, blur) {
    const img = new Float64Array(N * N);
    for (let i = 0; i < pts.length; i += 2) {
      const ix = Math.floor((pts[i] - x0) / c), iy = Math.floor((pts[i + 1] - y0) / c);
      if (ix < 0 || iy < 0 || ix >= N || iy >= N) continue;
      if (!blur) { img[iy * N + ix] = 1; continue; }
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const x = ix + dx, y = iy + dy;
        if (x < 0 || y < 0 || x >= N || y >= N) continue;
        const w = (dx ? 0.5 : 1) * (dy ? 0.5 : 1);
        if (w > img[y * N + x]) img[y * N + x] = w;
      }
    }
    return img;
  }

  /* Grobsuche: liefert Kandidaten [{yaw, tx, ty, score}] (yaw in rad) */
  function coarse(refW, movW, opts = {}, progress) {
    if (refW.length < 20 || movW.length < 20) return [];
    let rx0 = Infinity, ry0 = Infinity, rx1 = -Infinity, ry1 = -Infinity;
    for (let i = 0; i < refW.length; i += 2) {
      rx0 = Math.min(rx0, refW[i]); rx1 = Math.max(rx1, refW[i]); ry0 = Math.min(ry0, refW[i + 1]); ry1 = Math.max(ry1, refW[i + 1]);
    }
    let mcx = 0, mcy = 0; const nm = movW.length / 2;
    for (let i = 0; i < movW.length; i += 2) { mcx += movW[i]; mcy += movW[i + 1]; }
    mcx /= nm; mcy /= nm;
    let rm = 0; for (let i = 0; i < movW.length; i += 2) rm = Math.max(rm, Math.hypot(movW[i] - mcx, movW[i + 1] - mcy));
    const ext = Math.max(rx1 - rx0, ry1 - ry0) + 2 * rm + 0.4;
    let c = opts.cell || 0.1, N = 64;
    while (N * c < ext && N < 256) N *= 2;
    if (N * c < ext) c = ext / N;
    const FR = { re: rasterize(refW, rx0 - 0.2, ry0 - 0.2, c, N, true), im: new Float64Array(N * N) };
    fft2(FR.re, FR.im, N, false);
    const stepDeg = opts.stepDeg || 2, cands = [];
    const re = new Float64Array(N * N), im = new Float64Array(N * N), q = new Float64Array(movW.length);
    for (let deg = 0; deg < 360; deg += stepDeg) {
      const a = deg * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a);
      let qx0 = Infinity, qy0 = Infinity;
      for (let i = 0; i < movW.length; i += 2) {
        const x = movW[i] - mcx, y = movW[i + 1] - mcy;
        q[i] = ca * x - sa * y; q[i + 1] = sa * x + ca * y;
        qx0 = Math.min(qx0, q[i]); qy0 = Math.min(qy0, q[i + 1]);
      }
      re.set(rasterize(q, qx0, qy0, c, N, false)); im.fill(0);
      fft2(re, im, N, false);
      for (let k = 0; k < N * N; k++) {                       // R · conj(M)
        const ar = FR.re[k], ai = FR.im[k], br = re[k], bi = im[k];
        re[k] = ar * br + ai * bi; im[k] = ai * br - ar * bi;
      }
      fft2(re, im, N, true);
      let best = -1, bk = 0;
      for (let k = 0; k < N * N; k++) if (re[k] > best) { best = re[k]; bk = k; }
      let kx = bk % N, ky = (bk - kx) / N;
      if (kx > N / 2) kx -= N; if (ky > N / 2) ky -= N;
      // p -> Rot(p - mc) - q0 + r0 + k·c
      const tx = -(ca * mcx - sa * mcy) - qx0 + (rx0 - 0.2) + kx * c;
      const ty = -(sa * mcx + ca * mcy) - qy0 + (ry0 - 0.2) + ky * c;
      cands.push({ yaw: a, tx, ty, score: best / (N * N) / nm });
      if (progress) progress(deg / 360);
    }
    cands.sort((u, v) => v.score - u.score);
    const out = [];                                            // verschiedene Kandidaten behalten
    for (const cd of cands) {
      if (out.every(o => Math.abs(angDiff(o.yaw, cd.yaw)) > 8 * Math.PI / 180 || Math.hypot(o.tx - cd.tx, o.ty - cd.ty) > 0.5))
        out.push(cd);
      if (out.length >= (opts.candidates || 6)) break;
    }
    return out;
  }

  function angDiff(a, b) { let d = a - b; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return d; }

  /* --- Nachbarsuche über ein Hash-Raster -------------------------------------- */
  function buildGrid(xyz, h) {
    const n = xyz.length / 3, keys = new Float64Array(n), idx = new Uint32Array(n);
    const K = 8192, H = K / 2;
    for (let i = 0; i < n; i++) {
      keys[i] = ((Math.floor(xyz[i * 3 + 2] / h) + H) * K + Math.floor(xyz[i * 3 + 1] / h) + H) * K + Math.floor(xyz[i * 3] / h) + H;
      idx[i] = i;
    }
    idx.sort((a, b) => keys[a] - keys[b]);
    const map = new Map();
    for (let s = 0; s < n;) {
      let e = s + 1; while (e < n && keys[idx[e]] === keys[idx[s]]) e++;
      map.set(keys[idx[s]], [s, e]); s = e;
    }
    return { map, idx, h, K, H, xyz };
  }
  function nearest(g, x, y, z, maxD) {
    const { map, idx, h, K, H, xyz } = g;
    const ix = Math.floor(x / h) + H, iy = Math.floor(y / h) + H, iz = Math.floor(z / h) + H;
    let best = maxD * maxD, bi = -1;
    for (let dz = -1; dz <= 1; dz++) for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      const r = map.get(((iz + dz) * K + iy + dy) * K + ix + dx);
      if (!r) continue;
      for (let k = r[0]; k < r[1]; k++) {
        const j = idx[k] * 3, ex = xyz[j] - x, ey = xyz[j + 1] - y, ez = xyz[j + 2] - z;
        const d = ex * ex + ey * ey + ez * ez;
        if (d < best) { best = d; bi = idx[k]; }
      }
    }
    return bi < 0 ? null : [bi, best];
  }

  /* --- ICP (yaw, tx, ty, tz) ---------------------------------------------------- */
  function icp(refXYZ, movXYZ, init, grids, stages = [0.5, 0.2, 0.08]) {
    let { yaw, tx, ty, tz = 0 } = init;
    const n = movXYZ.length / 3;
    for (const stage of stages) {
      const g = grids[stage] || (grids[stage] = buildGrid(refXYZ, stage));
      for (let it = 0; it < 20; it++) {
        const c = Math.cos(yaw), s = Math.sin(yaw);
        let cnt = 0, px = 0, py = 0, pz = 0, qx = 0, qy = 0, qz = 0;
        const P = [], Q = [];
        for (let i = 0; i < n; i++) {
          const x = movXYZ[i * 3], y = movXYZ[i * 3 + 1], z = movXYZ[i * 3 + 2];
          const X = c * x - s * y + tx, Y = s * x + c * y + ty, Z = z + tz;
          const nn = nearest(g, X, Y, Z, stage);
          if (!nn) continue;
          const j = nn[0] * 3;
          P.push(X, Y, Z); Q.push(refXYZ[j], refXYZ[j + 1], refXYZ[j + 2]);
          px += X; py += Y; pz += Z; qx += refXYZ[j]; qy += refXYZ[j + 1]; qz += refXYZ[j + 2]; cnt++;
        }
        if (cnt < 50) break;
        px /= cnt; py /= cnt; pz /= cnt; qx /= cnt; qy /= cnt; qz /= cnt;
        let sxx = 0, sxy = 0;
        for (let k = 0; k < P.length; k += 3) {
          const ax = P[k] - px, ay = P[k + 1] - py, bx = Q[k] - qx, by = Q[k + 1] - qy;
          sxx += ax * bx + ay * by; sxy += ax * by - ay * bx;
        }
        const d = Math.atan2(sxy, sxx), cd = Math.cos(d), sd = Math.sin(d);
        // neue Lage = Rot(d)·(alte Lage um Schwerpunkt p) + q
        const ntx = cd * (tx - px) - sd * (ty - py) + qx, nty = sd * (tx - px) + cd * (ty - py) + qy;
        const dz = qz - pz;
        const change = Math.abs(d) + Math.hypot(ntx - tx, nty - ty) + Math.abs(dz);
        yaw += d; tx = ntx; ty = nty; tz += dz;
        if (change < 1e-4) break;
      }
    }
    return { yaw: angDiff(yaw, 0), tx, ty, tz, ...quality(refXYZ, movXYZ, { yaw, tx, ty, tz }, grids) };
  }

  /* Überdeckung (Anteil Punkte mit Nachbar < 10 cm) und RMSE dieser Punkte */
  function quality(refXYZ, movXYZ, pose, grids) {
    const g = grids[0.1] || (grids[0.1] = buildGrid(refXYZ, 0.1));
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw), n = movXYZ.length / 3;
    let inl = 0, se = 0;
    for (let i = 0; i < n; i++) {
      const x = movXYZ[i * 3], y = movXYZ[i * 3 + 1], z = movXYZ[i * 3 + 2];
      const nn = nearest(g, c * x - s * y + pose.tx, s * x + c * y + pose.ty, z + pose.tz, 0.1);
      if (nn) { inl++; se += nn[1]; }
    }
    return { overlap: n ? inl / n : 0, rmse: inl ? Math.sqrt(se / inl) : null, inliers: inl };
  }

  /* Gesamtablauf: ref/mov = Float32Array xyz (bereits ausgedünnt, 5 cm) */
  function align(refXYZ, movXYZ, opts = {}, progress = () => {}) {
    const t0 = Date.now();
    const refW = wallCells(refXYZ), movW = wallCells(movXYZ);
    const movS = movXYZ.length / 3 > 20000 ? subsample(movXYZ, 20000, 7) : movXYZ;
    let cands = opts.init ? [opts.init] : coarse(refW, movW, opts, f => progress(0.6 * f, 'Grobsuche'));
    if (!opts.init) cands.push({ yaw: 0, tx: 0, ty: 0 });     // „nicht bewegt“ immer mitprüfen
    if (!cands.length) throw new Error('Zu wenige Wände für die Ausrichtung');
    // Vorlauf: alle Kandidaten grob mit wenigen Punkten, dann nur die besten zwei fein
    const grids = {}, few = subsample(movXYZ, Math.min(2500, movXYZ.length / 3), 3);
    let pre = cands.map((cd, k) => {
      const r = icp(refXYZ, few, cd, grids, [0.5, 0.2]);
      progress(0.6 + 0.2 * (k + 1) / cands.length, 'Vorauswahl');
      return r;
    });
    pre.sort((a, b) => b.overlap - a.overlap);
    const distinct = [];                                       // gleiche Lösungen zusammenfassen
    for (const r of pre)
      if (distinct.every(d => Math.abs(angDiff(d.yaw, r.yaw)) > 3 * Math.PI / 180 || Math.hypot(d.tx - r.tx, d.ty - r.ty) > 0.2))
        distinct.push(r);
    const results = distinct.slice(0, 2).map((cd, k) => {
      const r = icp(refXYZ, movS, cd, grids, [0.2, 0.08]);
      progress(0.8 + 0.2 * (k + 1) / Math.min(2, distinct.length), 'Feinausrichtung');
      return r;
    });
    results.sort((a, b) => (b.overlap - a.overlap) || ((a.rmse || 1) - (b.rmse || 1)));
    const best = results[0];
    return { ...best, yawDeg: best.yaw * 180 / Math.PI, candidates: cands.length, distinct: distinct.length,
             second: results[1] ? results[1].overlap : 0, wallCells: [refW.length / 2, movW.length / 2], ms: Date.now() - t0 };
  }

  return { voxelSample, subsample, wallCells, coarse, icp, align, quality, fft2 };
})();

if (typeof self !== 'undefined') self.Registration = Registration;
