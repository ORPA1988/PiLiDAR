/* 2D-Polarplot der aktuellen Scan-Ebene (Canvas). Leichtgewichtig, für den
 * Lidar-Only-Modus zum Ausrichten/Prüfen.
 *
 * Je 0,2°-Winkelsegment wird der letzte Messwert mit Zeitstempel gehalten und
 * bis zur Nachleuchtzeit (persistMs) angezeigt, zum Ende hin ausgeblendet. So
 * bleibt die volle Kontur sichtbar, auch wenn nicht jedes Paket ankommt. */
'use strict';

const BIN_DEG = 0.2;
const N_BINS = Math.round(360 / BIN_DEG);

class Viewer2D {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.maxRange = 6000;            // mm, Anzeigeradius (Auto-Skalierung)
    this.persistMs = 1000;           // Nachleuchtzeit
    this.dist = new Float32Array(N_BINS);
    this.stamp = new Float64Array(N_BINS);   // performance.now() der letzten Messung
    this.angle = new Float32Array(N_BINS);
    this._raf = null;
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }

  _resize() {
    const r = this.canvas.getBoundingClientRect();
    this.canvas.width = r.width * devicePixelRatio;
    this.canvas.height = r.height * devicePixelRatio;
  }

  setPersistence(ms) { this.persistMs = Math.max(50, +ms || 1000); }

  clear() { this.stamp.fill(0); this.dist.fill(0); }

  addFrame(angles, dists) {
    const now = performance.now();
    for (let i = 0; i < dists.length; i++) {
      const d = dists[i];
      if (!(d > 0)) continue;
      let a = angles[i] % 360; if (a < 0) a += 360;
      const b = Math.min(N_BINS - 1, Math.floor(a / BIN_DEG));
      this.dist[b] = d; this.angle[b] = a; this.stamp[b] = now;
    }
  }

  start() { if (!this._raf) this._loop(); }
  stop() { if (this._raf) { cancelAnimationFrame(this._raf); this._raf = null; } }

  _loop() {
    this._draw();
    this._raf = requestAnimationFrame(() => this._loop());
  }

  _draw() {
    const ctx = this.ctx, W = this.canvas.width, H = this.canvas.height;
    ctx.clearRect(0, 0, W, H);
    const cx = W / 2, cy = H / 2;
    const R = Math.min(W, H) * 0.45;
    const now = performance.now(), persist = this.persistMs;

    // sichtbare Segmente sammeln
    const vis = [];
    for (let b = 0; b < N_BINS; b++) {
      const age = now - this.stamp[b];
      if (this.stamp[b] > 0 && age < persist) vis.push(b);
    }

    // Auto-Range über das 98. Perzentil (einzelne Ausreißer ignorieren)
    let target = 1000;
    if (vis.length) {
      const ds = vis.map(b => this.dist[b]).sort((x, y) => x - y);
      target = Math.max(1000, ds[Math.floor(0.98 * (ds.length - 1))] * 1.1);
    }
    this.maxRange = this.maxRange * 0.9 + target * 0.1;
    const scale = R / this.maxRange;

    // Gitter
    ctx.strokeStyle = '#243043';
    ctx.fillStyle = '#5b6675';
    ctx.font = `${12 * devicePixelRatio}px sans-serif`;
    ctx.lineWidth = devicePixelRatio;
    for (let k = 1; k <= 4; k++) {
      const rr = R * k / 4;
      ctx.beginPath(); ctx.arc(cx, cy, rr, 0, 2 * Math.PI); ctx.stroke();
      ctx.fillText(`${(this.maxRange * k / 4 / 1000).toFixed(1)} m`, cx + 4, cy - rr + 14 * devicePixelRatio);
    }
    ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
    ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();

    // Punkte, in 4 Altersstufen ausgeblendet
    // Rechte-Hand-Regel (Blick entlang +Y): X=links, Z=oben.
    // Winkel 0 → +X (links im Plot), Winkel 270 → +Z (oben).
    ctx.fillStyle = '#3fb950';
    const sz = 2 * devicePixelRatio;
    for (let level = 3; level >= 0; level--) {
      ctx.globalAlpha = 1 - level * 0.22;
      const lo = persist * level / 4, hi = persist * (level + 1) / 4;
      for (const b of vis) {
        const age = now - this.stamp[b];
        if (age < lo || age >= hi) continue;
        const a = this.angle[b] * Math.PI / 180, d = this.dist[b];
        ctx.fillRect(cx - Math.cos(a) * d * scale, cy + Math.sin(a) * d * scale, sz, sz);
      }
    }
    ctx.globalAlpha = 1;
    // Sensor
    ctx.fillStyle = '#f85149';
    ctx.beginPath(); ctx.arc(cx, cy, 4 * devicePixelRatio, 0, 2 * Math.PI); ctx.fill();
  }
}

window.Viewer2D = Viewer2D;
