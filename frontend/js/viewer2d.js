/* 2D-Polarplot der aktuellen Scan-Ebene (Canvas). Leichtgewichtig, für den
 * Lidar-Only-Modus zum Ausrichten/Prüfen. Zeigt die jeweils letzten Frames. */
'use strict';

class Viewer2D {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.maxRange = 6000; // mm, Anzeigeradius (Auto-Skalierung)
    this.points = [];     // {a, d}
    this._raf = null;
    this._resize();
    window.addEventListener('resize', () => this._resize());
  }

  _resize() {
    const r = this.canvas.getBoundingClientRect();
    this.canvas.width = r.width * devicePixelRatio;
    this.canvas.height = r.height * devicePixelRatio;
  }

  clear() { this.points = []; }

  addFrame(angles, dists) {
    for (let i = 0; i < dists.length; i++) {
      if (dists[i] > 0) this.points.push({ a: angles[i], d: dists[i] });
    }
    // STL27L: 0,167° Auflösung → ~2160 Pkt/Umdrehung; 2 Umdrehungen puffern
    if (this.points.length > 4320) this.points.splice(0, this.points.length - 4320);
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

    // Auto-Range
    let mx = 1000;
    for (const p of this.points) if (p.d > mx) mx = p.d;
    this.maxRange = this.maxRange * 0.9 + mx * 0.1;
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

    // Punkte
    // Rechte-Hand-Regel (Blick entlang +Y): X=links, Z=oben.
    // Winkel 0 → +X (links im Plot), Winkel 270 → +Z (oben).
    ctx.fillStyle = '#3fb950';
    for (const p of this.points) {
      const a = p.a * Math.PI / 180;
      const x = cx - Math.cos(a) * p.d * scale;
      const y = cy + Math.sin(a) * p.d * scale;
      ctx.fillRect(x, y, 2 * devicePixelRatio, 2 * devicePixelRatio);
    }
    // Sensor
    ctx.fillStyle = '#f85149';
    ctx.beginPath(); ctx.arc(cx, cy, 4 * devicePixelRatio, 0, 2 * Math.PI); ctx.fill();
  }
}

window.Viewer2D = Viewer2D;
