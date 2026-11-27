/* WebGL-Punktwolken-Viewer ohne externe Bibliothek.
 *
 * Bedienung: linke Maustaste ziehen = drehen, rechte Maustaste oder Umschalt+ziehen = verschieben,
 * Mausrad = zoomen, Doppelklick = auf Punkt zentrieren. Touch: 1 Finger drehen, 2 Finger zoomen.
 * Farbmodi: Intensität, Höhe, Entfernung, einfarbig (Farbskala „turbo“).
 * Höhenschnitt (z. B. Decke ausblenden), Messwerkzeug (Abstand zweier Punkte), feste Ansichten.
 * Punkte liegen in GPU-Puffern (Blöcke) und als CPU-Kopie für Messung/Statistik.
 */
'use strict';

const VERT = `
attribute vec3 aPos; attribute float aInt;
uniform mat4 uMVP; uniform float uSize;
uniform mediump int uMode; uniform vec2 uRange; uniform vec2 uClip;
varying float vVal; varying float vCut;
void main(){
  gl_Position = uMVP * vec4(aPos, 1.0);
  gl_PointSize = uSize;
  float v = aInt;                                   // 0: Intensität
  if (uMode == 1) v = aPos.z;                       // 1: Höhe
  else if (uMode == 2) v = length(aPos);            // 2: Entfernung zum Scanner
  vVal = (v - uRange.x) / max(uRange.y - uRange.x, 1e-6);
  vCut = (aPos.z < uClip.x || aPos.z > uClip.y) ? 1.0 : 0.0;
}`;

const FRAG = `
precision mediump float;
uniform mediump int uMode;              // gleiche Genauigkeit wie im Vertex-Shader (sonst Link-Fehler)
varying float vVal; varying float vCut;
vec3 turbo(float x){                                // Näherung der Farbskala „turbo“
  x = clamp(x, 0.0, 1.0);
  vec4 kr = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  vec4 kg = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  vec4 kb = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  vec2 kr2 = vec2(-152.94239396, 59.28637943);
  vec2 kg2 = vec2(4.27729857, 2.82956604);
  vec2 kb2 = vec2(-89.90310912, 27.34824973);
  vec4 v4 = vec4(1.0, x, x * x, x * x * x);
  vec2 v2 = v4.zw * v4.z;
  return vec3(dot(v4, kr) + dot(v2, kr2), dot(v4, kg) + dot(v2, kg2), dot(v4, kb) + dot(v2, kb2));
}
void main(){
  if (vCut > 0.5) discard;
  if (uMode == 3) { gl_FragColor = vec4(0.55, 0.85, 0.65, 1.0); return; }
  if (uMode == 0) { float g = clamp(vVal, 0.05, 1.0); gl_FragColor = vec4(g * 0.5, g, g * 0.7, 1.0); return; }
  gl_FragColor = vec4(turbo(vVal), 1.0);
}`;

// Linien (Achsen, Messstrecke): einfarbig
const LINE_VERT = `attribute vec3 aPos; uniform mat4 uMVP; uniform float uSize;
void main(){ gl_Position = uMVP * vec4(aPos,1.0); gl_PointSize = uSize; }`;
const LINE_FRAG = `precision mediump float; uniform vec3 uColor; void main(){ gl_FragColor = vec4(uColor,1.0); }`;

function mat4Mul(a, b) {
  const o = new Float32Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    o[i * 4 + j] = a[j] * b[i * 4] + a[4 + j] * b[i * 4 + 1] + a[8 + j] * b[i * 4 + 2] + a[12 + j] * b[i * 4 + 3];
  }
  return o;
}
function perspective(fovy, asp, n, f) {
  const t = 1 / Math.tan(fovy / 2);
  return new Float32Array([t / asp, 0, 0, 0, 0, t, 0, 0, 0, 0, (f + n) / (n - f), -1, 0, 0, 2 * f * n / (n - f), 0]);
}
function lookAt(eye, c, up) {
  const z = norm(sub(eye, c)), x = norm(cross(up, z)), y = cross(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
    -dot(x, eye), -dot(y, eye), -dot(z, eye), 1]);
}
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

class Viewer3D {
  constructor(canvas) {
    this.canvas = canvas;
    const gl = this.gl = canvas.getContext('webgl', { antialias: true, preserveDrawingBuffer: true });  // für PNG-Export
    if (!gl) { console.error('WebGL nicht verfügbar'); return; }
    this.prog = this._program(VERT, FRAG);
    this.aPos = gl.getAttribLocation(this.prog, 'aPos');
    this.aInt = gl.getAttribLocation(this.prog, 'aInt');
    this.u = {};
    for (const n of ['uMVP', 'uSize', 'uMode', 'uRange', 'uClip']) this.u[n] = gl.getUniformLocation(this.prog, n);
    this.lineProg = this._program(LINE_VERT, LINE_FRAG);
    this.lAPos = gl.getAttribLocation(this.lineProg, 'aPos');
    this.lU = { mvp: gl.getUniformLocation(this.lineProg, 'uMVP'), col: gl.getUniformLocation(this.lineProg, 'uColor'),
                size: gl.getUniformLocation(this.lineProg, 'uSize') };
    this.tmpBuf = gl.createBuffer();

    this.chunks = [];          // {posBuf, intBuf, count, pos (CPU), inten (CPU)}
    this.pending = { xyz: [], inten: [] };
    this.bbox = null;          // {min:[x,y,z], max:[x,y,z], rmax}
    this.total = 0;

    // Darstellung
    this.colorMode = 0;        // 0 Intensität, 1 Höhe, 2 Entfernung, 3 einfarbig
    this.pointSize = 2.0;
    this.clip = [-1e9, 1e9];   // Höhenschnitt [m]
    this.onInfo = null;        // Rückruf für Messwerte/Status (Text)

    // Kamera: auf +Y-Seite, Blick nach -Y, Z oben
    this.theta = Math.PI / 2; this.phi = 1.15; this.dist = 8.0;
    this.target = [0, 0, 0];
    this._drag = null; this._lx = 0; this._ly = 0;
    this.measuring = false; this.picks = [];
    this.axisLen = 1.0;
    this._initAxes();
    this._initLabels();
    this._bindControls();
    this._resize();
    window.addEventListener('resize', () => this._resize());
    this._loop();
  }

  // --- Achsen + Beschriftung --------------------------------------------
  _initAxes() {
    const gl = this.gl, L = this.axisLen;
    this._axes = [
      { color: [0.95, 0.30, 0.27], tip: [L, 0, 0], name: 'X' },
      { color: [0.25, 0.73, 0.31], tip: [0, L, 0], name: 'Y' },
      { color: [0.30, 0.55, 0.95], tip: [0, 0, L], name: 'Z' },
    ];
    for (const ax of this._axes) {
      ax.buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, ax.buf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([0, 0, 0, ...ax.tip]), gl.STATIC_DRAW);
    }
  }

  _initLabels() {
    const host = this.canvas.parentNode;
    let ov = host.querySelector('.v3d-labels');
    if (!ov) { ov = document.createElement('div'); ov.className = 'v3d-labels'; host.appendChild(ov); }
    this._overlay = ov;
    this._labelEls = {};
    for (const ax of this._axes) {
      const el = document.createElement('span');
      el.className = 'v3d-axis-label'; el.textContent = ax.name;
      el.style.color = `rgb(${ax.color.map(c => Math.round(c * 255)).join(',')})`;
      ov.appendChild(el); this._labelEls[ax.name] = el;
    }
    this._scaleEl = document.createElement('span');
    this._scaleEl.className = 'v3d-scale';
    ov.appendChild(this._scaleEl);
    this._measEl = document.createElement('span');
    this._measEl.className = 'v3d-measure';
    ov.appendChild(this._measEl);
    this._legendEl = document.createElement('div');
    this._legendEl.className = 'v3d-legend';
    ov.appendChild(this._legendEl);
    this._updateLegend();
  }

  _program(vs, fs) {
    const gl = this.gl;
    const c = (t, s) => { const sh = gl.createShader(t); gl.shaderSource(sh, s); gl.compileShader(sh);
      if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) console.error(gl.getShaderInfoLog(sh)); return sh; };
    const p = gl.createProgram();
    gl.attachShader(p, c(gl.VERTEX_SHADER, vs)); gl.attachShader(p, c(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) console.error(gl.getProgramInfoLog(p));
    return p;
  }

  _resize() {
    const r = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, r.width * devicePixelRatio);
    this.canvas.height = Math.max(1, r.height * devicePixelRatio);
  }

  // --- Bedienung ---------------------------------------------------------
  _bindControls() {
    const cv = this.canvas;
    cv.addEventListener('contextmenu', e => e.preventDefault());
    cv.addEventListener('mousedown', e => {
      this._drag = (e.button === 2 || e.shiftKey) ? 'pan' : 'rot';
      this._lx = e.clientX; this._ly = e.clientY; this._moved = false;
    });
    window.addEventListener('mouseup', e => {
      if (this._drag === 'rot' && !this._moved && this.measuring && e.target === cv) this._pickAt(e);
      this._drag = null;
    });
    window.addEventListener('mousemove', e => {
      if (!this._drag) return;
      const dx = e.clientX - this._lx, dy = e.clientY - this._ly;
      if (Math.abs(dx) + Math.abs(dy) > 2) this._moved = true;
      if (this._drag === 'rot') this._rotate(dx, dy); else this._pan(dx, dy);
      this._lx = e.clientX; this._ly = e.clientY;
    });
    cv.addEventListener('wheel', e => { e.preventDefault(); this.dist = Math.max(0.05, this.dist * (1 + Math.sign(e.deltaY) * 0.1)); },
      { passive: false });
    cv.addEventListener('dblclick', e => { const p = this._nearestPoint(e); if (p) this.target = p; });
    // Touch: 1 Finger drehen, 2 Finger zoomen/verschieben
    let tPrev = null;
    cv.addEventListener('touchstart', e => { tPrev = [...e.touches].map(t => [t.clientX, t.clientY]); }, { passive: true });
    cv.addEventListener('touchmove', e => {
      e.preventDefault();
      const t = [...e.touches].map(t => [t.clientX, t.clientY]);
      if (tPrev && t.length === 1 && tPrev.length === 1) this._rotate(t[0][0] - tPrev[0][0], t[0][1] - tPrev[0][1]);
      if (tPrev && t.length === 2 && tPrev.length === 2) {
        const d0 = Math.hypot(tPrev[0][0] - tPrev[1][0], tPrev[0][1] - tPrev[1][1]);
        const d1 = Math.hypot(t[0][0] - t[1][0], t[0][1] - t[1][1]);
        if (d0 > 0 && d1 > 0) this.dist = Math.max(0.05, this.dist * d0 / d1);
        this._pan((t[0][0] + t[1][0] - tPrev[0][0] - tPrev[1][0]) / 2, (t[0][1] + t[1][1] - tPrev[0][1] - tPrev[1][1]) / 2);
      }
      tPrev = t;
    }, { passive: false });
  }

  _rotate(dx, dy) {
    this.theta -= dx * 0.01;
    this.phi = Math.max(0.02, Math.min(Math.PI - 0.02, this.phi - dy * 0.01));
  }

  _pan(dx, dy) {
    const eye = this._eye();
    const f = norm(sub(this.target, eye)), r = norm(cross(f, [0, 0, 1])), u = cross(r, f);
    const k = this.dist * 0.0015;
    for (let i = 0; i < 3; i++) this.target[i] += (-dx * r[i] + dy * u[i]) * k;
  }

  _eye() {
    return [this.target[0] + this.dist * Math.sin(this.phi) * Math.cos(this.theta),
            this.target[1] + this.dist * Math.sin(this.phi) * Math.sin(this.theta),
            this.target[2] + this.dist * Math.cos(this.phi)];
  }

  /* Feste Ansichten: 'top', 'front', 'side', 'iso', 'reset' (auf Wolke einpassen) */
  setView(name) {
    const b = this.bbox;
    if (b) {
      this.target = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2];
      const span = Math.max(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]);
      this.dist = Math.max(1, span * 1.2);
    } else { this.target = [0, 0, 0]; this.dist = 8; }
    if (name === 'top') { this.phi = 0.02; this.theta = -Math.PI / 2; }
    else if (name === 'front') { this.phi = Math.PI / 2; this.theta = -Math.PI / 2; }
    else if (name === 'side') { this.phi = Math.PI / 2; this.theta = 0; }
    else { this.phi = 1.0; this.theta = -Math.PI / 4; }
  }

  // --- Daten -------------------------------------------------------------
  addPoints(xyz, inten) {
    this.pending.xyz.push(xyz); this.pending.inten.push(inten);
    let n = 0; for (const a of this.pending.xyz) n += a.length;
    if (n >= 15000) this._flush();
  }

  _flush() {
    const gl = this.gl;
    let n3 = 0; for (const a of this.pending.xyz) n3 += a.length;
    if (n3 === 0) return;
    const pos = new Float32Array(n3); const it = new Float32Array(n3 / 3);
    let o = 0, io = 0;
    for (let i = 0; i < this.pending.xyz.length; i++) {
      pos.set(this.pending.xyz[i], o); o += this.pending.xyz[i].length;
      it.set(this.pending.inten[i], io); io += this.pending.inten[i].length;
    }
    // Ausdehnung fortschreiben
    const b = this.bbox || { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity], rmax: 0 };
    for (let i = 0; i < pos.length; i += 3) {
      const x = pos[i], y = pos[i + 1], z = pos[i + 2];
      if (x < b.min[0]) b.min[0] = x; if (x > b.max[0]) b.max[0] = x;
      if (y < b.min[1]) b.min[1] = y; if (y > b.max[1]) b.max[1] = y;
      if (z < b.min[2]) b.min[2] = z; if (z > b.max[2]) b.max[2] = z;
      const r = x * x + y * y + z * z; if (r > b.rmax) b.rmax = r;
    }
    this.bbox = b;
    const posBuf = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, posBuf); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
    const intBuf = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, intBuf); gl.bufferData(gl.ARRAY_BUFFER, it, gl.STATIC_DRAW);
    this.chunks.push({ posBuf, intBuf, count: n3 / 3, pos, inten: it });
    this.total += n3 / 3;
    this.pending = { xyz: [], inten: [] };
    this._updateLegend();
  }

  clear() {
    const gl = this.gl;
    for (const c of this.chunks) { gl.deleteBuffer(c.posBuf); gl.deleteBuffer(c.intBuf); }
    this.chunks = []; this.pending = { xyz: [], inten: [] };
    this.bbox = null; this.total = 0; this.picks = []; this._showMeasure();
    this._updateLegend();
  }

  /* Statistik für die Anzeige: Punktzahl, Ausdehnung [m] */
  stats() {
    const b = this.bbox;
    return { points: this.total, bbox: b ? { min: b.min.slice(), max: b.max.slice(), rmax: Math.sqrt(b.rmax) } : null };
  }

  /* Punkte im Höhenschnitt (für Export): liefert {xyz, inten} */
  visiblePoints() {
    let n = 0;
    for (const c of this.chunks) for (let i = 2; i < c.pos.length; i += 3) if (c.pos[i] >= this.clip[0] && c.pos[i] <= this.clip[1]) n++;
    const xyz = new Float32Array(n * 3), inten = new Float32Array(n);
    let k = 0;
    for (const c of this.chunks) for (let i = 0, j = 0; i < c.pos.length; i += 3, j++) {
      const z = c.pos[i + 2];
      if (z < this.clip[0] || z > this.clip[1]) continue;
      xyz[k * 3] = c.pos[i]; xyz[k * 3 + 1] = c.pos[i + 1]; xyz[k * 3 + 2] = z; inten[k] = c.inten[j]; k++;
    }
    return { xyz, inten, total: n };
  }

  // --- Farbe / Schnitt -----------------------------------------------------
  setColorMode(m) { this.colorMode = m; this._updateLegend(); }
  setPointSize(s) { this.pointSize = s; }
  setClip(zmin, zmax) { this.clip = [zmin, zmax]; this._updateLegend(); }

  _range() {
    const b = this.bbox;
    if (this.colorMode === 0) return [0, 255];
    if (!b) return [0, 1];
    if (this.colorMode === 1) return [Math.max(b.min[2], this.clip[0]), Math.min(b.max[2], this.clip[1])];
    return [0, Math.min(Math.sqrt(b.rmax), 10)];
  }

  _updateLegend() {
    if (!this._legendEl) return;
    const names = ['Intensität', 'Höhe [m]', 'Entfernung [m]', 'einfarbig'];
    const r = this._range();
    const fmt = v => this.colorMode === 0 ? v.toFixed(0) : v.toFixed(2);
    this._legendEl.innerHTML = this.colorMode === 3 || this.colorMode === 0 ? `<span>${names[this.colorMode]}</span>`
      : `<span>${names[this.colorMode]}</span><div class="bar"></div><span class="lo">${fmt(r[0])}</span><span class="hi">${fmt(r[1])}</span>`;
    const b = this.bbox;
    this._scaleEl.textContent = b
      ? `Achsen = 1 m · Ausdehnung ${(b.max[0] - b.min[0]).toFixed(2)} × ${(b.max[1] - b.min[1]).toFixed(2)} × ${(b.max[2] - b.min[2]).toFixed(2)} m`
      : 'Achsen = 1 m';
  }

  // --- Messen --------------------------------------------------------------
  setMeasuring(on) { this.measuring = on; if (!on) { this.picks = []; this._showMeasure(); } this.canvas.style.cursor = on ? 'crosshair' : ''; }

  _nearestPoint(e) {
    // nächster sichtbarer Punkt zum Mauszeiger (Bildschirmabstand ≤ 10 px, davon der vorderste)
    const rect = this.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left, my = e.clientY - rect.top;
    const W = rect.width, H = rect.height, m = this._mvp;
    if (!m) return null;
    let best = null, bestScore = Infinity;
    for (const c of this.chunks) {
      const p = c.pos;
      for (let i = 0; i < p.length; i += 3) {
        const x = p[i], y = p[i + 1], z = p[i + 2];
        if (z < this.clip[0] || z > this.clip[1]) continue;
        const cw = m[3] * x + m[7] * y + m[11] * z + m[15];
        if (cw <= 0.05) continue;
        const sx = ((m[0] * x + m[4] * y + m[8] * z + m[12]) / cw + 1) / 2 * W;
        const sy = (1 - (m[1] * x + m[5] * y + m[9] * z + m[13]) / cw) / 2 * H;
        const d2 = (sx - mx) ** 2 + (sy - my) ** 2;
        if (d2 > 100) continue;
        const score = d2 / 100 + cw * 0.05;                     // nah am Zeiger, vorne bevorzugt
        if (score < bestScore) { bestScore = score; best = [x, y, z]; }
      }
    }
    return best;
  }

  _pickAt(e) {
    const p = this._nearestPoint(e);
    if (!p) return;
    if (this.picks.length >= 2) this.picks = [];
    this.picks.push(p);
    this._showMeasure();
  }

  _showMeasure() {
    if (!this._measEl) return;
    let txt = '';
    if (this.picks.length === 1) txt = `P1 (${this.picks[0].map(v => v.toFixed(3)).join(' | ')}) m – zweiten Punkt anklicken`;
    if (this.picks.length === 2) {
      const [a, b] = this.picks, d = sub(b, a);
      txt = `Abstand ${Math.hypot(...d).toFixed(3)} m  (Δx ${d[0].toFixed(3)}  Δy ${d[1].toFixed(3)}  Δz ${d[2].toFixed(3)})`;
    }
    this._measEl.textContent = txt;
    this._measEl.style.display = txt ? 'block' : 'none';
    if (this.onInfo) this.onInfo(txt);
  }

  // --- Zeichnen ------------------------------------------------------------
  _projectToScreen(p, mvp) {
    const x = p[0], y = p[1], z = p[2];
    const cx = mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12];
    const cy = mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13];
    const cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
    if (cw <= 1e-6) return null;
    const W = this.canvas.width / devicePixelRatio, H = this.canvas.height / devicePixelRatio;
    return { px: (cx / cw + 1) / 2 * W, py: (1 - cy / cw) / 2 * H };
  }

  _drawLines(mvp) {
    const gl = this.gl;
    gl.useProgram(this.lineProg);
    gl.uniformMatrix4fv(this.lU.mvp, false, mvp);
    gl.uniform1f(this.lU.size, 9.0 * devicePixelRatio);
    gl.enableVertexAttribArray(this.lAPos);
    for (const ax of this._axes) {
      gl.uniform3fv(this.lU.col, ax.color);
      gl.bindBuffer(gl.ARRAY_BUFFER, ax.buf);
      gl.vertexAttribPointer(this.lAPos, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.LINES, 0, 2);
      const s = this._projectToScreen(ax.tip, mvp), el = this._labelEls[ax.name];
      if (s) { el.style.display = 'block'; el.style.left = `${s.px}px`; el.style.top = `${s.py}px`; }
      else el.style.display = 'none';
    }
    if (this.picks.length) {                                   // Messpunkte + Strecke
      gl.disable(gl.DEPTH_TEST);
      gl.uniform3fv(this.lU.col, [1.0, 0.85, 0.1]);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.tmpBuf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.picks.flat()), gl.DYNAMIC_DRAW);
      gl.vertexAttribPointer(this.lAPos, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.POINTS, 0, this.picks.length);
      if (this.picks.length === 2) gl.drawArrays(gl.LINES, 0, 2);
      gl.enable(gl.DEPTH_TEST);
    }
  }

  _loop() {
    if (!this.gl) return;
    this._flush();
    const gl = this.gl, W = this.canvas.width, H = this.canvas.height;
    gl.viewport(0, 0, W, H); gl.clearColor(0.02, 0.03, 0.04, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.enable(gl.DEPTH_TEST);
    const view = lookAt(this._eye(), this.target, [0, 0, 1]);
    const proj = perspective(1.0, W / H, 0.02, 500);
    const mvp = this._mvp = mat4Mul(proj, view);
    gl.useProgram(this.prog);
    gl.uniformMatrix4fv(this.u.uMVP, false, mvp);
    gl.uniform1f(this.u.uSize, this.pointSize * devicePixelRatio);
    gl.uniform1i(this.u.uMode, this.colorMode);
    gl.uniform2fv(this.u.uRange, this._range());
    gl.uniform2fv(this.u.uClip, this.clip);
    for (const c of this.chunks) {
      gl.bindBuffer(gl.ARRAY_BUFFER, c.posBuf); gl.enableVertexAttribArray(this.aPos);
      gl.vertexAttribPointer(this.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, c.intBuf); gl.enableVertexAttribArray(this.aInt);
      gl.vertexAttribPointer(this.aInt, 1, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.POINTS, 0, c.count);
    }
    this._drawLines(mvp);
    requestAnimationFrame(() => this._loop());
  }
}

window.Viewer3D = Viewer3D;
