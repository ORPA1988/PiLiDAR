/* Minimalistischer WebGL-Punktwolken-Viewer ohne externe Bibliothek.
 * Orbit-Kamera (Maus ziehen = rotieren, Rad = zoomen). Punkte werden in
 * wachsende GPU-Puffer (Chunks) geschrieben, intensitätsbasiert eingefärbt. */
'use strict';

const VERT = `
attribute vec3 aPos; attribute float aInt;
uniform mat4 uMVP; uniform float uSize;
varying float vInt;
void main(){ gl_Position = uMVP * vec4(aPos,1.0); gl_PointSize = uSize; vInt = aInt; }`;

const FRAG = `
precision mediump float; varying float vInt;
void main(){ float g = clamp(vInt/255.0,0.05,1.0); gl_FragColor = vec4(g*0.5, g, g*0.7, 1.0); }`;

// Eigenes Programm für Achsenlinien (einfarbig, ohne Intensität)
const AXIS_VERT = `
attribute vec3 aPos; uniform mat4 uMVP;
void main(){ gl_Position = uMVP * vec4(aPos,1.0); }`;
const AXIS_FRAG = `
precision mediump float; uniform vec3 uColor;
void main(){ gl_FragColor = vec4(uColor,1.0); }`;

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
    const gl = this.gl = canvas.getContext('webgl', { antialias: true });
    if (!gl) { console.error('WebGL nicht verfügbar'); return; }
    this.prog = this._program(VERT, FRAG);
    this.aPos = gl.getAttribLocation(this.prog, 'aPos');
    this.aInt = gl.getAttribLocation(this.prog, 'aInt');
    this.uMVP = gl.getUniformLocation(this.prog, 'uMVP');
    this.uSize = gl.getUniformLocation(this.prog, 'uSize');

    this.chunks = [];          // {posBuf, intBuf, count}
    this.pending = { xyz: [], inten: [] };
    // Kamera auf der +Y-Seite, Blick zur -Y-Richtung, Z = oben.
    // So zeigt im Bild +X nach LINKS und +Z nach OBEN (Rechte-Hand-Regel:
    // Beobachter blickt entlang der Y-Achse, X+ links, Z+ oben).
    // theta = π/2 hält die Kamera exakt auf der Y-Achse; phi < π/2 kippt leicht von oben.
    this.theta = Math.PI / 2; this.phi = 1.15; this.dist = 8.0;
    this.target = [0, 0, 0];
    this._drag = false; this._lx = 0; this._ly = 0;
    this.axisLen = 1.0;        // Achsenlänge / Maßstab [m]
    this._initAxes();
    this._initLabels();
    this._bindControls();
    this._resize();
    window.addEventListener('resize', () => this._resize());
    this._loop();
  }

  // --- Koordinatenachsen (X rot, Y grün, Z blau) ----------------------
  _initAxes() {
    const gl = this.gl;
    this._axisProg = this._program(AXIS_VERT, AXIS_FRAG);
    this._axisAPos = gl.getAttribLocation(this._axisProg, 'aPos');
    this._axisUMVP = gl.getUniformLocation(this._axisProg, 'uMVP');
    this._axisUColor = gl.getUniformLocation(this._axisProg, 'uColor');
    const L = this.axisLen;
    this._axes = [
      { color: [0.95, 0.30, 0.27], tip: [L, 0, 0], name: 'X' },
      { color: [0.25, 0.73, 0.31], tip: [0, L, 0], name: 'Y' },
      { color: [0.30, 0.55, 0.95], tip: [0, 0, L], name: 'Z' },
    ];
    for (const ax of this._axes) {
      const buf = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      gl.bufferData(gl.ARRAY_BUFFER,
        new Float32Array([0, 0, 0, ax.tip[0], ax.tip[1], ax.tip[2]]), gl.STATIC_DRAW);
      ax.buf = buf;
    }
  }

  // HTML-Overlay-Labels über dem Canvas (WebGL kann keinen Text)
  _initLabels() {
    const host = this.canvas.parentNode;
    let ov = host.querySelector('.v3d-labels');
    if (!ov) {
      ov = document.createElement('div');
      ov.className = 'v3d-labels';
      host.appendChild(ov);
    }
    this._overlay = ov;
    this._labelEls = {};
    for (const ax of this._axes) {
      const el = document.createElement('span');
      el.className = 'v3d-axis-label';
      el.textContent = ax.name;
      el.style.color = `rgb(${ax.color.map(c => Math.round(c * 255)).join(',')})`;
      ov.appendChild(el);
      this._labelEls[ax.name] = el;
    }
    // statischer Maßstab unten links
    this._scaleEl = document.createElement('span');
    this._scaleEl.className = 'v3d-scale';
    this._scaleEl.textContent = `Achsen = ${this.axisLen.toFixed(0)} m`;
    ov.appendChild(this._scaleEl);
  }

  _projectToScreen(p, mvp) {
    // clip = mvp · [x,y,z,1]  (mvp ist spaltenweise gespeichert)
    const x = p[0], y = p[1], z = p[2];
    const cx = mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12];
    const cy = mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13];
    const cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
    if (cw <= 1e-6) return null;                  // hinter der Kamera
    const W = this.canvas.width / devicePixelRatio;
    const H = this.canvas.height / devicePixelRatio;
    return { px: (cx / cw + 1) / 2 * W, py: (1 - cy / cw) / 2 * H };
  }

  _drawAxes(mvp) {
    const gl = this.gl;
    gl.useProgram(this._axisProg);
    gl.uniformMatrix4fv(this._axisUMVP, false, mvp);
    gl.lineWidth(2.0);
    for (const ax of this._axes) {
      gl.uniform3fv(this._axisUColor, ax.color);
      gl.bindBuffer(gl.ARRAY_BUFFER, ax.buf);
      gl.enableVertexAttribArray(this._axisAPos);
      gl.vertexAttribPointer(this._axisAPos, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.LINES, 0, 2);
      // Label an die projizierte Achsenspitze setzen
      const s = this._projectToScreen(ax.tip, mvp);
      const el = this._labelEls[ax.name];
      if (s) { el.style.display = 'block'; el.style.left = `${s.px}px`; el.style.top = `${s.py}px`; }
      else { el.style.display = 'none'; }
    }
  }

  _program(vs, fs) {
    const gl = this.gl;
    const c = (t, s) => { const sh = gl.createShader(t); gl.shaderSource(sh, s); gl.compileShader(sh);
      if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) console.error(gl.getShaderInfoLog(sh)); return sh; };
    const p = gl.createProgram();
    gl.attachShader(p, c(gl.VERTEX_SHADER, vs)); gl.attachShader(p, c(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p); return p;
  }

  _resize() {
    const r = this.canvas.getBoundingClientRect();
    this.canvas.width = r.width * devicePixelRatio;
    this.canvas.height = r.height * devicePixelRatio;
  }

  _bindControls() {
    const cv = this.canvas;
    cv.addEventListener('mousedown', e => { this._drag = true; this._lx = e.clientX; this._ly = e.clientY; });
    window.addEventListener('mouseup', () => this._drag = false);
    window.addEventListener('mousemove', e => {
      if (!this._drag) return;
      this.theta -= (e.clientX - this._lx) * 0.01;
      this.phi = Math.max(0.05, Math.min(Math.PI - 0.05, this.phi - (e.clientY - this._ly) * 0.01));
      this._lx = e.clientX; this._ly = e.clientY;
    });
    cv.addEventListener('wheel', e => { e.preventDefault(); this.dist *= (1 + Math.sign(e.deltaY) * 0.1); }, { passive: false });
  }

  addPoints(xyz, inten) {
    // sammeln und gebündelt in GPU-Puffer schreiben (alle ~5000 Punkte)
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
    const posBuf = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, posBuf); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
    const intBuf = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, intBuf); gl.bufferData(gl.ARRAY_BUFFER, it, gl.STATIC_DRAW);
    this.chunks.push({ posBuf, intBuf, count: n3 / 3 });
    this.pending = { xyz: [], inten: [] };
  }

  clear() {
    const gl = this.gl;
    for (const c of this.chunks) { gl.deleteBuffer(c.posBuf); gl.deleteBuffer(c.intBuf); }
    this.chunks = []; this.pending = { xyz: [], inten: [] };
  }

  _loop() {
    if (!this.gl) return;
    this._flush();
    const gl = this.gl, W = this.canvas.width, H = this.canvas.height;
    gl.viewport(0, 0, W, H); gl.clearColor(0.02, 0.03, 0.04, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.enable(gl.DEPTH_TEST);
    const eye = [
      this.target[0] + this.dist * Math.sin(this.phi) * Math.cos(this.theta),
      this.target[1] + this.dist * Math.sin(this.phi) * Math.sin(this.theta),
      this.target[2] + this.dist * Math.cos(this.phi)];
    const view = lookAt(eye, this.target, [0, 0, 1]);
    const proj = perspective(1.0, W / H, 0.05, 500);
    const mvp = mat4Mul(proj, view);
    gl.useProgram(this.prog);
    gl.uniformMatrix4fv(this.uMVP, false, mvp);
    gl.uniform1f(this.uSize, 2.0 * devicePixelRatio);
    for (const c of this.chunks) {
      gl.bindBuffer(gl.ARRAY_BUFFER, c.posBuf); gl.enableVertexAttribArray(this.aPos);
      gl.vertexAttribPointer(this.aPos, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, c.intBuf); gl.enableVertexAttribArray(this.aInt);
      gl.vertexAttribPointer(this.aInt, 1, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.POINTS, 0, c.count);
    }
    this._drawAxes(mvp);
    requestAnimationFrame(() => this._loop());
  }
}

window.Viewer3D = Viewer3D;
