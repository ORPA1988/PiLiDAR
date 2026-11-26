"""Prueft cad/plattform_v3.py: Volumenkoerper, Durchdringungen, freie Scan-Ebene, Massen; rendert die Vorschau."""
import itertools, pathlib, tempfile
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as MPath
from matplotlib.patches import PathPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import Pos, export_stl
import plattform_v3 as k

out = pathlib.Path(__file__).parent / "out"
R = k.make_ringe()
P = {"1_stativfuss": k.make_fuss(), "2_teller": k.make_teller(), "3_aufbau": k.make_aufbau(), "4_bruecke": k.make_bruecke(),
     "5_klemmleiste_vorn": k.make_leiste(True), "5_klemmleiste_hinten": k.make_leiste(False),
     "6_ring_oben": Pos(0, 0, k.Z_B2 + k.BRG_W) * R["6_ring_oben"],
     "7_distanz_aussen": Pos(0, 0, k.Z_B1 + k.BRG_W) * R["7_distanz_aussen"],
     "8_distanz_innen": Pos(0, 0, k.Z_B1 + k.BRG_W) * R["8_distanz_innen"]}
B = k.make_bestand()
RHO, FILL = 1.27e-3, 0.8   # PETG; 4 Waende + 20 % Fuellung ~ 80 % der Vollmasse
ok = True
for n, p in P.items():
    bb = p.bounding_box(); s = p.solids(); good = len(s) == 1
    ok &= good
    print(f"{n:22s} {bb.size.X:5.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f} mm  {p.volume/1000:5.1f} cm3  ~{p.volume*RHO*FILL:4.0f} g  Koerper {len(s)}")
for f in sorted(out.glob("*.stl")):
    m = trimesh.load(f)
    if not m.is_watertight: ok = False; print("NICHT DICHT:", f.name)

allp = {**P, **B}
bad = []
for (a, pa), (b, pb) in itertools.combinations(allp.items(), 2):
    v = pa & pb
    vol = sum(s.volume for s in v.solids()) if v is not None else 0.0
    if vol > 0.5: bad.append((a, b, round(vol, 1)))
ok &= not bad
print("Durchdringungen:", bad or "keine")

# Scan-Ebene y = -LIDAR_D: ausser dem STL27L darf nichts naeher als 6 mm heranreichen
ymin = {n: p.bounding_box().min.Y for n, p in allp.items() if n != "stl27l"}
worst = min(ymin, key=ymin.get)
frei = ymin[worst] + k.LIDAR_D
ok &= frei >= 6.0
print(f"Scan-Ebene bei y = {-k.LIDAR_D:.1f}: naechstes Teil '{worst}' mit {frei:.1f} mm Abstand")

# Schwerpunkt des Drehteils mit ANGENOMMENEN Massen der Zukaufteile [g]
M = {"motor_getriebe": 190, "deckel": 5, "stl27l": 45, "pi": 50, "akku": 230, "kamera": 80}
tot, cx, cy, cz = 0.0, 0.0, 0.0, 0.0
for n, p in allp.items():
    if n in ("1_stativfuss", "8_distanz_innen", "abtrieb", "lager"): continue
    m = M.get(n, p.volume * RHO * FILL); c = p.center()
    tot += m; cx += m * c.X; cy += m * c.Y; cz += m * c.Z
print(f"Drehteil ~{tot:.0f} g (Zukaufteile angenommen), Schwerpunkt x {cx/tot:+.1f}  y {cy/tot:+.1f}  z {cz/tot:.0f} mm; Lagermitte z {(k.Z_B1 + k.Z_B2 + k.BRG_W)/2:.1f}")
print("ERGEBNIS:", "OK" if ok else "FEHLER")

COL = {"1_stativfuss": "#c0392b", "2_teller": "#5b6770", "3_aufbau": "#8fa3b0", "4_bruecke": "#2f7ed8",
       "5_klemmleiste_vorn": "#e08a2e", "5_klemmleiste_hinten": "#e08a2e", "6_ring_oben": "#f1c40f", "7_distanz_aussen": "#f1c40f",
       "8_distanz_innen": "#f1c40f", "deckel": "#b9b9b9", "motor_getriebe": "#444444", "abtrieb": "#d9d9d9", "lager": "#7f8c8d",
       "stl27l": "#222222", "pi": "#27ae60", "akku": "#8e44ad", "kamera": "#16a085"}
def mesh(p):
    with tempfile.NamedTemporaryFile(suffix=".stl") as f:
        export_stl(p, f.name, tolerance=0.1, angular_tolerance=0.5); return trimesh.load(f.name)
MESH = {n: mesh(p) for n, p in allp.items()}
def draw(ax, names, elev, azim, title):
    tris, cols = [], []
    for n in names:
        m = MESH[n]; t = m.vertices[m.faces]
        sh = 0.45 + 0.55 * np.clip(np.abs(m.face_normals @ np.array([0.35, -0.5, 0.8])), 0, 1)
        tris.append(t); cols.append(np.clip(sh[:, None] * np.array(matplotlib.colors.to_rgb(COL[n])), 0, 1))
    tris, cols = np.concatenate(tris), np.concatenate(cols)
    ax.add_collection3d(Poly3DCollection(tris, facecolors=cols, edgecolor="none"))   # eine Sammlung -> saubere Tiefensortierung
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0); c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.set_box_aspect((1, 1, 1), zoom=1.4); ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title(title, fontsize=11)
T = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])
def section(ax, n):
    sec = MESH[n].section(plane_origin=[0.01, 0, 0], plane_normal=[1, 0, 0])
    if sec is None: return
    pl, _ = sec.to_2D(to_2D=T)
    for poly in pl.polygons_full:
        v, c = [], []
        for ring in [poly.exterior, *poly.interiors]:
            xy = np.asarray(ring.coords); v += list(xy); c += [MPath.MOVETO] + [MPath.LINETO] * (len(xy) - 2) + [MPath.CLOSEPOLY]
        ax.add_patch(PathPatch(MPath(v, c), facecolor=COL[n], edgecolor="black", lw=0.4))
fig = plt.figure(figsize=(16, 7), dpi=100)
draw(fig.add_axes([0.0, 0.0, 0.33, 0.95], projection="3d"), list(allp), 20, -62, "vorne / Akkuseite")
draw(fig.add_axes([0.31, 0.0, 0.33, 0.95], projection="3d"), list(allp), 20, 128, "hinten / Pi-Seite")
ax = fig.add_axes([0.67, 0.07, 0.32, 0.85])
for n in allp: section(ax, n)
ax.axvline(0, color="k", ls="-.", lw=0.7); ax.axvline(-k.LIDAR_D, color="#c0392b", ls="--", lw=0.9)
ax.text(-k.LIDAR_D - 2, 128, "Scan-Ebene", color="#c0392b", fontsize=8, ha="right")
ax.set_xlim(-95, 70); ax.set_ylim(-5, 145); ax.set_aspect("equal"); ax.set_xlabel("y [mm]"); ax.set_ylabel("z [mm]")
ax.set_title("Schnitt durch die Drehachse (x = 0)", fontsize=11); ax.grid(alpha=0.25)
plt.savefig(out / "vorschau.png", facecolor="white")
