"""Prueft cad/kopf_v3.py: Volumenkoerper, Durchdringungen, Toleranz +-TOL, Massen; rendert Vorschau.
Aufruf: python cad/pruefen.py   (braucht build123d, trimesh, matplotlib)"""
import itertools, pathlib, tempfile
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import Pos, export_stl
import kopf_v3 as k

out = pathlib.Path(__file__).parent / "out"
P = {n: f() for n, f in k.TEILE.items()}
B = k.make_bestand()
RHO = 1.27e-3  # g/mm3 PETG, massiv gerechnet (Obergrenze)
ok = True
for n, p in P.items():
    bb = p.bounding_box(); s = p.solids()
    m = trimesh.load(out / f"{n}.stl")
    good = len(s) == 1 and m.is_watertight
    ok &= good
    print(f"{n:16s} {bb.size.X:5.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f} mm  {p.volume/1000:5.1f} cm3  <= {p.volume*RHO:4.0f} g  Koerper {len(s)}  dicht {m.is_watertight}")

def clash(shift):
    rot = {"3_nabe": P["3_nabe"], "4_lidartraeger": P["4_lidartraeger"], "stl27l": B["stl27l"], "abtrieb": B["abtrieb"]}
    rot = {n: Pos(0, 0, shift) * p for n, p in rot.items()}
    allp = {"1_boden": P["1_boden"], "2_glocke": P["2_glocke"], "motor": B["motor"], "getriebe": B["getriebe"], "lager": B["lager"], **rot}
    bad = []
    for (a, pa), (b, pb) in itertools.combinations(allp.items(), 2):
        v = (pa & pb)
        vol = sum(s.volume for s in v.solids()) if v is not None else 0.0
        if vol > 0.5 and {a, b} != {'getriebe', 'abtrieb'}: bad.append((a, b, round(vol, 1)))   # Pruefkoerper Getriebe ist starr
    return bad
for sh in (0, -k.TOL, k.TOL):
    bad = clash(sh); ok &= not bad
    print(f"Durchdringung bei Abtriebshoehe {sh:+.0f} mm:", bad or "keine")

rot_mass = (P["3_nabe"].volume + P["4_lidartraeger"].volume) * RHO
print(f"Drehende Druckteile zusammen <= {rot_mass:.0f} g (massiv); dazu STL27L, Lagerinnenring, Schrauben")
# Schwerpunkt Traeger+Nabe gegen Achse
c = (P["3_nabe"].center() * P["3_nabe"].volume + P["4_lidartraeger"].center() * P["4_lidartraeger"].volume) * (1 / (P["3_nabe"].volume + P["4_lidartraeger"].volume))
print(f"Schwerpunkt drehende Druckteile: x {c.X:.1f}  y {c.Y:.1f} mm neben der Achse")
print(f"LiDAR-Achse {k.ZC - (k.Z_BRG0 + k.BRG_W/2):.1f} mm ueber Lagermitte; Gesamthoehe {k.ZC + k.L_HOLE_PITCH/2 + 4.1:.1f} mm")
print("ERGEBNIS:", "OK" if ok else "FEHLER")

# ---------------- Vorschau
COL = {"1_boden": "#5b6770", "2_glocke": "#8fa3b0", "3_nabe": "#e08a2e", "4_lidartraeger": "#2f7ed8",
       "motor": "#444444", "getriebe": "#b9b9b9", "abtrieb": "#d9d9d9", "lager": "#c0392b", "stl27l": "#222222"}
def mesh(p):
    with tempfile.NamedTemporaryFile(suffix=".stl") as f:
        export_stl(p, f.name, tolerance=0.08, angular_tolerance=0.4); return trimesh.load(f.name)
def draw(ax, items, elev, azim, title):
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    for p, col in items:
        m = mesh(p); tri = m.vertices[m.faces]
        sh = 0.45 + 0.55 * np.clip(np.abs(m.face_normals @ np.array([0.35, -0.5, 0.8])), 0, 1)
        rgb = np.array(matplotlib.colors.to_rgb(col))
        ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(sh[:, None] * rgb, 0, 1), edgecolor="none"))
        lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
    c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.set_box_aspect((1, 1, 1), zoom=1.45); ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title(title, fontsize=11)
allp = {**P, **B}
from matplotlib.path import Path as MPath
from matplotlib.patches import PathPatch
T = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1.0]])   # (x,y,z) -> (y,z,x)
def section(ax, p, col):
    sec = mesh(p).section(plane_origin=[0.01, 0, 0], plane_normal=[1, 0, 0])
    if sec is None: return
    pl, _ = sec.to_2D(to_2D=T)
    for poly in pl.polygons_full:
        v, c = [], []
        for ring in [poly.exterior, *poly.interiors]:
            xy = np.asarray(ring.coords); v += list(xy); c += [MPath.MOVETO] + [MPath.LINETO] * (len(xy) - 2) + [MPath.CLOSEPOLY]
        ax.add_patch(PathPatch(MPath(v, c), facecolor=col, edgecolor="black", lw=0.5))
show = ("1_boden", "2_glocke", "3_nabe", "4_lidartraeger", "stl27l", "lager")
fig = plt.figure(figsize=(15, 7), dpi=100)
draw(fig.add_axes([0.0, 0.0, 0.34, 0.95], projection="3d"), [(allp[n], COL[n]) for n in show], 16, -58, "LiDAR-Seite")
draw(fig.add_axes([0.30, 0.0, 0.34, 0.95], projection="3d"), [(allp[n], COL[n]) for n in show], 20, 140, "Rueckseite")
ax = fig.add_axes([0.66, 0.06, 0.33, 0.86])
for n in COL: section(ax, allp[n], COL[n])
ax.axvline(0, color="k", ls="-.", lw=0.7)
for z, t in ((k.Z_OUT, "Abtrieb"), (k.Z_BRG0 + k.BRG_W / 2, "Lager 6806"), (k.ZC, "LiDAR-Achse")):
    ax.plot([42, 50], [z, z], color="k", lw=0.6); ax.text(51, z, f"{t}  z = {z:.1f}", va="center", fontsize=8)
ax.set_xlim(-45, 95); ax.set_ylim(-5, 150); ax.set_aspect("equal"); ax.set_xlabel("y [mm]"); ax.set_ylabel("z [mm]")
ax.set_title("Schnitt durch die Drehachse (x = 0)", fontsize=11); ax.grid(alpha=0.25)
plt.savefig(out / "vorschau.png", facecolor="white")
