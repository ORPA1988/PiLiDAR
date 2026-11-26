"""Nachruestsatz fuer das unveraenderte Original-Gehaeuse: Getriebedeckel mit angeformter Lagerhuelse.

python cad/nachruest_original.py <Pfad>/PiLiDAR-Hardware/FDM/gearbox_deckel.obj
  -> out/2_lagerdeckel_original.stl  (ersetzt gearbox_deckel; dazu 1_stativfuss und die Ringe 6-8 aus turm_v3.py)

Der Original-Deckel (PiLiDAR-Hardware, CC BY-NC-SA 4.0, Philip Gutjahr) bleibt als Netz unveraendert und wird mit
Scheibe und Lagerhuelse vereinigt. Die vier Gehaeuseschrauben sitzen wie bisher im Deckel; Schraubenlaenge unveraendert.
"""
import pathlib, sys, tempfile
import numpy as np, trimesh
from build123d import Pos, export_stl
import turm_v3 as k

lid = trimesh.load(sys.argv[1], force="mesh")          # Aussenflaeche bei z = 0, Innenseite nach +z
c = lid.bounds.mean(0); lid.apply_translation([-c[0], -c[1], -lid.bounds[0][2]])
assert lid.is_watertight
# Scheibe (Dicke DECK_T) unter dem Deckel + Lagerhuelse, in Deckelkoordinaten (z = 0 = Deckel-Aussenflaeche)
add = k.cyl(k.SL_RO, k.FL_T + 0.6, k.Z_D + 0.05) - k.cyl(k.SEAT_D / 2, k.FL_T, k.Z_DB)
add = add - k.cyl(17.0, k.Z_DB - 1, k.Z_D + 1)                              # Abtrieb (Rundteil 32,3)
holes = [(sx * 15.5, sy * 15.5) for sx in (-1, 1) for sy in (-1, 1)]
for x, y in holes:
    add = add - k.cyl(3.0, k.Z_DB - 1, k.Z_D + 1, x, y)                     # Zugang zu den vier Schraubenkoepfen
add = Pos(0, 0, -k.Z_D) * add
with tempfile.NamedTemporaryFile(suffix=".stl") as f:
    export_stl(add, f.name, tolerance=0.02, angular_tolerance=0.2); addm = trimesh.load(f.name)
# Der Original-Deckel besteht aus vier sich nur beruehrenden Huellen (Platte, Bund, zwei Augen):
# die kleinen um 0,02 mm in die Platte setzen, damit ein einziger Koerper entsteht.
shells = sorted(lid.split(only_watertight=False), key=lambda m: -m.volume)
for m in shells[1:]:
    m.apply_translation([0, 0, -0.02])
part = trimesh.boolean.union([*shells, addm], engine="manifold")
out = pathlib.Path(__file__).parent / "out" / "2_lagerdeckel_original.stl"
part.export(out)
print("Lagerdeckel:", np.round(part.extents, 1), "mm, dicht:", part.is_watertight, "Koerper:", len(part.split()), f"{part.volume/1000:.1f} cm3")
# Pruefung gegen Fuss, Lager und Abtrieb (Einbaulage: Deckel-Aussenflaeche auf z = Z_D)
part.apply_translation([0, 0, k.Z_D])
B = k.make_bestand()
for name, body in {"1_stativfuss": k.make_fuss(), "lager": B["lager"], "abtrieb": B["abtrieb"]}.items():
    with tempfile.NamedTemporaryFile(suffix=".stl") as f:
        export_stl(body, f.name, tolerance=0.05); m = trimesh.load(f.name)
    inter = trimesh.boolean.intersection([part, m], engine="manifold")
    print(f"  Durchdringung mit {name}: {0.0 if inter.is_empty else inter.volume:.1f} mm3")
