"""PiLiDAR v3 - Scankopf Stufe 2 (gelagerter Abtrieb, nur LiDAR dreht).

Erzeugt mit build123d:  python cad/kopf_v3.py   ->  cad/out/*.step, *.stl, geometrie.json
Koordinaten: Z = Drehachse (oben), Ursprung = Unterseite Bodenplatte.
Scan-Ebene des STL27L = X-Z-Ebene (y = 0), enthaelt die Drehachse.

Quellen der Masse
  STL27L   : Datenblatt (PiLiDAR-Hardware/Datasheets/STL27L_Datasheet.pdf), Abschnitt 5.1
  Abtrieb  : PiLiDAR-Hardware/FDM "gearbox kameraplatte.obj", "gearbox_holder.obj", "gearbox_deckel.obj"
  GEAR_STACK_H ist aus diesen Dateien ABGELEITET (nicht gemessen) -> Zapfen laesst +-4 mm zu.
"""
from build123d import *
import json, math, pathlib

# ---------------------------------------------------------------- Parameter
# Motor / Getriebe (Bestand)
NEMA = 42.3            # Kantenlaenge NEMA 17
MOTOR_L = 23.0         # Laenge Motorkoerper (17HS4023)
GEAR_STACK_H = 33.0    # Motorstirnflaeche -> Aussenflaeche Abtriebs-Vierkant (abgeleitet)
GEAR_SQ = 52.3         # Getriebedeckel Kantenlaenge
GEAR_R = 32.2          # Getriebedeckel groesster Radius (gefaste Ecken)
SQ_OUT = 32.3          # Abtriebs-Vierkant
SQ_T = 5.0             # Dicke des Vierkants
OUT_NUT_DEPTH = 7.6    # Aussenflaeche -> Beginn Mutter 1/4" im Abtrieb
OUT_NUT_END = 13.1     # Aussenflaeche -> Ende Mutter (dahinter Bohrung nur 5,5 mm)

# Passungen (nach Passprobe anpassen)
SQ_POCKET = 32.5       # Vierkant-Tasche (wie Stufe 1c)
SQ_DEPTH = 4.0
J_D = 29.9             # Zapfen fuer Lager-Innenring 30 mm (Schiebesitz)
SEAT_D = 42.1          # Lagersitz fuer Aussenring 42 mm
MOTOR_CLEAR = 0.5      # Spiel Motor zu den Eckpfosten je Seite

# Lager 6806-2RS
BRG_ID, BRG_OD, BRG_W = 30.0, 42.0, 7.0

# Schrauben
SCREW_L = 19.05        # 1/4"-20 x 3/4"
D_M3_FORM = 2.9        # Kernloch M3 direkt in Kunststoff
D_M3_THRU = 3.4
D_M2_THRU = 2.2

# Aufbau
BASE_T = 8.0
R_IN, WALL = 36.0, 3.0
R_OUT = R_IN + WALL
TOL = 4.0              # zulaessige Abweichung von GEAR_STACK_H
RIB_R0, RIB_W = 27.5, 9.0
TOP_CH = 8.5           # Fase Glockenoberkante
SCREW_R = 31.5         # Teilkreis Boden <-> Glocke
HUB_BOLT_R = 10.75     # Teilkreis Traeger <-> Nabe

# STL27L (Datenblatt)
L_BODY = 38.1          # Sockel quadratisch (aus Zeichnung, +-0,5)
L_H = 34.8             # Gesamthoehe
L_TUR_D, L_TUR_H = 35.29, 12.6
L_OPT = L_H - 7.8      # optische Ebene ueber der Montageflaeche = 27,0
L_HOLE_PITCH = 46.8    # Abstand der zwei Befestigungsloecher
L_HOLE_3 = 31.92       # Abstand zum dritten Loch
L_HOLE_EDGE = 10.3     # Zweilochlinie -> Sockelkante (aus Zeichnung, +-0,5)

# ---------------------------------------------------------------- abgeleitete Hoehen
Z_MOT0 = BASE_T
Z_MOT1 = Z_MOT0 + MOTOR_L
Z_OUT = Z_MOT1 + GEAR_STACK_H                 # Aussenflaeche Abtrieb
Z_HUB0 = Z_OUT - SQ_DEPTH
Z_COLLAR1 = Z_OUT + 4.0
Z_SEAT = Z_OUT + (SCREW_L - (OUT_NUT_END - 1.0))   # Kopfauflage: Schraube endet 1 mm vor Mutterende
Z_ROOF0 = Z_COLLAR1 + TOL + 1.0
Z_BRG0 = Z_ROOF0 + 2.0
Z_TOP = Z_BRG0 + BRG_W                         # Oberkante Glocke
Z_J1 = Z_TOP + TOL + 1.0                       # Zapfenende = Unterseite Traeger
FLOOR_T, WALL_T = 3.0, 3.0
FLOOR_R = 16.5         # Bodenscheibe ueber dem Zapfen
Z_FLOOR1 = Z_J1 + FLOOR_T
ZC = Z_J1 + L_HOLE_PITCH / 2 + 4.1             # LiDAR-Achse: untere Lasche schliesst mit Traegerboden ab
Y_WALL = -L_OPT                                # Anlageflaeche des LiDAR-Sockels
HX_AB = L_BODY / 2 - L_HOLE_EDGE               # x der Zweilochlinie
HX_C = HX_AB - L_HOLE_3                        # x des dritten Lochs
LIDAR_HOLES = [(HX_AB, ZC + L_HOLE_PITCH / 2), (HX_AB, ZC - L_HOLE_PITCH / 2), (HX_C, ZC)]

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=MIN)


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def ycyl(r, x, z, y0, y1):
    """Zylinder mit Achse entlang Y."""
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def rcyl(r, ang, z, r0, r1):
    """Zylinder radial nach aussen unter Winkel ang."""
    return Rot(0, 0, ang) * (Pos((r0 + r1) / 2, 0, z) * Rot(0, 90, 0) * Cylinder(r, r1 - r0))


def polar(r, ang):
    a = math.radians(ang)
    return r * math.cos(a), r * math.sin(a)


# ---------------------------------------------------------------- Teil 1: Boden
def make_boden():
    p = cyl(R_OUT, 0, BASE_T)
    # vier Eckpfosten als Motorfuehrung (Seiten offen fuer Kabel/Stecker)
    half = NEMA / 2 + MOTOR_CLEAR
    ring = box(-half - 3, half + 3, -half - 3, half + 3, BASE_T, BASE_T + 10) - \
        box(-half, half, -half, half, BASE_T - 1, BASE_T + 11)
    gap = half - 9
    ring = ring - box(-gap, gap, -40, 40, BASE_T, BASE_T + 11) - box(-40, 40, -gap, gap, BASE_T, BASE_T + 11)
    p = p + ring
    # Stativmutter 1/4"-20 (SW 7/16" = 11,11) von oben, liegt unter dem Motor
    p = p - Pos(0, 0, BASE_T - 5.8) * extrude(RegularPolygon(5.7, 6, major_radius=False), 6.0)
    p = p - cyl(3.3, -1, BASE_T + 1)
    for a in (0, 90, 180, 270):
        x, y = polar(SCREW_R, a)
        p = p - cyl(D_M3_THRU / 2, -1, BASE_T + 1, x, y) - cyl(3.1, -1, 3.4, x, y)
    return p


# ---------------------------------------------------------------- Teil 2: Glocke (Stator mit Lagersitz)
def make_glocke():
    p = cyl(R_OUT, BASE_T, Z_TOP) - cyl(R_IN, BASE_T - 1, Z_ROOF0)
    for a in (0, 90, 180, 270):
        p = p + Rot(0, 0, a) * box(RIB_R0, R_IN + 0.5, -RIB_W / 2, RIB_W / 2, BASE_T, Z_ROOF0)
    # Lagersitz von oben, darunter 45-Grad-Schulter (druckt kopfueber ohne Stuetze)
    p = p - cyl(SEAT_D / 2, Z_BRG0, Z_TOP + 1)
    p = p - Pos(0, 0, Z_ROOF0) * Cone(SEAT_D / 2 - 2.0, SEAT_D / 2, 2.0, align=MIN)
    p = p - cyl(SEAT_D / 2 - 2.0, Z_ROOF0 - 1, Z_ROOF0 + 0.01)
    # 45-Grad-Fase aussen oben: verkleinert den Schattenkegel des LiDAR nach unten
    p = p - (cyl(R_OUT + 1, Z_TOP - TOP_CH, Z_TOP + 1) - Pos(0, 0, Z_TOP - TOP_CH) * Cone(R_OUT, R_OUT - TOP_CH - 1, TOP_CH + 1, align=MIN))
    for a in (0, 90, 180, 270):
        x, y = polar(SCREW_R, a)
        p = p - cyl(D_M3_FORM / 2, BASE_T - 1, BASE_T + 12, x, y)          # Boden -> Glocke
        p = p - rcyl(D_M3_FORM / 2, a, Z_MOT1 - 5, RIB_R0 - 1, R_OUT + 1)   # Motorklemmung
    # vier Fenster (Gewicht, Kuehlung, Kabel), beide Enden spitz -> ohne Stuetze druckbar
    w, z0, z1 = 26.0, BASE_T + 6, Z_HUB0
    pts = [(-w / 2, z0 + w / 2), (0, z0), (w / 2, z0 + w / 2), (w / 2, z1 - w / 2), (0, z1), (-w / 2, z1 - w / 2)]
    win = extrude(Plane.XZ * Polygon(*pts, align=None), amount=R_OUT + 5, both=True)
    for a in (45, 135):
        p = p - Rot(0, 0, a) * win
    return p


# ---------------------------------------------------------------- Teil 3: Nabe (auf dem Abtrieb)
def make_nabe():
    c = SQ_POCKET / 2 + 4.0
    col = box(-c, c, -c, c, Z_HUB0, Z_COLLAR1)
    col = fillet(col.edges().filter_by(Axis.Z), 4.0)
    p = col + cyl(J_D / 2, Z_COLLAR1, Z_J1)
    p = p + Pos(0, 0, Z_COLLAR1) * Cone(J_D / 2 + 4.0, J_D / 2, 3.0, align=MIN)
    p = p - box(-SQ_POCKET / 2, SQ_POCKET / 2, -SQ_POCKET / 2, SQ_POCKET / 2, Z_HUB0 - 1, Z_OUT)
    p = p - cyl(3.4, Z_OUT - 1, Z_SEAT + 1) - cyl(6.75, Z_SEAT, Z_J1 + 1)
    for a in (90, 210, 330):
        x, y = polar(HUB_BOLT_R, a)
        p = p - cyl(D_M3_FORM / 2, Z_J1 - 12, Z_J1 + 1, x, y)
    return p


# ---------------------------------------------------------------- Teil 4: LiDAR-Traeger
X0, X1 = HX_C - 4.3, L_BODY / 2 + 4.0      # Breite: drittes Loch ... Sockelkante + Rippe
Y_BACK = Y_WALL - WALL_T
Z_WALL1 = ZC + L_BODY / 2 + 2.0


def make_traeger():
    p = box(X0, X1, Y_BACK, -12.0, Z_J1, Z_FLOOR1) + cyl(FLOOR_R, Z_J1, Z_FLOOR1)
    p = p + box(X0, X1, Y_BACK, Y_WALL, Z_J1, Z_WALL1)
    p = p + box(HX_AB - 6.5, HX_AB + 6.5, Y_BACK, Y_WALL, Z_WALL1 - 1, ZC + L_HOLE_PITCH / 2 + 4.1)
    for xa, xb in ((X0, X0 + 2.5), (X1 - 2.5, X1)):
        tri = Plane.YZ * Polygon((Y_WALL, Z_FLOOR1), (Y_WALL + 14, Z_FLOOR1), (Y_WALL, Z_FLOOR1 + 18), align=None)
        p = p + Pos(xa, 0, 0) * extrude(tri, amount=xb - xa)
    # Fenster hinter dem Sockel, 45-Grad-Dach
    wx0, wx1, wz0, wz1 = HX_C + 6.2, HX_AB - 5.8, ZC - 16, ZC + 6
    pts = [(wx0, wz0), (wx1, wz0), (wx1, wz1), ((wx0 + wx1) / 2, wz1 + (wx1 - wx0) / 2), (wx0, wz1)]
    p = p - Pos(0, Y_WALL + 1, 0) * extrude(Plane.XZ * Polygon(*pts, align=None), amount=WALL_T + 2)
    for x, z in LIDAR_HOLES:
        p = p - ycyl(D_M2_THRU / 2, x, z, Y_BACK - 1, Y_WALL + 1)
    for z in (Z_FLOOR1 + 5, Z_FLOOR1 + 11):                       # Kabelbinder (Zugentlastung)
        p = p - ycyl(1.7, X1 - 6.0, z, Y_BACK - 1, Y_WALL + 1)
    p = p - cyl(6.75, Z_J1 - 1, Z_FLOOR1 + 1)                     # Zugang zur 1/4"-Schraube
    # Aussparung im Boden fuer die untere Befestigungslasche des STL27L
    p = p - box(HX_AB - 4.4, HX_AB + 4.4, Y_WALL, Y_WALL + 3.0, Z_J1 - 1, Z_FLOOR1 + 1)
    for a in (90, 210, 330):
        x, y = polar(HUB_BOLT_R, a)
        p = p - cyl(D_M3_THRU / 2, Z_J1 - 1, Z_FLOOR1 + 1, x, y)
    return p


# ---------------------------------------------------------------- Passproben
def make_passproben():
    c = SQ_POCKET / 2 + 4.0
    vier = box(-c, c, -c, c, 0, SQ_DEPTH + 1.2) - box(-SQ_POCKET / 2, SQ_POCKET / 2, -SQ_POCKET / 2, SQ_POCKET / 2, 1.2, 9)
    vier = vier - cyl(8, -1, 2)
    sitz = cyl(SEAT_D / 2 + 3, 0, BRG_W) - cyl(SEAT_D / 2, -1, BRG_W + 1)
    zapfen = cyl(J_D / 2, 0, 10) - cyl(J_D / 2 - 3, -1, 11)
    return {"passprobe_vierkant": vier, "passprobe_lagersitz": sitz, "passprobe_zapfen": zapfen}


# ---------------------------------------------------------------- Bestandsteile als Pruefkoerper
def make_bestand():
    motor = box(-NEMA / 2, NEMA / 2, -NEMA / 2, NEMA / 2, Z_MOT0, Z_MOT1)
    z_round0 = Z_OUT - 15.6
    getr = box(-GEAR_SQ / 2, GEAR_SQ / 2, -GEAR_SQ / 2, GEAR_SQ / 2, Z_MOT1, z_round0) & cyl(GEAR_R, Z_MOT1, z_round0)
    abtr = cyl(16.17, z_round0, Z_OUT - SQ_T) + box(-SQ_OUT / 2, SQ_OUT / 2, -SQ_OUT / 2, SQ_OUT / 2, Z_OUT - SQ_T, Z_OUT)
    lager = cyl(BRG_OD / 2, Z_BRG0, Z_TOP) - cyl(BRG_ID / 2, Z_BRG0 - 1, Z_TOP + 1)
    y_base1 = Y_WALL + (L_H - L_TUR_H)
    lid = box(-L_BODY / 2, L_BODY / 2, Y_WALL, y_base1, ZC - L_BODY / 2, ZC + L_BODY / 2)
    lid = lid + ycyl(L_TUR_D / 2, 0, ZC, y_base1, Y_WALL + L_H)
    for x, z in LIDAR_HOLES:
        lid = lid + ycyl(4.0, x, z, Y_WALL, Y_WALL + 2.5)
    return {"motor": motor, "getriebe": getr, "abtrieb": abtr, "lager": lager, "stl27l": lid}


TEILE = {"1_boden": make_boden, "2_glocke": make_glocke, "3_nabe": make_nabe, "4_lidartraeger": make_traeger}

if __name__ == "__main__":
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    parts = {k: f() for k, f in TEILE.items()}
    parts.update(make_passproben())
    for name, p in parts.items():
        export_step(p, str(out / f"{name}.step"))
        export_stl(p, str(out / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.2)
    geo = {
        "z_abtrieb": Z_OUT, "z_glocke_oben": Z_TOP, "z_traeger_unten": Z_J1, "z_lidar_achse": ZC,
        "lidar_optik_y": 0.0, "lidar_achse_x": 0.0,
        "schatten_unten_grad": round(math.degrees(max(math.atan2(R_OUT, ZC - (Z_TOP - TOP_CH)), math.atan2(R_OUT - TOP_CH, ZC - Z_TOP), math.atan2(FLOOR_R, ZC - Z_FLOOR1))), 1),
        "hinweis": "Scan-Ebene y=0 enthaelt die Drehachse: Sollwert Achsversatz 0 / 0, Feinwert aus Kalibrierung",
    }
    (out / "geometrie.json").write_text(json.dumps(geo, indent=2))
    print(json.dumps(geo, indent=1))
