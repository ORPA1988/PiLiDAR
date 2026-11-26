"""PiLiDAR v3 - drehende Plattform: unten nur der Stativfuss, alles andere dreht mit.

python cad/plattform_v3.py  ->  cad/out/*.step, *.stl, geometrie.json   (build123d)

Koordinaten: Z = Drehachse, z = 0 Auflage auf der Stativplatte. Vorne = -y (LiDAR), +x = Akkuseite, -x = Pi-Seite.
Der STL27L sitzt aussen an der Frontwand; seine Scan-Ebene liegt bei y = -LIDAR_D parallel zur Drehachse und
sieht an Plattform, Fuss und Stativ vorbei. Es gibt kein Kabel ueber die Drehstelle.

Herkunft der Masse
  Getriebe   Printables 782336 (gearbox.obj 42,3 x 42,3 x 20) + PiLiDAR-Hardware (gearbox_deckel 52,3 x 3,
             gearbox_holder, gearbox kameraplatte): Abtriebsflaeche 16,0 mm unter der Deckel-Aussenflaeche.
  STL27L     Datenblatt Abschnitt 5.1.   Pi 4/5: Bohrbild 58 x 49 (M2.5).   HQ-Kamera: Bohrbild 30 x 30 (M2.5).
  Akku       ANNAHME: Powerbank bis 150 x 75 x 22 mm (AKKU_*).
"""
from build123d import *
import json, math, pathlib

# ---------------------------------------------------------------- Bestand
NEMA = 42.3
GEAR_H, LID_T, LID_SQ, LID_R = 20.0, 3.0, 52.3, 32.2      # Getriebegehaeuse, PiLiDAR-Deckel
MOTOR_L = 23.0
KP_DROP = 16.0             # Deckel-Aussenflaeche -> Aussenflaeche Abtriebs-Vierkant
SQ_OUT, SQ_T, KP_ROUND_R = 32.3, 5.0, 16.17

# ---------------------------------------------------------------- Passungen
SQ_POCKET = 32.5           # Vierkant-Aufnahme im Fuss
J_D = 49.95                # Zapfen fuer Innenring 50
SEAT_D = 65.1              # Sitz fuer Aussenring 65
BRG_ID, BRG_OD, BRG_W = 50.0, 65.0, 7.0                   # 2 x 6810-2RS
D_M3_FORM, D_M3_THRU, D_M25, D_M2_FORM = 2.9, 3.4, 2.8, 1.8

# ---------------------------------------------------------------- Fuss / Lagerung
FL_R, FL_T = 33.0, 4.0     # Fussflansch
SP = 12.0                  # Abstand der beiden Lager
Z_B1 = FL_T + 1.0          # Unterkante unteres Lager
Z_B2 = Z_B1 + BRG_W + SP   # Unterkante oberes Lager
Z_DB = Z_B2 + BRG_W + 1.0  # Unterseite Teller (1 mm Distanzring)
DECK_T = 5.0
Z_D = Z_DB + DECK_T        # Oberseite Teller = Auflage Getriebedeckel
Z_KP = Z_D - KP_DROP       # Aussenflaeche Abtrieb
Z_PF = Z_KP - 1.0          # Boden der Vierkant-Aufnahme (Abtrieb schwebt 1 mm)
H_F = Z_DB - 1.5           # Oberkante Fuss
SL_RO = 37.0               # Aussenradius Lagerhuelse

# ---------------------------------------------------------------- Aufbau
BX_I, WALL = 26.6, 4.0     # halbe Innenbreite, Wandstaerke
BX_O = BX_I + WALL
Y_FI, Y_BI = -38.0, 37.0   # Innenflaechen Front / Rueckwand
Y_FO, Y_BO = Y_FI - WALL, Y_BI + WALL
Z_W = Z_D + LID_T + GEAR_H + MOTOR_L + 5.0                # Wandoberkante (5 mm Luft ueber dem Motor)
BLK = 8.0                  # Eckkloetze
BLOCKS = [(sx * (BX_I - BLK / 2), y) for sx in (-1, 1) for y in (Y_FI + BLK / 2, Y_BI - BLK / 2)]
BAR_SCREWS = [(sx * 13.0, sy * 31.0) for sx in (-1, 1) for sy in (-1, 1)]

# STL27L aussen an der Frontwand, Stecker oben
L_BODY, L_H, L_TUR_D, L_TUR_H = 38.1, 34.8, 35.29, 12.6
L_OPT = L_H - 7.8
L_HOLE_PITCH, L_HOLE_3, L_HOLE_EDGE = 46.8, 31.92, 10.3
ZC = Z_D + (L_HOLE_3 - (L_BODY / 2 - L_HOLE_EDGE)) + 4.1 + 0.6   # unterste Lasche knapp ueber dem Teller
Z_AB = ZC + L_BODY / 2 - L_HOLE_EDGE
LIDAR_HOLES = [(-L_HOLE_PITCH / 2, Z_AB), (L_HOLE_PITCH / 2, Z_AB), (0.0, Z_AB - L_HOLE_3)]
LIDAR_D = -Y_FO + L_OPT    # Abstand Scan-Ebene - Drehachse

# Raspberry Pi 4/5 aussen an der -x-Wand, USB nach hinten
PI_Y = (-33.0, 25.0)
PI_Z = (Z_D + 6.5, Z_D + 6.5 + 49.0)
# Akku aussen an der +x-Wand, hochkant auf der langen Kante
AKKU_L, AKKU_H, AKKU_T = 140.0, 70.0, 22.0
SH_X1, SH_Y1 = BX_O + AKKU_T + 2.4, 60.0
STRAP_Y = (-22.0, 22.0)
# HQ-Kamera auf der Bruecke, Eintrittspupille auf der Drehachse
BR_T = 3.0
CAM_PUPIL = 15.0           # Sensor -> Eintrittspupille des Fischauges (ANNAHME)
CAM_Y0 = CAM_PUPIL + 6.0   # Vorderseite Kameraplatte (Platine 1,6 + Abstandshalter 3 + Sensorhoehe)
CAM_Z = Z_W + BR_T + 4.0 + 19.0
IMU_HOLES = [(-14.0 - 7.62, -28.0), (-14.0 + 7.62, -28.0)]   # GY-521, Lochabstand 15,24

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=MIN)


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def ycyl(r, x, z, y0, y1):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def xcyl(r, y, z, x0, x1):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def xz_prism(pts, y0, y1):
    """Prisma aus einem Polygon in der X-Z-Ebene, von y0 bis y1."""
    return Pos(0, max(y0, y1), 0) * extrude(Plane.XZ * Polygon(*pts, align=None), amount=abs(y1 - y0))


def yz_prism(pts, x0, x1):
    return Pos(min(x0, x1), 0, 0) * extrude(Plane.YZ * Polygon(*pts, align=None), amount=abs(x1 - x0))


def roof_window(a0, a1, z0, z1):
    """Fenster mit 45-Grad-Dach (ohne Stuetze druckbar): Punkte in (a, z)."""
    return [(a0, z0), (a1, z0), (a1, z1), ((a0 + a1) / 2, z1 + (a1 - a0) / 2), (a0, z1)]


# ---------------------------------------------------------------- 1 Stativfuss (steht fest)
def make_fuss():
    p = cyl(FL_R, 0, FL_T) + cyl(26.5, FL_T, Z_B1) + cyl(J_D / 2, Z_B1, H_F)
    pk = box(-SQ_POCKET / 2, SQ_POCKET / 2, -SQ_POCKET / 2, SQ_POCKET / 2, Z_PF, H_F + 1)
    p = p - fillet(pk.edges().filter_by(Axis.Z), 2.5)
    # Schacht fuer die Stativmutter 1/4"-20 (SW 11,11): Mutter faellt von oben bis auf 2 mm Boden
    p = p - Pos(0, 0, 2.0) * extrude(RegularPolygon(5.7, 6, major_radius=False), Z_PF - 2.0 + 0.1)
    p = p - cyl(3.3, -1, 3)
    return p


# ---------------------------------------------------------------- 2 Teller mit Lagerhuelse (Druck: Oberseite aufs Bett)
def make_teller():
    p = box(-BX_O, BX_O, Y_FO, Y_BO, Z_DB, Z_D) + box(BX_O - 0.1, SH_X1, Y_FO, SH_Y1, Z_DB, Z_D)
    p = p + cyl(SL_RO, FL_T + 0.6, Z_D) - cyl(SEAT_D / 2, FL_T, Z_DB)
    p = p - cyl(23.0, Z_DB - 1, Z_D + 1)                                   # Abtrieb (Vierkant passt durch)
    for x, y in BLOCKS:
        p = p - cyl(D_M3_THRU / 2, Z_DB - 1, Z_D + 1, x, y)               # Schrauben von unten in den Aufbau
    for x, y in BAR_SCREWS:
        p = p - cyl(D_M3_FORM / 2, Z_D - 4.2, Z_D + 1, x, y)              # Klemmleisten, Sackloch
    return p


# ---------------------------------------------------------------- 3 Aufbau (vier Waende)
def make_aufbau():
    p = box(-BX_O, BX_O, Y_FO, Y_FI, Z_D, Z_W) + box(-BX_O, BX_O, Y_BI, Y_BO, Z_D, Z_W)
    p = p + box(-BX_O, -BX_I, Y_FO, Y_BO, Z_D, Z_W) + box(BX_I, BX_O, Y_FO, Y_BO, Z_D, Z_W)
    for x, y in BLOCKS:
        p = p + box(x - BLK / 2, x + BLK / 2, y - BLK / 2, y + BLK / 2, Z_D, Z_W)
    for y in PI_Y:                                                         # Ohren fuer die oberen Pi-Loecher
        p = p + box(-BX_O, -BX_I, y - 5, y + 5, Z_W - 1, PI_Z[1] + 4.5)
    # Akkuauflage: zwei Arme und eine Anschlagleiste (liegen auf dem Teller)
    for y in STRAP_Y:
        p = p + box(BX_O - 0.1, SH_X1, y - 4, y + 4, Z_D, Z_D + 2)
    p = p + box(SH_X1 - 2.0, SH_X1, STRAP_Y[0] - 6, STRAP_Y[1] + 6, Z_D, Z_D + 6)
    # --- Bohrungen
    for x, y in BLOCKS:
        p = p - cyl(D_M3_FORM / 2, Z_D - 1, Z_D + 12, x, y) - cyl(D_M3_FORM / 2, Z_W - 12, Z_W + 1, x, y)
    for x, z in LIDAR_HOLES:                                               # STL27L: M2 direkt in den Kunststoff
        p = p - ycyl(D_M2_FORM / 2, x, z, Y_FO - 1, Y_FI + (7 if abs(x) > 1 else 1))
    # Frontwand: Fenster hinter dem Sockel, nach oben offen fuer das LiDAR-Kabel
    key = [(-13, ZC - 17), (13, ZC - 17), (13, ZC + 5), (5, ZC + 13), (5, Z_W + 1), (-5, Z_W + 1), (-5, ZC + 13), (-13, ZC + 5)]
    p = p - xz_prism(key, Y_FO - 1, Y_FI + 1)
    # Rueckwand: Lochraster 10 mm fuer ESP32, Treiber, Aufwaertswandler
    for x in (-15, -5, 5, 15):
        for z in range(int(Z_D) + 13, int(Z_W) - 5, 10):
            p = p - ycyl(1.6, x, z, Y_BI - 1, Y_BO + 1)
    # -x-Wand: Pi-Bohrbild 58 x 49 und Fenster
    for y in PI_Y:
        for z in PI_Z:
            p = p - xcyl(D_M25 / 2, y, z, -BX_O - 1, -BX_I + 1)
    p = p - yz_prism(roof_window(-22, 14, Z_D + 13, Z_D + 29), -BX_O - 1, -BX_I + 1)
    # +x-Wand: Schlitze fuer das Akkuband und Fenster
    for y in STRAP_Y:
        p = p - box(BX_I - 1, BX_O + 1, y - 1.5, y + 1.5, Z_D + 15, Z_D + 31)
    p = p - yz_prism(roof_window(-12, 12, Z_D + 13, Z_D + 25), BX_I - 1, BX_O + 1)
    return p


# ---------------------------------------------------------------- 4 Bruecke mit Kameraplatte
def make_bruecke():
    z0, z1 = Z_W, Z_W + BR_T
    p = box(-BX_I + 0.3, BX_I - 0.3, Y_FO, Y_BO, z0, z1)
    p = p - box(-16, 16, -20, CAM_Y0 - 2, z0 - 1, z1 + 1)                  # Abluft Motor, Flachbandkabel
    p = p - box(-5.5, 5.5, Y_FO - 1, Y_FI + 1, z0 - 1, z1 + 1)            # LiDAR-Kabel
    p = p + box(-22, 22, CAM_Y0, CAM_Y0 + 3, z1 - 0.1, CAM_Z + 22)
    for sx in (-1, 1):
        tri = [(CAM_Y0 + 3, z1), (Y_BO - 3, z1), (CAM_Y0 + 3, z1 + 26)]
        p = p + yz_prism(tri, sx * 19 - 1.5, sx * 19 + 1.5)
    for sx in (-1, 1):
        for sz in (-1, 1):
            p = p - ycyl(D_M25 / 2, sx * 15, CAM_Z + sz * 15, CAM_Y0 - 1, CAM_Y0 + 4)
    p = p - ycyl(9.0, 0, CAM_Z, CAM_Y0 - 1, CAM_Y0 + 4)
    for x, y in BLOCKS:
        p = p - cyl(D_M3_THRU / 2, z0 - 1, z1 + 1, x, y)
    for x, y in IMU_HOLES:
        p = p - cyl(D_M3_FORM / 2, z0 - 1, z1 + 1, x, y)
    for x in (6, 16):
        for y in (-32, -24):
            p = p - cyl(1.6, z0 - 1, z1 + 1, x, y)                         # Raster fuer das LiDAR-USB-Board
    return p


# ---------------------------------------------------------------- 5 Klemmleiste (2 Stueck, vorne und hinten)
def make_leiste(front=True):
    s = -1 if front else 1
    ya, yb = sorted((s * 22.0, s * 36.0))
    p = box(-BX_I + BLK + 1, BX_I - BLK - 1, ya, yb, Z_D, Z_D + 7)
    yr = sorted((s * 20.0, s * (LID_SQ / 2 + 0.4)))
    p = p - box(-30, 30, yr[0], yr[1], Z_D - 1, Z_D + LID_T + 0.1)         # Stufe ueber dem Deckelrand
    for x, y in BAR_SCREWS:
        if (y < 0) == front:
            p = p - cyl(D_M3_THRU / 2, Z_D - 1, Z_D + 8, x, y) - cyl(3.1, Z_D + 4, Z_D + 8, x, y)
    return p


def make_ringe():
    return {
        "6_ring_oben": cyl(32.4, 0, 1.0) - cyl(30.3, -1, 2),
        "7_distanz_aussen": cyl(32.4, 0, SP) - cyl(30.5, -1, SP + 1),
        "8_distanz_innen": cyl(27.0, 0, SP) - cyl(25.15, -1, SP + 1),
    }


def make_passproben():
    fuss = make_fuss() & cyl(40, H_F - 9, H_F + 2)
    sitz = cyl(SEAT_D / 2 + 3, 0, BRG_W) - cyl(SEAT_D / 2, -1, BRG_W + 1)
    return {"passprobe_fuss_oben": Pos(0, 0, -(H_F - 9)) * fuss, "passprobe_lagersitz": sitz}


# ---------------------------------------------------------------- Bestand als Pruefkoerper
def make_bestand():
    z = Z_D
    lid = box(-LID_SQ / 2, LID_SQ / 2, -LID_SQ / 2, LID_SQ / 2, z, z + LID_T) & cyl(LID_R, z, z + LID_T)
    geh = box(-NEMA / 2, NEMA / 2, -NEMA / 2, NEMA / 2, z + LID_T, z + LID_T + GEAR_H + MOTOR_L)
    sq = box(-SQ_OUT / 2, SQ_OUT / 2, -SQ_OUT / 2, SQ_OUT / 2, Z_KP, Z_KP + SQ_T)
    kp = cyl(KP_ROUND_R, Z_KP + SQ_T, z - 0.4) + fillet(sq.edges().filter_by(Axis.Z), 3.0)
    lager = (cyl(BRG_OD / 2, Z_B1, Z_B1 + BRG_W) - cyl(BRG_ID / 2, Z_B1 - 1, Z_B1 + BRG_W + 1)) + \
            (cyl(BRG_OD / 2, Z_B2, Z_B2 + BRG_W) - cyl(BRG_ID / 2, Z_B2 - 1, Z_B2 + BRG_W + 1))
    yb = Y_FO - (L_H - L_TUR_H)
    lidar = box(-L_BODY / 2, L_BODY / 2, yb, Y_FO, ZC - L_BODY / 2, ZC + L_BODY / 2) + ycyl(L_TUR_D / 2, 0, ZC, Y_FO - L_H, yb)
    for x, zz in LIDAR_HOLES:
        lidar = lidar + ycyl(4.0, x, zz, Y_FO - 2.5, Y_FO)
    xp = -BX_O - 6.0                                                        # 6 mm Abstandshalter
    pi = box(xp - 1.6, xp, PI_Y[0] - 3.5, PI_Y[0] - 3.5 + 85, PI_Z[0] - 3.5, PI_Z[0] - 3.5 + 56)
    pi = pi + box(xp - 18, xp - 1.6, PI_Y[0] + 61.5 - 21.5, PI_Y[0] - 3.5 + 87, PI_Z[0], PI_Z[1])   # USB/LAN-Block
    akku = box(BX_O, BX_O + AKKU_T, Y_FO, Y_FO + AKKU_L, Z_D + 2, Z_D + 2 + AKKU_H)
    yp = CAM_Y0 - 3.0
    cam = box(-19, 19, yp - 1.6, yp, CAM_Z - 19, CAM_Z + 19) + ycyl(15.0, 0, CAM_Z, yp - 1.6 - 32, yp - 1.6)
    return {"deckel": lid, "motor_getriebe": geh, "abtrieb": kp, "lager": lager, "stl27l": lidar, "pi": pi, "akku": akku, "kamera": cam}


TEILE = {"1_stativfuss": make_fuss, "2_teller": make_teller, "3_aufbau": make_aufbau, "4_bruecke": make_bruecke,
         "5_klemmleiste": make_leiste}

if __name__ == "__main__":
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    parts = {k: f() for k, f in TEILE.items()}
    parts.update(make_ringe()); parts.update(make_passproben())
    for name, p in parts.items():
        export_step(p, str(out / f"{name}.step"))
        export_stl(p, str(out / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.2)
    geo = {
        "lidar_abstand_scanebene_mm": round(LIDAR_D, 2), "lidar_achse_z_mm": round(ZC, 2),
        "teller_oben_z_mm": Z_D, "wand_oben_z_mm": Z_W, "kamera_achse_z_mm": CAM_Z,
        "gesamthoehe_mm": CAM_Z + 22,
        "min_scanwinkel_grad": {f"{r} m": round(180 + 2 * math.degrees(math.asin(LIDAR_D / (r * 1000))), 1) for r in (0.3, 0.5, 1.0)},
    }
    (out / "geometrie.json").write_text(json.dumps(geo, indent=2))
    print(json.dumps(geo, indent=1))
