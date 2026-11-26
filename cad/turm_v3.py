"""PiLiDAR v3 - Turmgehaeuse nach Originalbauweise (hochkant), neu als CAD-Code.

python cad/turm_v3.py  ->  cad/out/*.step, *.stl, geometrie.json   (build123d)

Koordinaten: Z = Drehachse, z = 0 Auflage auf der Stativplatte. Vorne = -y (Kamera), hinten = +y (LiDAR, Powerbank).
Wie im Original dreht das ganze Gehaeuse; unten steht nur der Stativfuss. Neu gegenueber dem Original:
  1. Der Turm laeuft auf zwei Kugellagern 6810 um den Fuss statt auf dem Getriebeabtrieb.
  2. Powerbank 91 x 63 x 22 wie im Original, auf einer Auflage an der Rueckwand mit Band statt in einer Schale.
  3. STL27L direkt auf der Rueckwand, Lochbild nach Datenblatt (46,8 / 31,92).

Herkunft der Masse: Gehaeuse 92 x 200 und Anordnung aus PiLiDAR-Hardware/FDM; Getriebe aus Printables 782336 und
gearbox_deckel / gearbox_holder / gearbox kameraplatte; STL27L aus dem Datenblatt Abschnitt 5.1.
"""
from build123d import *
import json, math, pathlib

# ---------------------------------------------------------------- Bestand
NEMA = 42.3
GEAR_H, LID_T, LID_SQ, LID_R = 20.0, 3.0, 52.3, 32.2
MOTOR_L = 23.0
KP_DROP = 16.0             # Deckel-Aussenflaeche -> Aussenflaeche Abtriebs-Vierkant
SQ_OUT, SQ_T, KP_ROUND_R = 32.3, 5.0, 16.17
AKKU_L, AKKU_H, AKKU_T = 91.0, 63.0, 22.0             # Powerbank wie im Original (Intenso XS10000)
L_BODY, L_H, L_TUR_D, L_TUR_H = 38.6, 34.8, 35.29, 12.6
L_OPT = L_H - 7.8
L_HOLE_PITCH, L_HOLE_3, L_HOLE_EDGE = 46.8, 31.92, 10.3

# ---------------------------------------------------------------- Passungen
SQ_POCKET = 32.5
J_D, SEAT_D = 49.95, 65.1
BRG_ID, BRG_OD, BRG_W = 50.0, 65.0, 7.0                   # 2 x 6810-2RS
D_M3_FORM, D_M3_THRU, D_M25_FORM, D_M2_FORM = 2.9, 3.4, 2.2, 1.8

# ---------------------------------------------------------------- Fuss / Lagerung
FL_R, FL_T = 33.0, 4.0
SP = 12.0                  # Abstand der Lager (groesser = steiferer Turm, aber hoeher)
Z_B1 = FL_T + 1.0
Z_B2 = Z_B1 + BRG_W + SP
Z_DB = Z_B2 + BRG_W + 1.0
DECK_T = 5.0
Z_D = Z_DB + DECK_T        # Oberseite Teller = Auflage Getriebedeckel = Unterkante Turm
Z_KP = Z_D - KP_DROP
Z_PF = Z_KP - 1.0
H_F = Z_DB - 1.5
SL_RO = 37.0

# ---------------------------------------------------------------- Turm (Original 92 x 200)
W2 = 46.0                  # halbe Breite
T = 2.4                    # Wandstaerke
H_TURM = 200.0
Z_T = Z_D + H_TURM
Y_F = -37.1                # Frontflaeche (Original: 37,1 mm vor der Achse)
Y_M = -26.0                # Trennfuge Front / Mitte
Y_P1 = 16.5                # Rueckseite der Mittelplatte = Anlage LiDAR und Powerbank
Y_P0 = Y_P1 - T
LIDAR_D = Y_P1 + L_OPT     # Abstand Scan-Ebene - Drehachse (43,5; Original 37,5)
Z_MOT1 = Z_D + LID_T + GEAR_H + MOTOR_L
Z_CUT = Z_MOT1 + 5.0       # Ausschnitt der Platte um Motor und Getriebe
Z_SHELF = Z_CUT            # Auflage Powerbank
Z_AKKU1 = Z_SHELF + AKKU_H
ZC = Z_AKKU1 + 2.0 + (L_HOLE_3 - (L_BODY / 2 - L_HOLE_EDGE)) + 4.1    # LiDAR-Achse, einzelne Lasche unten
Z_AB = ZC + L_BODY / 2 - L_HOLE_EDGE
LIDAR_HOLES = [(-L_HOLE_PITCH / 2, Z_AB), (L_HOLE_PITCH / 2, Z_AB), (0.0, Z_AB - L_HOLE_3)]
CAM_Z = Z_T - T - 4.6 - 19.0          # Kameraachse
CAM_SENS = 30.0            # Sensor -> Frontflaeche (ANNAHME, haengt vom Objektiv ab)
Y_CAM = Y_F + CAM_SENS + 1.6          # Rueckseite der Kameraplatine = Ende der Kamerasaeulen
PI_STAND = 5.0
PI_X0 = -(W2 - T) + 0.2    # Platinenkante USB-Seite (-x)
PI_HOLES_X = (PI_X0 + 85 - 61.5, PI_X0 + 85 - 3.5)
PI_HOLES_Z = (Z_CUT + 5.0, Z_CUT + 5.0 + 49.0)
FEET = [(sx * 40.5, y) for sx in (-1, 1) for y in (-16.0, 4.0)]      # Teller -> Mitte, von unten
Z_BLK = Z_D + 13.0
FRONT_SCREWS = [(sx * 33.0, Z_D + 7.0) for sx in (-1, 1)] + [(sx * 39.8, Z_T - T - 3.8) for sx in (-1, 1)]
AKKU_SCREWS = [(sx * 35.5, z) for sx in (-1, 1) for z in (Z_D + 29.0, Z_D + 41.0)]
IMU_HOLES = [(24.4, Z_AKKU1 + 18.0), (24.4 + 15.24, Z_AKKU1 + 18.0)]   # GY-521

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=MIN)


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def ycyl(r, x, z, y0, y1):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def yz_prism(pts, x0, x1):
    return Pos(min(x0, x1), 0, 0) * extrude(Plane.YZ * Polygon(*pts, align=None), amount=abs(x1 - x0))


# ---------------------------------------------------------------- 1 Stativfuss (steht fest)
def make_fuss():
    p = cyl(FL_R, 0, FL_T) + cyl(26.5, FL_T, Z_B1) + cyl(J_D / 2, Z_B1, H_F)
    pk = box(-SQ_POCKET / 2, SQ_POCKET / 2, -SQ_POCKET / 2, SQ_POCKET / 2, Z_PF, H_F + 1)
    p = p - fillet(pk.edges().filter_by(Axis.Z), 2.5)
    p = p - Pos(0, 0, 2.0) * extrude(RegularPolygon(5.7, 6, major_radius=False), Z_PF - 2.0 + 0.1)   # Stativmutter 1/4"
    p = p - cyl(3.3, -1, 3)
    return p


# ---------------------------------------------------------------- 2 Teller mit Lagerhuelse (Druck: Oberseite aufs Bett)
def make_teller():
    p = box(-W2, W2, Y_F, 30.0, Z_DB, Z_D) + cyl(SL_RO, FL_T + 0.6, Z_D) - cyl(SEAT_D / 2, FL_T, Z_DB)
    p = p - cyl(23.0, Z_DB - 1, Z_D + 1)
    for x, y in FEET:
        p = p - cyl(D_M3_THRU / 2, Z_DB - 1, Z_D + 1, x, y)
    return p


# ---------------------------------------------------------------- 3 Mitte (Druck liegend, Plattenrueckseite aufs Bett)
def make_mitte():
    p = box(-W2, W2, Y_P0, Y_P1, Z_D, Z_T) - box(-27, 27, Y_P0 - 1, Y_P1 + 1, Z_D - 1, Z_CUT)
    p = p + box(-W2, -W2 + T, Y_M, Y_P1, Z_D, Z_T) + box(W2 - T, W2, Y_M, Y_P1, Z_D, Z_T)
    p = p + box(-W2, W2, Y_M, Y_P1, Z_T - T, Z_T)
    for sx in (-1, 1):
        xa, xb = sorted((sx * 26.6, sx * (W2 - T + 0.1)))
        p = p + box(xa, xb, Y_M, Y_P0 + 0.1, Z_D, Z_BLK)                                  # Fussklotz
        xa, xb = sorted((sx * 22.0, sx * 26.7))
        p = p + box(xa, xb, Y_M, Y_P0 + 0.1, Z_D + LID_T, Z_BLK)                          # Leiste klemmt den Deckelrand
        xa, xb = sorted((sx * 36.0, sx * (W2 - T + 0.1)))
        p = p + box(xa, xb, Y_M, Y_P0 + 0.1, Z_T - T - 7.6, Z_T - T + 0.1)                # Eckklotz oben
    yf = Y_P0 + 0.1
    for x in PI_HOLES_X:                                                                   # Raspberry Pi 4/5
        for z in PI_HOLES_Z:
            p = p + ycyl(3.2, x, z, Y_P0 - PI_STAND, yf)
    for x, z in LIDAR_HOLES:
        p = p + ycyl(3.5, x, z, Y_P0 - 6.0, yf)
    for sx in (-1, 1):
        for sz in (-1, 1):
            p = p + ycyl(3.5, sx * 15, CAM_Z + sz * 15, Y_CAM, yf)                         # Kamerasaeulen
    for x, z in AKKU_SCREWS:
        p = p + ycyl(4.0, x, z, Y_P0 - 5.0, yf)
    for x, z in IMU_HOLES:
        p = p + ycyl(3.5, x, z, Y_P0 - 4.0, yf)
    # --- Bohrungen
    for x, y in FEET:
        p = p - cyl(D_M3_FORM / 2, Z_D - 1, Z_D + 12, x, y)
    for x, z in FRONT_SCREWS:
        p = p - ycyl(D_M3_FORM / 2, x, z, Y_M - 1, Y_M + 12)
    for x in PI_HOLES_X:
        for z in PI_HOLES_Z:
            p = p - ycyl(D_M25_FORM / 2, x, z, Y_P0 - PI_STAND - 1, Y_P0 + 1.5)
    for x, z in LIDAR_HOLES:
        p = p - ycyl(D_M2_FORM / 2, x, z, Y_P0 - 7, Y_P1 + 1)
    for sx in (-1, 1):
        for sz in (-1, 1):
            p = p - ycyl(D_M25_FORM / 2, sx * 15, CAM_Z + sz * 15, Y_CAM - 1, Y_CAM + 10)
    for x, z in AKKU_SCREWS:
        p = p - ycyl(D_M3_FORM / 2, x, z, Y_P0 - 4.3, Y_P1 + 1)
    for x, z in IMU_HOLES:
        p = p - ycyl(D_M3_FORM / 2, x, z, Y_P0 - 5, Y_P0 + 1.6)
    # --- Ausschnitte
    p = p - box(-W2 - 1, -W2 + T + 0.1, Y_M - 1, 10.0, Z_CUT, Z_CUT + 59)                 # Pi: USB / LAN, nach vorne offen
    p = p - box(W2 - T - 0.1, W2 + 1, 6.0, 13.0, PI_HOLES_Z[0] + 14, PI_HOLES_Z[0] + 35)  # Pi: SD-Karte
    for sx in (-1, 1):                                                                     # Baender Powerbank (oben; unten durch den Motorausschnitt)
        xa, xb = sorted((sx * 16.0, sx * 34.0))
        p = p - box(xa, xb, Y_P0 - 1, Y_P1 + 1, Z_AKKU1 + 1.0, Z_AKKU1 + 4.0)
    p = p - box(-6, 6, Y_P0 - 1, Y_P1 + 1, ZC + L_BODY / 2 + 2.4, ZC + L_BODY / 2 + 8.4)  # LiDAR-Kabel
    for x0 in (-24, 0, 24):
        p = p - box(x0 - 8, x0 + 8, -18, 6, Z_T - T - 1, Z_T + 1)                         # Abluft
    return p


# ---------------------------------------------------------------- 4 Frontdeckel (Druck: Aussenflaeche aufs Bett)
def make_front():
    yi = Y_F + T
    p = box(-W2, W2, Y_F, yi, Z_D, Z_T)
    p = p + box(-W2, -W2 + T, yi, Y_M, Z_D, Z_T) + box(W2 - T, W2, yi, Y_M, Z_D, Z_T)
    p = p + box(-W2, W2, yi, Y_M, Z_T - T, Z_T) + box(-W2, W2, yi, Y_M - 0.4, Z_D, Z_D + T)
    for x, z in FRONT_SCREWS:
        p = p + ycyl(3.5, x, z, yi - 0.1, Y_M)
    for x, z in FRONT_SCREWS:
        p = p - ycyl(D_M3_THRU / 2, x, z, Y_F - 1, Y_M + 1)
    p = p - ycyl(18.0, 0, CAM_Z, Y_F - 1, yi + 1)                                          # Objektiv
    for x in range(-30, 31, 10):                                                           # Lochraster Elektronik, Lueftung
        for z in range(int(Z_D) + 43, int(CAM_Z) - 25, 10):
            p = p - ycyl(1.6, x, z, Y_F - 1, yi + 1)
    return p


# ---------------------------------------------------------------- 5 Akkutraeger (Druck: Anlageflaeche aufs Bett)
def make_akkutraeger():
    p = box(-42, 42, Y_P1, Y_P1 + AKKU_T, Z_SHELF - 3.0, Z_SHELF)
    for sx in (-1, 1):
        xa, xb = sorted((sx * 29.0, sx * 42.0))
        p = p + box(xa, xb, Y_P1, Y_P1 + 3.0, Z_D + 23.0, Z_SHELF)
        xa, xb = sorted((sx * 29.0, sx * 32.0))
        p = p + yz_prism([(Y_P1 + 2.9, Z_SHELF - 2.9), (Y_P1 + 15.0, Z_SHELF - 2.9), (Y_P1 + 2.9, Z_SHELF - 15.0)], xa, xb)
    for x, z in AKKU_SCREWS:
        p = p - ycyl(D_M3_THRU / 2, x, z, Y_P1 - 1, Y_P1 + 4)
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
    geh = box(-NEMA / 2, NEMA / 2, -NEMA / 2, NEMA / 2, z + LID_T, Z_MOT1)
    sq = box(-SQ_OUT / 2, SQ_OUT / 2, -SQ_OUT / 2, SQ_OUT / 2, Z_KP, Z_KP + SQ_T)
    kp = cyl(KP_ROUND_R, Z_KP + SQ_T, z - 0.4) + fillet(sq.edges().filter_by(Axis.Z), 3.0)
    lager = (cyl(BRG_OD / 2, Z_B1, Z_B1 + BRG_W) - cyl(BRG_ID / 2, Z_B1 - 1, Z_B1 + BRG_W + 1)) + \
            (cyl(BRG_OD / 2, Z_B2, Z_B2 + BRG_W) - cyl(BRG_ID / 2, Z_B2 - 1, Z_B2 + BRG_W + 1))
    yb = Y_P1 + (L_H - L_TUR_H)
    lidar = box(-L_BODY / 2, L_BODY / 2, Y_P1, yb, ZC - L_BODY / 2, ZC + L_BODY / 2) + ycyl(L_TUR_D / 2, 0, ZC, yb, Y_P1 + L_H)
    for x, zz in LIDAR_HOLES:
        lidar = lidar + ycyl(4.0, x, zz, Y_P1, Y_P1 + 2.5)
    yp = Y_P0 - PI_STAND
    pz0 = PI_HOLES_Z[0] - 3.5
    pi = box(PI_X0, PI_X0 + 85, yp - 1.6, yp, pz0, pz0 + 56)
    pi = pi + box(PI_X0 - 2.5, PI_X0 + 19, yp - 1.6 - 16, yp - 1.6, pz0 + 1, pz0 + 55)     # USB / LAN
    akku = box(-AKKU_L / 2, AKKU_L / 2, Y_P1, Y_P1 + AKKU_T, Z_SHELF, Z_AKKU1)
    cam = box(-19, 19, Y_CAM - 1.6, Y_CAM, CAM_Z - 19, CAM_Z + 19) + ycyl(14.0, 0, CAM_Z, Y_F - 5.0, Y_CAM - 1.6)
    return {"deckel": lid, "motor_getriebe": geh, "abtrieb": kp, "lager": lager, "stl27l": lidar, "pi": pi, "akku": akku, "kamera": cam}


TEILE = {"1_stativfuss": make_fuss, "2_teller": make_teller, "3_mitte": make_mitte, "4_front": make_front,
         "5_akkutraeger": make_akkutraeger}

if __name__ == "__main__":
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    parts = {k: f() for k, f in TEILE.items()}
    parts.update(make_ringe()); parts.update(make_passproben())
    for name, p in parts.items():
        export_step(p, str(out / f"{name}.step"))
        export_stl(p, str(out / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.2)
    geo = {
        "lidar_abstand_scanebene_mm": round(LIDAR_D, 2), "lidar_achse_z_mm": round(ZC, 2), "kamera_achse_z_mm": round(CAM_Z, 2),
        "lidar_unter_kamera_mm": round(CAM_Z - ZC, 2), "turm_unterkante_z_mm": Z_D, "gesamthoehe_mm": Z_T,
        "min_scanwinkel_grad": {f"{r} m": round(180 + 2 * math.degrees(math.asin(LIDAR_D / (r * 1000))), 1) for r in (0.3, 0.5, 1.0)},
    }
    (out / "geometrie.json").write_text(json.dumps(geo, indent=2))
    print(json.dumps(geo, indent=1))
