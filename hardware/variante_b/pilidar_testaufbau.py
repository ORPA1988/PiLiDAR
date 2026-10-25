"""PiLiDAR Variante B – Testaufbau (parametrisch, CadQuery)

Teile
  1  motorgehaeuse    Grundplatte mit Stativmutter 1/4", Motorschacht NEMA 17,
                      zwei Klemmschrauben, Fenster für den Motorstecker
  2  hall_halter      Säule mit Langloch, auf der Grundplatte verschraubt
  3  hall_schlitten   trägt den Hall-Sensor (TO-92 liegend), höhenverstellbar
  4  abtriebsadapter  Scheibe zwischen Getriebeabtrieb und Rotorplatte
                      (nur dieses Teil anpassen, wenn der Abtrieb anders ist)
  5  rotorplatte      Testaufsatz: STL27L-Halter, Halter für HQ-Kamera mit
                      UVC-Adapter, Magnettasche, Wuchtbohrungen, Kabelbinderschlitze
  6  lidarplatte      Stufe 1a: nur STL27L, Nabe direkt auf dem Getriebeabtrieb
                      (Schnittstelle 1:1 vom Original-PiLiDAR-Stativadapter)

Exportiert werden derzeit nur 1 und 6 (Kamera und Hall folgen später).

Aufruf:  python pilidar_testaufbau.py      -> STL + STEP je Teil, assembly.step
Alle Maße in mm. Druck: PETG, 4 Wände, 30 % Füllung, ohne Stützen.
"""

from __future__ import annotations

import math
from pathlib import Path

import cadquery as cq

# =====================================================================
# Parameter
# =====================================================================
# Motor 17HE08-1004S
MOTOR_W = 42.3            # Kantenlänge NEMA 17
MOTOR_L = 23.0            # Gehäuselänge
FIT = 0.3                 # Spiel je Seite im Motorschacht

# Getriebe: Höhe von der Motorstirnfläche bis zur Abtriebsfläche
# >>> vor dem Druck am eigenen Getriebe messen <<<
GEAR_STACK_H = 30.0

# Getriebeabtrieb (abgeleitet vom Original-PiLiDAR-"tripod adapter":
# 4 Bohrungen auf ±10,7 mm im Kreuz, Mitte frei)
OUT_BOLT_R = 10.7
OUT_BOLT_N = 4
OUT_BOLT_D = 3.3          # M3 Durchgang

# Nabe der LiDAR-Platte = Schnittstelle des Original-Stativadapters
# (gemessen an "tripod adapter.obj" aus PiLiDAR-Hardware):
#   Sechskanttasche SW 11,17, ca. 6 mm tief, Ecken zu den Bohrungen 0°/180°
#   4 × Ø3,0 auf ±10,7 mm im Kreuz, Senkung Ø6 von der Gegenseite
#   Mitte Ø6,5 durchgehend
HUB_HEX_AF = 11.3         # Original 11,17 + 0,13 Spiel
HUB_HEX_DEPTH = 6.2
HUB_CENTER_D = 6.5
HUB_BOLT_D = 3.2
HUB_CBORE_D = 6.2
HUB_CBORE_T = 3.0
HUB_D = 28.0              # Nabenauflage unten (passt in die Ø30,5-Aussparung)
HUB_DROP = 1.5            # Nabe steht unten vor -> Platte schleift nicht am Getriebedeckel
HUB_TOP_D = 30.0          # Verstärkung oben
HUB_TOP_H = 3.5
OUT_CENTER_D = 12.4       # Freistich für Abtriebsnabe/Wellenende

# Grundplatte
BASE_W = 72.0
BASE_T = 8.0
SLEEVE_WALL = 3.5
SLEEVE_H = 18.0           # Höhe Motorschacht über der Grundplatte

# Stativmutter 1/4"-20 (Schlüsselweite 7/16" = 11,11 mm, Dicke 5,56 mm)
TRIPOD_NUT_AF = 11.4
TRIPOD_NUT_T = 6.0
TRIPOD_BORE = 6.8

# M3 / M2.5
M3 = 3.4
M3_NUT_AF = 5.7
M3_NUT_T = 2.6
M25 = 2.7
M25_NUT_AF = 5.2
M25_NUT_T = 2.0

# Rotor
ROTOR_D = 106.0
ROTOR_T = 5.0
ADAPTER_D = 48.0
ADAPTER_T = 6.0
PLATE_BOLT_R = 20.0       # Adapter <-> Rotorplatte, 4 × M3 unter 45°

# Hall-Sensor / Magnet
HALL_R = 45.0             # Abstand Sensor/Magnet von der Drehachse
MAGNET_D = 6.0            # Scheibenmagnet 6 × 3
MAGNET_T = 3.0
HALL_GAP = 2.0            # Luftspalt Sensor – Rotorplatte
TO92_W, TO92_L, TO92_T = 5.4, 5.4, 2.2   # Tasche für TO-92 liegend

# STL27L (Datenblatt: 54,00 × 46,29 × 34,80), auf der Seite montiert
LIDAR_L, LIDAR_W, LIDAR_D = 54.0, 46.29, 34.8
LIDAR_FIT = 0.3
LIDAR_GRIP = 8.0          # Umfassungstiefe am Sockel (Fensterband bleibt frei)
LIDAR_BACK_X = 18.0       # Innenkante Rückwand von der Achse

# HQ-Kamera (Platine 38 × 38, Bohrbild 30 × 30, M2.5) mit UVC-Adapter
CAM_HOLE = 30.0
CAM_BACK_X = 18.0         # Rückwand Kamerahalter, auf der Gegenseite
CAM_CENTER_Z = 25.0       # Bildmitte über Oberkante Rotorplatte

OUT = Path(__file__).resolve().parent / "out"

# =====================================================================
# Hilfen
# =====================================================================
def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cyl(d, h, x=0.0, y=0.0, z=0.0, axis="Z"):
    wp = {"Z": "XY", "X": "YZ", "Y": "XZ"}[axis]
    c = cq.Workplane(wp).circle(d / 2).extrude(h)
    if axis == "Z":
        return c.translate((x, y, z))
    if axis == "X":
        return c.translate((x, y, z))
    # Workplane XZ extrudiert in -Y
    return c.translate((x, y + h, z))


def hexprism(af, h, x=0.0, y=0.0, z=0.0, axis="Z"):
    d = af / math.cos(math.radians(30))
    wp = {"Z": "XY", "X": "YZ", "Y": "XZ"}[axis]
    p = cq.Workplane(wp).polygon(6, d).extrude(h)
    if axis == "Y":
        return p.translate((x, y + h, z))
    return p.translate((x, y, z))


def polar(r, n, start_deg):
    return [(r * math.cos(math.radians(start_deg + i * 360 / n)),
             r * math.sin(math.radians(start_deg + i * 360 / n))) for i in range(n)]


def gusset_xz(x_wall, x_end, y0, t, z0, h):
    """Dreieckiges Stützblech in der XZ-Ebene, Dicke t ab y0."""
    pts = [(x_wall, z0), (x_end, z0), (x_wall, z0 + h)]
    g = cq.Workplane("XZ").polyline(pts).close().extrude(-t)
    return g.translate((0, y0, 0))


# =====================================================================
# Abgeleitete Höhen (Zusammenbau, z = 0 Unterkante Grundplatte)
# =====================================================================
Z_MOTOR_TOP = BASE_T + MOTOR_L
Z_ADAPTER = Z_MOTOR_TOP + GEAR_STACK_H
Z_ROTOR = Z_ADAPTER + ADAPTER_T
SLEEVE_IN = MOTOR_W + 2 * FIT
SLEEVE_OUT = SLEEVE_IN + 2 * SLEEVE_WALL


# =====================================================================
# 1 Motorgehäuse
# =====================================================================
def motorgehaeuse():
    b = (cq.Workplane("XY").rect(BASE_W, BASE_W).extrude(BASE_T)
         .edges("|Z").fillet(6))
    so, si = SLEEVE_OUT / 2, SLEEVE_IN / 2
    sleeve = box(-so, so, -so, so, BASE_T, BASE_T + SLEEVE_H).cut(
        box(-si, si, -si, si, BASE_T, BASE_T + SLEEVE_H + 1))
    b = b.union(sleeve)
    # Klemmschrauben-Bosse auf -X
    zc = BASE_T + 10.0
    for y in (-10.0, 10.0):
        boss = box(-so - 7, -so + 0.5, y - 5, y + 5, BASE_T, zc + 6)
        b = b.union(boss)
    for y in (-10.0, 10.0):
        b = b.cut(cyl(M3, 20, -so - 8, y, zc, axis="X"))
        # Mutternschlitz von oben
        b = b.cut(box(-so - 4.3 - M3_NUT_T / 2, -so - 4.3 + M3_NUT_T / 2,
                      y - M3_NUT_AF / 2, y + M3_NUT_AF / 2, zc - M3_NUT_AF / 2, zc + 7))
    # Fenster Motorstecker (+Y)
    b = b.cut(box(-10, 10, si - 1, so + 1, BASE_T, BASE_T + 12))
    # Stativmutter 1/4"
    b = b.cut(hexprism(TRIPOD_NUT_AF, TRIPOD_NUT_T + 0.01, 0, 0, BASE_T - TRIPOD_NUT_T))
    b = b.cut(cyl(TRIPOD_BORE, BASE_T, 0, 0, -0.01))
    # Befestigung Hall-Halter (+X) mit Muttern von unten
    for y in (-8.0, 8.0):
        b = b.cut(cyl(M3, BASE_T + 1, 30.0, y, -0.5))
        b = b.cut(hexprism(M3_NUT_AF, M3_NUT_T, 30.0, y, -0.01))
    # Kabelbinderschlitze (-Y Kante)
    for x in (-14.0, 14.0):
        b = b.cut(box(x - 2, x + 2, -BASE_W / 2 + 4, -BASE_W / 2 + 7, -1, BASE_T + 1))
    return b


# =====================================================================
# 2 Hall-Halter + 3 Schlitten
# =====================================================================
COL_X0 = 36.0                                   # Innenseite der Säule
COL_T = 5.0
COL_H = (Z_ROTOR - 4.0) - BASE_T                # Säule endet 4 mm unter der Rotorplatte
SLIDER_H = 14.0
SLIDER_W = 2 * (HALL_R - (COL_X0 + COL_T))      # Sensor mittig auf HALL_R


def hall_halter():
    foot = box(25.5, COL_X0 + COL_T, -12, 12, 0, 5)
    col = box(COL_X0, COL_X0 + COL_T, -8, 8, 0, COL_H)
    h = foot.union(col)
    # ein mittiges Stützblech (seitlich wären die Schraubenköpfe im Weg)
    h = h.union(gusset_xz(COL_X0, 25.5, -1.5, 3, 5, 18))
    for y in (-8.0, 8.0):
        h = h.cut(cyl(M3, 10, 30.0, y, -1))
    # Langloch für den Schlitten
    slot = (cq.Workplane("YZ").center(0, (12 + COL_H - 5) / 2)
            .slot2D(COL_H - 5 - 12, M3, angle=90).extrude(COL_T + 2)
            .translate((COL_X0 - 1, 0, 0)))
    return h.cut(slot)


def hall_schlitten():
    x0 = COL_X0 + COL_T
    s = box(x0, x0 + SLIDER_W, -8, 8, 0, SLIDER_H)
    s = s.cut(cyl(M3, SLIDER_W + 2, x0 - 1, 0, 5.0, axis="X"))
    # Schraubenkopf versenkt von außen
    s = s.cut(cyl(6.2, 3.0, x0 + SLIDER_W - 3.0, 0, 5.0, axis="X"))
    # TO-92 liegend, Markierungsseite nach oben, Beinchen Richtung -Y
    s = s.cut(box(HALL_R - TO92_L / 2, HALL_R + TO92_L / 2, -TO92_W / 2, TO92_W / 2,
                  SLIDER_H - TO92_T, SLIDER_H + 1))
    s = s.cut(box(HALL_R - 2.25, HALL_R + 2.25, -9, -TO92_W / 2 + 0.1,
                  SLIDER_H - 1.5, SLIDER_H + 1))
    return s


# =====================================================================
# 4 Abtriebsadapter
# =====================================================================
def abtriebsadapter():
    a = cq.Workplane("XY").circle(ADAPTER_D / 2).extrude(ADAPTER_T)
    a = a.cut(cyl(OUT_CENTER_D, ADAPTER_T + 2, 0, 0, -1))
    for x, y in polar(OUT_BOLT_R, OUT_BOLT_N, 0):
        a = a.cut(cyl(OUT_BOLT_D, ADAPTER_T + 2, x, y, -1))
        a = a.cut(cyl(6.2, 3.2, x, y, ADAPTER_T - 3.2))
    for x, y in polar(PLATE_BOLT_R, 4, 45):
        a = a.cut(cyl(M3, ADAPTER_T + 2, x, y, -1))
        a = a.cut(hexprism(M3_NUT_AF, M3_NUT_T, x, y, -0.01))
    return a


# =====================================================================
# 5 Rotorplatte mit Haltern
# =====================================================================
def rotorplatte():
    t = ROTOR_T
    p = cq.Workplane("XY").circle(ROTOR_D / 2).extrude(t)

    # --- STL27L-Halter (+X): Rückwand + Sockelrahmen ------------------
    iy = (LIDAR_L + 2 * LIDAR_FIT) / 2           # Innenmaß halb (Y)
    iz0 = t + 3.0                                # Unterkante LiDAR
    iz1 = iz0 + LIDAR_W + 2 * LIDAR_FIT
    zt = iz1 + 3.0
    bx0, bx1 = LIDAR_BACK_X, LIDAR_BACK_X + 4.0
    back = box(bx0, bx1, -(iy + 5), iy + 5, t, zt)
    # U-Rahmen, oben offen (kein Brückendruck); LiDAR wird von oben eingesetzt
    frame = box(bx1, bx1 + LIDAR_GRIP, -(iy + 3), iy + 3, t, zt).cut(
        box(bx1 - 1, bx1 + LIDAR_GRIP + 1, -iy, iy, iz0, zt + 1))
    p = p.union(back).union(frame)
    p = p.cut(box(bx0 - 1, bx1 + 1, -10, 10, iz0 + 12, iz0 + 32))   # Kabeldurchgang
    # Klemmschrauben M3 seitlich gegen den Sockel (Bosse mit Mutternschlitz)
    xm, zm = bx1 + LIDAR_GRIP / 2, iz0 + (LIDAR_W / 2)
    for s in (1, -1):
        y0, y1 = sorted((s * (iy + 3), s * (iy + 9)))
        p = p.union(box(bx1, bx1 + LIDAR_GRIP, y0, y1, zm - 8, zm + 8))
        p = p.cut(cyl(M3, 12, xm, s * (iy - 1) if s > 0 else s * (iy + 11), zm, axis="Y"))
        yn = s * (iy + 6)
        p = p.cut(box(xm - M3_NUT_AF / 2, xm + M3_NUT_AF / 2, yn - M3_NUT_T / 2, yn + M3_NUT_T / 2,
                      zm - M3_NUT_AF / 2, zm + 9))
    for y0 in (24.5, -27.5):
        p = p.union(gusset_xz(bx0, bx0 - 8, y0, 3, t, 20))

    # --- Kamerahalter (-X) ---------------------------------------------
    cz = t + CAM_CENTER_Z
    cx0, cx1 = -CAM_BACK_X - 4.0, -CAM_BACK_X
    cam = box(cx0, cx1, -23, 23, t, cz + 25)
    p = p.union(cam)
    for dy in (-CAM_HOLE / 2, CAM_HOLE / 2):
        for dz in (-CAM_HOLE / 2, CAM_HOLE / 2):
            p = p.cut(cyl(M25, 6, cx0 - 1, dy, cz + dz, axis="X"))
            p = p.cut(hexprism(M25_NUT_AF, M25_NUT_T + 0.01, cx1 - M25_NUT_T, dy, cz + dz, axis="X"))
    p = p.cut(box(cx0 - 1, cx1 + 1, -9, 9, cz - 9, cz + 9))        # Kabelfenster
    for y0 in (20.0, -23.0):
        p = p.union(gusset_xz(cx1, cx1 + 8, y0, 3, t, 20))

    # --- Bohrungen in der Platte ----------------------------------------
    p = p.cut(cyl(OUT_CENTER_D, t + 2, 0, 0, -1))
    for x, y in polar(PLATE_BOLT_R, 4, 45):
        p = p.cut(cyl(M3, t + 2, x, y, -1))
        p = p.cut(cyl(6.2, 2.5, x, y, t - 2.5))
    # Magnettasche unten bei (HALL_R, 0)
    p = p.cut(cyl(MAGNET_D + 0.3, MAGNET_T + 0.2, HALL_R, 0, -0.01))
    # Wuchtbohrungen (Muttern/Scheiben als Gegengewicht)
    for x, y in polar(36.0, 8, 22.5):
        p = p.cut(cyl(M3, t + 2, x, y, -1))
    # Kabelbinderschlitze
    for yc in (-42.0, 42.0):
        for xc in (-3.5, 3.5):
            p = p.cut(box(xc - 1.25, xc + 1.25, yc - 2, yc + 2, -1, t + 1))
    return p


# =====================================================================
# Nabe der LiDAR-Platte – zwei Ausführungen
#   "vierkant"  (Standard): passt auf das bestehende Abtriebsteil 32,3 × 32,3 mm
#               (Original-PiLiDAR "gearbox kameraplatte", mit eingegossener
#               1/4"-Stativmutter). Drehmoment über die Vierkant-Tasche,
#               Klemmung mit einer 1/4"-20-Schraube durch die Mitte.
#   "sechskant": Schnittstelle des Original-Stativadapters (siehe oben).
# =====================================================================
HUB_STYLE = "vierkant"
SQ_OUT = 32.0             # Kantenlänge Abtriebsteil (am Bauteil gemessen: 32 mm)
SQ_FIT = 0.25             # Spiel je Seite
SQ_DEPTH = 4.0            # Eingriffstiefe der Tasche
SQ_COLLAR = 4.0           # Wandstärke des Kragens um die Tasche
SCREW_D = 6.8             # Durchgang 1/4" (6,35 mm)
SCREW_HEAD_D = 11.0       # Senkung für Zylinderkopf 1/4" (Kopf ca. 9,5 mm)
SCREW_HEAD_T = 4.0


def nabe(p, t):
    """Schnittstelle zum Abtrieb in die Platte p (Dicke t, Unterseite z = 0) einbringen."""
    p = p.union(cq.Workplane("XY").circle(HUB_TOP_D / 2).extrude(HUB_TOP_H).translate((0, 0, t)))
    z_top = t + HUB_TOP_H
    if HUB_STYLE == "vierkant":
        a = SQ_OUT + 2 * SQ_FIT
        c = a + 2 * SQ_COLLAR
        collar = (cq.Workplane("XY").rect(c, c).extrude(SQ_DEPTH).edges("|Z").fillet(3)
                  .translate((0, 0, -SQ_DEPTH)))
        p = p.union(collar)
        p = p.cut(cq.Workplane("XY").rect(a, a).extrude(SQ_DEPTH + 0.01)
                  .edges("|Z").fillet(0.8).translate((0, 0, -SQ_DEPTH - 0.01)))
        p = p.cut(cyl(SCREW_D, z_top + 2, 0, 0, -1))
        p = p.cut(cyl(SCREW_HEAD_D, SCREW_HEAD_T + 0.01, 0, 0, z_top - SCREW_HEAD_T))
        return p
    # sechskant
    p = p.union(cq.Workplane("XY").circle(HUB_D / 2).extrude(HUB_DROP).translate((0, 0, -HUB_DROP)))
    z_bot = -HUB_DROP
    hexd = HUB_HEX_AF / math.cos(math.radians(30))
    p = p.cut(cq.Workplane("XY").polygon(6, hexd).extrude(HUB_HEX_DEPTH).translate((0, 0, z_bot - 0.01)))
    p = p.cut(cyl(HUB_CENTER_D, z_top - z_bot + 2, 0, 0, z_bot - 1))
    for x, y in polar(OUT_BOLT_R, 4, 0):
        p = p.cut(cyl(HUB_BOLT_D, z_top - z_bot + 2, x, y, z_bot - 1))
        p = p.cut(cyl(HUB_CBORE_D, HUB_CBORE_T + 0.01, x, y, z_top - HUB_CBORE_T))
    return p


# =====================================================================
# 0 Nabentest: nur die Nabe (kurzer Druck) zum Probieren am Abtrieb
# =====================================================================
def nabentest():
    d = SQ_OUT + 2 * SQ_FIT + 2 * SQ_COLLAR + 6 if HUB_STYLE == "vierkant" else 34.0
    disc = cq.Workplane("XY").circle(d / 2).extrude(3.0)
    return nabe(disc, 3.0)


# =====================================================================
# 6 LiDAR-Platte (Stufe 1a, direkt auf den Abtrieb)
# =====================================================================
def lidarplatte():
    t = ROTOR_T
    p = cq.Workplane("XY").circle(ROTOR_D / 2).extrude(t)
    p = nabe(p, t)

    # STL27L-Halter wie in der Rotorplatte
    iy = (LIDAR_L + 2 * LIDAR_FIT) / 2
    iz0 = t + 3.0
    iz1 = iz0 + LIDAR_W + 2 * LIDAR_FIT
    zt = iz1 + 3.0
    bx0, bx1 = LIDAR_BACK_X, LIDAR_BACK_X + 4.0
    p = p.union(box(bx0, bx1, -(iy + 5), iy + 5, t, zt))
    p = p.union(box(bx1, bx1 + LIDAR_GRIP, -(iy + 3), iy + 3, t, zt).cut(
        box(bx1 - 1, bx1 + LIDAR_GRIP + 1, -iy, iy, iz0, zt + 1)))
    p = p.cut(box(bx0 - 1, bx1 + 1, -10, 10, iz0 + 12, iz0 + 32))
    xm, zm = bx1 + LIDAR_GRIP / 2, iz0 + LIDAR_W / 2
    for s in (1, -1):
        y0, y1 = sorted((s * (iy + 3), s * (iy + 9)))
        p = p.union(box(bx1, bx1 + LIDAR_GRIP, y0, y1, zm - 8, zm + 8))
        p = p.cut(cyl(M3, 12, xm, s * (iy - 1) if s > 0 else s * (iy + 11), zm, axis="Y"))
        yn = s * (iy + 6)
        p = p.cut(box(xm - M3_NUT_AF / 2, xm + M3_NUT_AF / 2, yn - M3_NUT_T / 2, yn + M3_NUT_T / 2,
                      zm - M3_NUT_AF / 2, zm + 9))
    for y0 in (24.5, -27.5):
        p = p.union(gusset_xz(bx0, bx0 - 8, y0, 3, t, 20))

    # Gegengewicht auf der Gegenseite: 2 × M5 für Scheibenstapel
    for y in (-10.0, 10.0):
        p = p.cut(cyl(5.3, t + 2, -38.0, y, -1))
    # Wuchtbohrungen M3 (Feinabgleich)
    for x, y in polar(36.0, 8, 22.5):
        p = p.cut(cyl(M3, t + 2, x, y, -1))
    # vorbereitet für später: Magnettasche, 4 × M3 für den Kamerahalter
    p = p.cut(cyl(MAGNET_D + 0.3, MAGNET_T + 0.2, HALL_R, 0, -0.01))
    for x in (-22.0, -32.0):
        for y in (-12.0, 12.0):
            p = p.cut(cyl(M3, t + 2, x, y, -1))
    # Kabelbinderschlitze
    for yc in (-42.0, 42.0):
        for xc in (-3.5, 3.5):
            p = p.cut(box(xc - 1.25, xc + 1.25, yc - 2, yc + 2, -1, t + 1))
    return p


# =====================================================================
# 7 LiDAR-Halter kompakt: Vierkant-Nabe + Rückwand + U-Rahmen, sonst nichts
# =====================================================================
LH_TOP_T = 5.0            # Deckplatte über der Tasche
LH_BACK_X = 6.5           # Innenkante Rückwand von der Achse (Schraubenkopf 1/4" frei)


def lidarhalter():
    a = SQ_OUT + 2 * SQ_FIT
    c = a + 2 * SQ_COLLAR
    t = LH_TOP_T
    # Grundkörper: Kragen (unten) + Deckplatte, ein Quader mit Tasche
    p = (cq.Workplane("XY").rect(c, c).extrude(SQ_DEPTH + t).edges("|Z").fillet(3)
         .translate((0, 0, -SQ_DEPTH)))
    p = p.cut(cq.Workplane("XY").rect(a, a).extrude(SQ_DEPTH + 0.01)
              .edges("|Z").fillet(0.8).translate((0, 0, -SQ_DEPTH - 0.01)))
    p = p.cut(cyl(SCREW_D, t + 2, 0, 0, -1))

    iy = (LIDAR_L + 2 * LIDAR_FIT) / 2
    iz0 = t + 3.0
    iz1 = iz0 + LIDAR_W + 2 * LIDAR_FIT
    zt = iz1 + 3.0
    bx0, bx1 = LH_BACK_X, LH_BACK_X + 4.0
    p = p.union(box(bx0, bx1, -(iy + 3), iy + 3, 0, zt))                     # Rückwand
    frame = box(bx1, bx1 + LIDAR_GRIP, -(iy + 3), iy + 3, 0, zt).cut(
        box(bx1 - 1, bx1 + LIDAR_GRIP + 1, -iy, iy, iz0, zt + 1))             # U-Rahmen, oben offen
    p = p.union(frame)
    p = p.cut(box(bx0 - 1, bx1 + 1, -10, 10, iz0 + 12, iz0 + 32))            # Kabeldurchgang
    # Klemmschrauben seitlich
    xm, zm = bx1 + LIDAR_GRIP / 2, iz0 + LIDAR_W / 2
    for s in (1, -1):
        y0, y1 = sorted((s * (iy + 3), s * (iy + 9)))
        p = p.union(box(bx1, bx1 + LIDAR_GRIP, y0, y1, zm - 8, zm + 8))
        p = p.cut(cyl(M3, 12, xm, s * (iy - 1) if s > 0 else s * (iy + 11), zm, axis="Y"))
        yn = s * (iy + 6)
        p = p.cut(box(xm - M3_NUT_AF / 2, xm + M3_NUT_AF / 2, yn - M3_NUT_T / 2, yn + M3_NUT_T / 2,
                      zm - M3_NUT_AF / 2, zm + 9))
    # Stützrippen hinter der Rückwand (auf der Deckplatte, Schraubenkopf bleibt frei)
    for y0 in (12.0, -15.0):
        p = p.union(gusset_xz(bx0, -c / 2 + 1, y0, 3, t, 28))
    return p


# =====================================================================
# 8 LiDAR-Halter V2: echte STL27L-Maße (Datenblatt V0.3), LiDAR auf der Diagonale
# =====================================================================
# STL27L-Datenblatt, Kap. 5.1: Grundplatte 54,00 × 46,29, 3 Laschen mit Ø2,50 (durch),
# Senkung Ø4,46 × 2; Lochabstand 46,80 (zwei Löcher), 31,92 senkrecht zum dritten Loch;
# Gesamthöhe 34,80, Turm Ø35,29 × 12,60.
# Lage der Drehachse des LiDAR zu den Löchern aus der Datenblattzeichnung abgegriffen
# (± ca. 1 mm, wird ohnehin über die Offset-Kalibrierung bestimmt):
STL_HOLE_PAIR = 46.80     # Abstand der zwei Löcher
STL_HOLE_V = 31.92        # Abstand Lochpaar -> Einzelloch
STL_AXIS_FROM_PAIR = 8.5  # Drehachse liegt 8,5 mm vom Lochpaar Richtung Einzelloch
STL_EDGE_PAIR = 10.6      # Außenkante jenseits des Lochpaars
STL_H = 34.80             # Bauhöhe (Grundplatte bis Turmdeckel)
STL_HOLE_D = 2.5
NEMA_HALF_DIAG = 42.3 / 2 * math.sqrt(2)      # 29,9 mm

LIDAR_DIR_DEG = 0.0       # LiDAR gerade zur Vierkant-Aufnahme (parallel zu den Motorkanten)
LIDAR_OVERHANG = 20.0     # LiDAR ragt 20 mm über die Motorecke hinaus
STRUCT_R = 30.0           # Halter (ohne LiDAR) bleibt innerhalb 30 mm Radius auf den Diagonalen
LH2_CLEAR = 1.5           # Luft LiDAR-Unterkante – Deckplatte
LH2_BACK_T = 4.0
M2 = 2.4
M2_NUT_AF, M2_NUT_T = 4.2, 1.8


def _lidar_axis_geom():
    # Anlagefläche = Vorderkante der Bodenplatte (Rückwand steht bündig auf dem Kragen)
    r_base = (SQ_OUT + 2 * SQ_FIT + 2 * SQ_COLLAR) / 2
    z_ax = LH_TOP_T + LH2_CLEAR + STL_EDGE_PAIR + STL_AXIS_FROM_PAIR
    holes = [(-STL_HOLE_PAIR / 2, -STL_AXIS_FROM_PAIR), (STL_HOLE_PAIR / 2, -STL_AXIS_FROM_PAIR),
             (0.0, STL_HOLE_V - STL_AXIS_FROM_PAIR)]           # (tangential, vertikal) zur Achse
    return r_base, z_ax, holes


def lidarhalter_v2():
    a = SQ_OUT + 2 * SQ_FIT
    c = a + 2 * SQ_COLLAR
    t = LH_TOP_T
    body = (cq.Workplane("XY").rect(c, c).extrude(SQ_DEPTH + t).edges("|Z").fillet(3)
            .translate((0, 0, -SQ_DEPTH)))
    body = body.cut(cq.Workplane("XY").rect(a, a).extrude(SQ_DEPTH + 0.01)
                    .edges("|Z").fillet(0.8).translate((0, 0, -SQ_DEPTH - 0.01)))

    r_base, z_ax, holes = _lidar_axis_geom()
    u1 = r_base                     # Anlagefläche (außen)
    u0 = r_base - LH2_BACK_T        # Rückseite
    th = STL_HOLE_PAIR / 2 + 4.0    # halbe Breite der Rückwand
    z1 = z_ax + (STL_HOLE_V - STL_AXIS_FROM_PAIR) + 4.0
    # lokales System: x = radial entlang der Diagonale, y = tangential
    # Rückwand steht über die ganze Breite bis zum Druckbett (kein freier Überhang)
    loc = box(u0, u1, -th, th, -SQ_DEPTH, z1)
    # Gewichtsfenster mit 45°-Dach (druckt ohne Brücke), Löcher bleiben im Material
    wb, wt, ww = z_ax - 3, z_ax + 4, 14.0
    win = (cq.Workplane("YZ").polyline([(-ww, wb), (ww, wb), (ww, wt), (0, wt + ww), (-ww, wt)])
           .close().extrude(LH2_BACK_T + 2).translate((u0 - 1, 0, 0)))
    loc = loc.cut(win)
    for ty, tv in holes:
        loc = loc.cut(cyl(M2, LH2_BACK_T + 2, u0 - 1, ty, z_ax + tv, axis="X"))
        loc = loc.cut(hexprism(M2_NUT_AF, M2_NUT_T + 0.01, u0 - 0.01, ty, z_ax + tv, axis="X"))
    # Rippen nach innen/hinten, Ende innerhalb STRUCT_R (Mittelschraube bleibt frei)
    u_end = -math.sqrt(max(STRUCT_R ** 2 - 15.0 ** 2, 0)) + 2.0
    for y0 in (12.0, -15.0):
        loc = loc.union(gusset_xz(u0, u_end, y0, 3, t, z_ax - t))
    loc = loc.rotate((0, 0, 0), (0, 0, 1), LIDAR_DIR_DEG)
    p = body.union(loc)
    p = p.cut(cyl(SCREW_D, t + 2, 0, 0, -1))
    return p


def stl27l_dummy():
    """Vereinfachter STL27L in Einbaulage (für Prüfung und Vorschau)."""
    r_base, z_ax, holes = _lidar_axis_geom()
    lo_v = -(STL_EDGE_PAIR + STL_AXIS_FROM_PAIR)                 # Unterkante relativ zur Achse
    flange = box(r_base, r_base + 2.5, -27.0, 27.0, z_ax + lo_v, z_ax + lo_v + 46.29)
    bodyb = box(r_base, r_base + 22.2, -19.25, 19.25, z_ax - 19.25, z_ax + 19.25)
    turret = cyl(35.29, 12.6, r_base + 22.2, 0, z_ax, axis="X")
    d = flange.union(bodyb).union(turret)
    return d.rotate((0, 0, 0), (0, 0, 1), LIDAR_DIR_DEG)


# =====================================================================
# Export
# =====================================================================
PARTS = {
    "0_nabentest": nabentest,
    "1_motorgehaeuse": motorgehaeuse,
    "8_lidarhalter_v2": lidarhalter_v2,
}
# Vorgänger mit U-Rahmen: "7_lidarhalter": lidarhalter
# Alternative mit großer Platte: "6_lidarplatte": lidarplatte
# später: "2_hall_halter": hall_halter, "3_hall_schlitten": hall_schlitten,
#         "4_abtriebsadapter": abtriebsadapter, "5_rotorplatte": rotorplatte

# Lage im Zusammenbau (Druckteile werden auf z = 0 gelegt exportiert)
ASSEMBLY_Z = {
    "0_nabentest": Z_ADAPTER + HUB_DROP + 80,
    "1_motorgehaeuse": 0.0,
    "2_hall_halter": BASE_T,
    "3_hall_schlitten": Z_ROTOR - HALL_GAP - SLIDER_H,
    "4_abtriebsadapter": Z_ADAPTER,
    "5_rotorplatte": Z_ROTOR,
    "6_lidarplatte": Z_ADAPTER + 20,
    "7_lidarhalter": Z_ADAPTER + 20,
    "8_lidarhalter_v2": Z_ADAPTER + 20,
}

# Druckorientierung: Drehung um X (Grad) vor dem Ablegen
PRINT_ROT = {"4_abtriebsadapter": 0, "3_hall_schlitten": 0}


def main():
    OUT.mkdir(exist_ok=True)
    asm = cq.Assembly()
    for name, fn in PARTS.items():
        solid = fn()
        bb = solid.val().BoundingBox()
        printable = solid.translate((0, 0, -bb.zmin))
        cq.exporters.export(printable, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(printable, str(OUT / f"{name}.step"))
        asm.add(solid.translate((0, 0, ASSEMBLY_Z[name] - bb.zmin)), name=name)
        print(f"{name:22s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f} mm")
    asm.save(str(OUT / "zusammenbau.step"))
    print(f"Rotorplatte Unterkante z = {Z_ROTOR:.1f} mm, Säulenhöhe {COL_H:.1f} mm")


if __name__ == "__main__":
    main()
