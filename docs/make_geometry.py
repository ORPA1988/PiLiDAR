"""Technische Zeichnung (Ingenieurstandard) des Mess-Koordinatensystems und der
Offsets — mit rechtshändigem Koordinatensystem, Bemaßung (Maßhilfslinien,
Maßpfeile, Werte in mm), Mittellinien und Schriftfeld.

Zwei Projektionen:
  Ansicht A (Blick entlang +X): Y-Z-Ebene → zeigt MODEL_Y und MODEL_Z.
  Ansicht B (Blick entlang +Y): Scan-Ebene X-Z → zeigt angle_offset, α, r, z_angle.

Ausgabe: docs/images/geometrie_koordinaten.png/.svg
Aufruf: python docs/make_geometry.py
"""

from __future__ import annotations

import os
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch, Rectangle, Circle

OUT = os.path.join(os.path.dirname(__file__), "images")
os.makedirs(OUT, exist_ok=True)

INK = "#111111"
DIM = "#1f4e79"   # Bemaßung
AUX = "#9aa3ad"   # Hilfslinien
ACC = "#b00020"   # Hervorhebung (Sensor)

MODEL_Y = -37.5
MODEL_Z = -41.9
ANGLE_OFFSET = -1.05


def centerline(ax, x0, y0, x1, y1):
    ax.plot([x0, x1], [y0, y1], color=INK, lw=0.9,
            dashes=[8, 3, 1.5, 3], zorder=1)


def dim_linear(ax, p0, p1, text, off=0.0, vertical=False, side=1):
    """Maßkette mit Maßhilfslinien und beidseitigen Pfeilen."""
    if vertical:
        xd = (p0[0] + p1[0]) / 2 + off
        ax.plot([p0[0], xd + 2 * side], [p0[1], p0[1]], color=AUX, lw=0.7)
        ax.plot([p1[0], xd + 2 * side], [p1[1], p1[1]], color=AUX, lw=0.7)
        ax.annotate("", xy=(xd, p0[1]), xytext=(xd, p1[1]),
                    arrowprops=dict(arrowstyle="<->", color=DIM, lw=1.2))
        ax.text(xd + 1.5 * side, (p0[1] + p1[1]) / 2, text, color=DIM, fontsize=9,
                rotation=90, va="center", ha="left" if side > 0 else "right")
    else:
        yd = (p0[1] + p1[1]) / 2 + off
        ax.plot([p0[0], p0[0]], [p0[1], yd + 2 * side], color=AUX, lw=0.7)
        ax.plot([p1[0], p1[0]], [p1[1], yd + 2 * side], color=AUX, lw=0.7)
        ax.annotate("", xy=(p0[0], yd), xytext=(p1[0], yd),
                    arrowprops=dict(arrowstyle="<->", color=DIM, lw=1.2))
        ax.text((p0[0] + p1[0]) / 2, yd + 1.5 * side, text, color=DIM, fontsize=9,
                ha="center", va="bottom" if side > 0 else "top")


def axis_arrow(ax, x0, y0, x1, y1, label, color=INK):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8))
    ax.text(x1, y1, "  " + label, color=color, fontsize=11, fontweight="bold",
            va="center")


def into_page(ax, x, y, label):
    ax.add_patch(Circle((x, y), 1.4, fill=False, ec=INK, lw=1.4))
    ax.plot([x - 1, x + 1], [y - 1, y + 1], color=INK, lw=1.2)
    ax.plot([x - 1, x + 1], [y + 1, y - 1], color=INK, lw=1.2)
    ax.text(x + 2.2, y, label, fontsize=11, fontweight="bold", va="center")


def out_of_page(ax, x, y, label):
    ax.add_patch(Circle((x, y), 1.4, fill=False, ec=INK, lw=1.4))
    ax.add_patch(Circle((x, y), 0.35, fc=INK, ec=INK))
    ax.text(x + 2.2, y, label, fontsize=11, fontweight="bold", va="center")


def view_A(ax):
    ax.set_title("Ansicht A — Y-Z-Ebene (Blick entlang +X)", fontsize=11, fontweight="bold")
    ax.set_xlim(-70, 45); ax.set_ylim(-75, 45); ax.set_aspect("equal"); ax.axis("off")

    # Drehachse Z (Mittellinie)
    centerline(ax, 0, -70, 0, 38)
    ax.text(1.5, 36, "Drehachse (Z)", fontsize=8.5, color=INK)
    # Koordinatensystem im Ursprung O (auf der Drehachse = Datum)
    axis_arrow(ax, 0, 0, 0, 26, "Z")          # Z nach oben
    axis_arrow(ax, 0, 0, 26, 0, "Y")          # Y nach rechts (Spinachse)
    into_page(ax, 0, 0, "X")                   # X in die Ebene
    ax.text(2, -3, "O (Datum, auf der Drehachse)", fontsize=8, color=INK)

    # Sensor-Optikzentrum S
    sx, sy = MODEL_Y, MODEL_Z
    ax.add_patch(Rectangle((sx - 6, sy - 5), 12, 10, fc="#f2dede", ec=ACC, lw=1.5))
    ax.add_patch(Circle((sx, sy), 1.3, fc=ACC, ec=ACC))
    ax.text(sx, sy - 9, "S  (LiDAR-Optikzentrum)", fontsize=8.5, color=ACC, ha="center")

    # Bemaßung MODEL_Y (horizontal) und MODEL_Z (vertikal)
    dim_linear(ax, (0, sy), (sx, sy), "MODEL_Y = −37,5 mm", off=-14, side=-1)
    dim_linear(ax, (0, 0), (0, sy), "MODEL_Z = −41,9 mm", off=0, vertical=True, side=1)
    # Hilfslinien Sensor -> Achsen
    ax.plot([sx, sx], [sy, -56], color=AUX, lw=0.7)
    ax.plot([sx, 18], [sy, sy], color=AUX, lw=0.7)

    ax.text(-68, 41, "Maße in mm · rechtshändiges System", fontsize=8, color="#555")


def view_B(ax):
    ax.set_title("Ansicht B — Scan-Ebene X-Z (Blick entlang +Y)", fontsize=11, fontweight="bold")
    ax.set_xlim(-30, 60); ax.set_ylim(-30, 60); ax.set_aspect("equal"); ax.axis("off")

    # Koordinatensystem (X rechts, Z oben, Y aus der Ebene)
    axis_arrow(ax, 0, 0, 34, 0, "X")
    axis_arrow(ax, 0, 0, 0, 34, "Z")
    out_of_page(ax, -16, 28, "Y (Spinachse, aus Ebene)")
    ax.add_patch(Circle((0, 0), 1.2, fc=ACC, ec=ACC))
    ax.text(1.5, -3.5, "S (Sensor)", fontsize=8.5, color=ACC)

    # nominale Null (α=0 entlang +X, gestrichelt) und tatsächliche Null (um angle_offset gedreht)
    ax.plot([0, 40], [0, 0], color=AUX, lw=1.0, dashes=[5, 3])
    ax.text(40.5, 0, "α = 0 (nominal)", fontsize=8, color="#666", va="center")
    th = np.radians(ANGLE_OFFSET)
    ax.annotate("", xy=(40 * np.cos(th), 40 * np.sin(th)), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color=DIM, lw=1.4))
    ax.text(41 * np.cos(th), 41 * np.sin(th) - 3, "tatsächliche Null", fontsize=8, color=DIM)
    ax.add_patch(Arc((0, 0), 26, 26, theta1=min(0, ANGLE_OFFSET) - 0,
                     theta2=max(0, ANGLE_OFFSET) + 0, color=DIM, lw=1.4))
    ax.text(20, -6.5, "angle_offset = −1,05°\n(Drehung um Y, in der Ebene)",
            fontsize=8.5, color=DIM, ha="center")

    # Messstrahl bei α, Länge r, Punkt P
    a = np.radians(35)
    r = 50
    px, py = r * np.cos(a + th), r * np.sin(a + th)
    ax.annotate("", xy=(px, py), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color="#7c3aed", lw=1.6))
    ax.add_patch(Circle((px, py), 1.0, fc="#7c3aed"))
    ax.text(px + 1, py + 1, "P", fontsize=10, color="#7c3aed")
    ax.add_patch(Arc((0, 0), 34, 34, theta1=ANGLE_OFFSET, theta2=35 + ANGLE_OFFSET,
                     color="#7c3aed", lw=1.2))
    ax.text(20, 14, "α (Messwinkel)", fontsize=8.5, color="#7c3aed")
    ax.text(px / 2 - 6, py / 2 + 2, "r (Distanz)", fontsize=8.5, color="#7c3aed", rotation=35)

    # z_angle Revolve um Z (Pfeil oben um die Z-Achse)
    ax.add_patch(Arc((0, 50), 22, 10, theta1=200, theta2=-20, color="#16a34a", lw=1.6))
    ax.annotate("", xy=(11, 52), xytext=(9, 47),
                arrowprops=dict(arrowstyle="-|>", color="#16a34a", lw=1.6))
    ax.text(0, 56, "z_angle: Revolve der Ebene um Z", fontsize=8.5, color="#16a34a", ha="center")


def title_block(fig, title):
    ax = fig.add_axes([0.62, 0.02, 0.36, 0.16]); ax.axis("off")
    ax.add_patch(Rectangle((0, 0), 1, 1, transform=ax.transAxes, fill=False, ec=INK, lw=1.2))
    rows = [
        ("Titel", title),
        ("Projekt", "PiLiDAR 2.0"),
        ("Einheit", "mm   ·   Winkel in Grad"),
        ("System", "rechtshändig: X×Y=Z"),
        ("Maßstab", "ohne Maßstab (schematisch)"),
    ]
    for i, (k, v) in enumerate(rows):
        y = 1 - (i + 1) / (len(rows) + 0.5)
        ax.text(0.02, y, k + ":", fontsize=8, fontweight="bold", color=INK)
        ax.text(0.34, y, v, fontsize=8, color=INK)


def frame(fig):
    b = fig.add_axes([0, 0, 1, 1]); b.axis("off")
    b.add_patch(Rectangle((0.01, 0.01), 0.98, 0.98, transform=b.transAxes,
                fill=False, ec=INK, lw=1.5))


def _save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"), dpi=300,
                    facecolor="white", bbox_inches="tight")
    plt.close(fig)
    print("ok", name)


def build():
    fig = plt.figure(figsize=(15, 8))
    fig.patch.set_facecolor("white")
    fig.suptitle("PiLiDAR 2.0 — Technische Zeichnung: Koordinatensystem & Offsets",
                 fontsize=14, fontweight="bold", y=0.97)
    view_A(fig.add_axes([0.04, 0.20, 0.44, 0.70]))
    view_B(fig.add_axes([0.52, 0.20, 0.44, 0.70]))
    title_block(fig, "Mess-Koordinatensystem & Offsets")
    frame(fig)
    _save(fig, "geometrie_koordinaten")


# ---------------------------------------------------------------------
def montage_view(ax):
    ax.set_title("Baugruppe — Schnitt Y-Z-Ebene (Blick entlang +X)", fontsize=11, fontweight="bold")
    ax.set_xlim(-70, 70); ax.set_ylim(-15, 115); ax.set_aspect("equal"); ax.axis("off")

    # Drehachse (Mittellinie) durch die ganze Höhe
    centerline(ax, 0, -10, 0, 110)
    ax.text(2, 107, "Drehachse Z", fontsize=8.5, color=INK)

    # Sockel / Motor+Getriebe / Rotorplatte
    ax.add_patch(Rectangle((-45, -8), 90, 8, fc="#e9eef5", ec=INK, lw=1.2))     # Sockel
    ax.text(0, -12, "Sockel (fest)", fontsize=8, color=INK, ha="center")
    ax.add_patch(Rectangle((-16, 0), 32, 22, fc="#dfe6ef", ec=INK, lw=1.2))     # Motor+Getriebe
    ax.text(0, 11, "Motor + Getriebe", fontsize=7.5, color=INK, ha="center")
    ax.add_patch(Rectangle((-40, 22), 80, 6, fc="#cdd8e6", ec=INK, lw=1.2))     # Rotorplatte
    ax.text(-41, 25, "Rotorplatte (dreht um Z)", fontsize=7.5, color=INK, ha="right")
    # Rotationssymbol
    ax.add_patch(Arc((0, 28), 26, 10, theta1=200, theta2=-20, color="#16a34a", lw=1.4))
    ax.annotate("", xy=(13, 30), xytext=(11, 25),
                arrowprops=dict(arrowstyle="-|>", color="#16a34a", lw=1.4))
    ax.text(16, 31, "z_angle", fontsize=8, color="#16a34a")

    # Rückplatte (senkrecht) + LiDAR auf der Seite
    ax.add_patch(Rectangle((6, 28), 5, 70, fc="#cdd8e6", ec=INK, lw=1.2))       # Rückplatte
    ax.text(-13, 50, "Rückplatte\n(senkrecht)", fontsize=7.5, color=INK, ha="right")
    sy = 70  # Höhe Sensorzentrum
    ax.add_patch(Rectangle((11, sy - 18), 34, 36, fc="#f2dede", ec=ACC, lw=1.5))  # LiDAR
    ax.text(28, sy + 26, "STL27L (auf der Seite)", fontsize=8.5, color=ACC, ha="center")
    ax.add_patch(Circle((11, sy), 1.6, fc=ACC, ec=ACC))
    ax.text(11, sy - 24, "Optikzentrum S", fontsize=7.5, color=ACC, ha="center")

    # Koordinatentriade am Ursprung O (auf der Drehachse, Höhe Rotorplatte-Oberkante als Datum? -> O bei 0)
    axis_arrow(ax, 0, 40, 0, 62, "Z")
    axis_arrow(ax, 0, 40, 22, 40, "Y")
    into_page(ax, 0, 40, "X")
    ax.text(2, 37, "O", fontsize=9, color=INK)

    # Spinachse Y horizontal aus dem Sensor
    axis_arrow(ax, 11, sy, 52, sy, "Y (Spinachse, horizontal)", color="#b45309")
    # Scan-Ebene X-Z (in dieser Ansicht edge-on = senkrechte Linie durch S)
    ax.plot([11, 11], [sy - 40, sy + 40], color="#22d3ee", lw=1.3, dashes=[6, 3])
    ax.text(-30, 100, "Scan-Ebene X-Z\n(vertikal, senkr. zum Bild)", fontsize=7.5, color="#0e7490", ha="left")

    # Bemaßung: Sensorhöhe und Y-Versatz der Rückplatte
    dim_linear(ax, (0, 0), (0, sy), "center_height (Z)", off=0, vertical=True, side=-1)
    dim_linear(ax, (0, sy), (11, sy), "axis_offset (Y)", off=14, side=1)

    # Orientierung
    ax.text(-66, 100, "OBEN ↑", fontsize=9, color=INK, fontweight="bold")
    ax.text(-66, -6, "UNTEN ↓", fontsize=9, color=INK, fontweight="bold")
    ax.text(-66, 60, "Hinweis: Bodenplatte des\nLiDAR steht SENKRECHT an\nder Rückplatte; Optikfenster\nrundum frei halten.",
            fontsize=7.5, color="#444")


def build_montage():
    fig = plt.figure(figsize=(13, 8)); fig.patch.set_facecolor("white")
    fig.suptitle("PiLiDAR 2.0 — Technische Zeichnung: LiDAR-Montage & Ausrichtung",
                 fontsize=14, fontweight="bold", y=0.97)
    montage_view(fig.add_axes([0.06, 0.20, 0.6, 0.70]))
    title_block(fig, "LiDAR-Montage (Seitenmontage)")
    frame(fig)
    _save(fig, "lidar_montage")


# ---------------------------------------------------------------------
def scanprocess_view(ax):
    ax.set_title("Messkette — vom Strahl zum 3D-Punkt", fontsize=11, fontweight="bold")
    ax.set_xlim(-30, 70); ax.set_ylim(-30, 60); ax.set_aspect("equal"); ax.axis("off")
    # nutzt dieselbe Scan-Ebenen-Darstellung wie Ansicht B
    view_B(ax)


def build_scanprocess():
    fig = plt.figure(figsize=(15, 8)); fig.patch.set_facecolor("white")
    fig.suptitle("PiLiDAR 2.0 — Technische Zeichnung: Scan-Vorgang & Punkt-Berechnung",
                 fontsize=14, fontweight="bold", y=0.97)
    # links: Messgeometrie (Scan-Ebene), rechts: Rechenkette als Notizblock
    view_B(fig.add_axes([0.04, 0.20, 0.44, 0.70]))
    ax2 = fig.add_axes([0.52, 0.20, 0.44, 0.70]); ax2.axis("off")
    ax2.set_xlim(0, 10); ax2.set_ylim(0, 10)
    ax2.add_patch(Rectangle((0.2, 0.5), 9.6, 9.0, fill=False, ec=INK, lw=1.0))
    ax2.text(0.5, 9.0, "Rechenkette pro Messpunkt (α, r):", fontsize=10, fontweight="bold", color=INK)
    steps = [
        "1)  Ebene:  P = (r·cos α, 0, r·sin α)        [X-Z, y=0]",
        "2)  Y-Rotation um angle_offset  (In-Ebenen-Clocking)",
        "3)  + Offset  (0, MODEL_Y, MODEL_Z)",
        "4)  Z-Rotation um −z_angle  (Revolve um Drehachse)",
        "",
        "z_angle:",
        "   Modus A  →  konstant je Schritt",
        "   Modus B  →  z_angle = ω · Δt   (kontinuierlich)",
        "",
        "Einheiten: Längen in mm, Winkel in Grad.",
        "Reihenfolge ist verbindlich (Offset VOR Revolve).",
    ]
    for i, s in enumerate(steps):
        ax2.text(0.5, 8.2 - i * 0.66, s, fontsize=9,
                 family="monospace" if s.strip().startswith(("1", "2", "3", "4", "z_angle", "Modus")) else "sans-serif",
                 color=INK)
    title_block(fig, "Scan-Vorgang & Punkt-Berechnung")
    frame(fig)
    _save(fig, "scan_vorgang")


if __name__ == "__main__":
    build()
    build_montage()
    build_scanprocess()
