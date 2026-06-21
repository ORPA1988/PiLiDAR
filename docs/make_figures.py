"""Erzeugt alle Bild-Deliverables (Verkabelung, GPIO, Montage, Scan-Vorgang)
als PNG (300 dpi, für Word) und SVG (verlustfrei) in docs/images/.

Aufruf:  python docs/make_figures.py
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, FancyArrowPatch, Polygon

OUT = os.path.join(os.path.dirname(__file__), "images")
os.makedirs(OUT, exist_ok=True)

C_PI = "#2d6cdf"
C_DRV = "#16a34a"
C_MOT = "#b45309"
C_PWR = "#dc2626"
C_LID = "#7c3aed"
C_CAP = "#0891b2"
C_BG = "#0e1116"


def box(ax, x, y, w, h, label, color, sub="", fc=None, text_color="white", fs=11):
    fc = fc or color
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                linewidth=1.5, edgecolor=color, facecolor=fc, alpha=0.95))
    ax.text(x + w / 2, y + h - 0.32, label, ha="center", va="top",
            fontsize=fs, fontweight="bold", color=text_color)
    if sub:
        ax.text(x + w / 2, y + 0.22, sub, ha="center", va="bottom",
                fontsize=8.5, color=text_color)


def pin(ax, x, y, label, side="left", color="#111"):
    ax.add_patch(Circle((x, y), 0.06, color=color, zorder=5))
    dx = -0.16 if side == "left" else 0.16
    ha = "right" if side == "left" else "left"
    ax.text(x + dx, y, label, ha=ha, va="center", fontsize=8, color="#cbd5e1")


def wire(ax, p0, p1, color="#94a3b8", lw=2.0, label="", ls="-"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-", mutation_scale=1,
                                 linewidth=lw, color=color, linestyle=ls, zorder=2))
    if label:
        mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        ax.text(mx, my + 0.12, label, ha="center", va="bottom", fontsize=7.5, color=color)


def new_ax(title, w=13, h=8):
    fig, ax = plt.subplots(figsize=(w, h))
    fig.patch.set_facecolor(C_BG)
    ax.set_facecolor(C_BG)
    ax.set_xlim(0, 13); ax.set_ylim(0, 8); ax.axis("off")
    ax.text(0.2, 7.7, title, fontsize=15, fontweight="bold", color="white")
    ax.text(0.2, 7.38, "PiLiDAR 2.0 — Verkabelungs- & Aufbaudokumentation",
            fontsize=9, color="#94a3b8")
    return fig, ax


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"), dpi=300,
                    facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    print("ok", name)


# ---------------------------------------------------------------------
def fig_wiring(driver: str):
    is_tmc = driver == "TMC2209"
    fig, ax = new_ax(f"Verkabelungsplan — Schrittmotor-Treiber {driver}")

    # --- Komponenten (überlappungsfrei platziert) ---
    box(ax, 0.5, 3.0, 2.5, 2.7, "Raspberry Pi 4", C_PI,
        sub="3,3 V Logik", fc="#13294f")
    box(ax, 0.5, 6.05, 2.5, 0.95, "STL27L LiDAR", C_LID, sub="USB (CP2102)", fc="#2e1065", fs=10)
    box(ax, 5.4, 3.0, 2.3, 2.8, driver, C_DRV,
        sub=("UART-Strom" if is_tmc else "VREF-Poti"), fc="#0c3a22")
    box(ax, 10.2, 3.7, 2.3, 1.9, "NEMA17\n42-23", C_MOT, sub="1,8° · ~1,2 A/Phase", fc="#4a2a06")
    box(ax, 5.4, 0.8, 2.3, 1.2, "Motor-NT 12 V", C_PWR, sub="≥2–3 A", fc="#4a0f0f")
    box(ax, 0.5, 0.8, 2.5, 1.2, "USB-C 5 V", C_PWR, sub="Pi (≥3 A)", fc="#4a0f0f")

    # Kondensatoren
    box(ax, 8.15, 1.5, 1.5, 1.0, "100 µF", C_CAP, sub="Elko ≥25 V", fc="#083344", fs=9)
    box(ax, 8.4, 5.95, 1.5, 0.8, "0,1 µF", C_CAP, sub="X7R", fc="#083344", fs=9)

    # --- LiDAR via USB ---
    wire(ax, (1.75, 6.05), (1.75, 5.7), color=C_LID, lw=2.2)
    ax.text(3.15, 6.5, "USB → /dev/ttyUSB0\n(kein GPIO-Pegelwandler nötig)",
            fontsize=7.5, color=C_LID, va="center")

    # --- Signalpins Pi -> Treiber ---
    sig = [("GPIO19 STEP", 5.5), ("GPIO26 DIR", 5.1),
           ("GPIO5 MS1", 4.7), ("GPIO6 MS2", 4.3), ("GPIO13 MS3", 3.9)]
    for lbl, y in sig:
        pin(ax, 3.0, y, lbl, side="right")
        wire(ax, (3.0, y), (5.4, y), color="#60a5fa", lw=1.6)
    ax.text(4.2, 5.95, "STEP = Hardware-PWM\n(Modus B: konst. Drehung)",
            ha="center", fontsize=7.5, color="#60a5fa")

    # --- Sternmasse ---
    wire(ax, (3.0, 3.2), (3.9, 3.2), color="#e5e7eb", lw=2.2)
    wire(ax, (3.9, 3.2), (3.9, 1.4), color="#e5e7eb", lw=2.2)
    wire(ax, (3.9, 1.4), (5.4, 1.4), color="#e5e7eb", lw=2.2)
    ax.text(4.05, 2.3, "Sternmasse\n(EIN Punkt)", fontsize=8, color="#e5e7eb")

    # --- 12V -> VMOT + 100µF ---
    wire(ax, (6.55, 2.0), (6.55, 3.0), color=C_PWR, lw=2.6, label="VMOT 12 V")
    wire(ax, (7.7, 3.4), (8.15, 2.3), color=C_PWR, lw=2.0)        # zur Kappe
    wire(ax, (8.9, 1.5), (8.9, 1.1), color="#e5e7eb", lw=2.0)      # Kappe GND
    wire(ax, (8.9, 1.1), (6.55, 1.1), color="#e5e7eb", lw=2.0)
    wire(ax, (6.55, 1.1), (6.55, 0.8), color="#e5e7eb", lw=2.0)
    ax.text(9.7, 2.0, "100 µF DIREKT\nan VMOT/GND!", fontsize=8, color=C_CAP, va="center")

    # --- Treiber -> Motor (4 Adern verdrillt) ---
    for k, y in enumerate([4.9, 4.5, 4.1, 3.7]):
        wire(ax, (7.7, y), (10.2, 3.9 + k * 0.35), color=C_MOT, lw=2.0)
    ax.text(8.95, 5.35, "A+/A−, B+/B−\n18–20 AWG, verdrillt", ha="center",
            fontsize=7.5, color=C_MOT)

    # --- 0,1 µF an VDD/Logik ---
    wire(ax, (7.7, 5.5), (8.4, 6.2), color="#60a5fa", lw=1.6, label="VDD")

    if is_tmc:
        pin(ax, 3.0, 3.5, "GPIO TX/RX", side="right")
        wire(ax, (3.0, 3.5), (5.4, 3.5), color="#f59e0b", lw=1.6, ls="--",
             label="PDN_UART (1 kΩ)")
        ax.text(6.55, 6.55, "StealthChop2: leise & sehr glatt (bis 1/256)",
                ha="center", fontsize=8, color="#86efac")
    else:
        ax.text(6.55, 6.55, "Strom per VREF-Poti:  I = VREF / (8 · Rcs)",
                ha="center", fontsize=8, color="#86efac")

    notes = ("Hinweise:  • 100 µF Elektrolyt ZWINGEND direkt an VMOT/GND (LC-Spitzen > 35 V zerstören den Treiber).   "
             "• Getrennte Netzteile Pi/Motor, EINE gemeinsame Sternmasse.   "
             "• Signal 24–28 AWG, Motor 18–20 AWG verdrillt; Schirm einseitig auf GND.")
    ax.text(0.2, 0.2, notes, fontsize=7.5, color="#cbd5e1")
    save(fig, f"verkabelung_{driver.lower()}")


# ---------------------------------------------------------------------
def fig_gpio():
    fig, ax = new_ax("Raspberry Pi 4 — belegte GPIO-Pins (BCM)")
    used = {
        "GPIO19 (Pin 35)": ("STEP / Hardware-PWM1", C_DRV),
        "GPIO26 (Pin 37)": ("DIR", C_DRV),
        "GPIO5 (Pin 29)": ("MS1", C_DRV),
        "GPIO6 (Pin 31)": ("MS2", C_DRV),
        "GPIO13 (Pin 33)": ("MS3 / PWM1-alt", C_DRV),
        "5V (Pin 2/4)": ("Treiber-Logik VDD (optional)", C_PWR),
        "GND (Pin 6/9/…)": ("gemeinsame Sternmasse", "#e5e7eb"),
        "USB-A": ("STL27L LiDAR (CP2102)", C_LID),
        "GPIO14/15 (TX/RX)": ("nur TMC2209-UART-Option", C_MOT),
    }
    y = 6.6
    for k, (v, c) in used.items():
        ax.add_patch(Circle((1.0, y), 0.12, color=c))
        ax.text(1.4, y, k, fontsize=11, color="white", va="center", fontweight="bold")
        ax.text(6.0, y, v, fontsize=10, color="#cbd5e1", va="center")
        y -= 0.62
    ax.text(0.2, 0.4, "Hinweis: GPIO ist 3,3-V-Logik. Der LiDAR hängt am USB, daher keine "
            "UART-Pegelwandlung am GPIO nötig.", fontsize=8.5, color="#cbd5e1")
    save(fig, "gpio_pinout")


# ---------------------------------------------------------------------
def fig_mounting():
    fig, ax = new_ax("Korrekte LiDAR-Montage (STL27L) — Ausrichtung")
    # Drehachse Z
    ax.annotate("", xy=(6.5, 7.0), xytext=(6.5, 1.2),
                arrowprops=dict(arrowstyle="->", color="#e5e7eb", lw=2))
    ax.text(6.65, 6.9, "Z = Drehachse (senkrecht, Stepper)", color="#e5e7eb", fontsize=9)

    # Grundplatte / Rotor
    ax.add_patch(Rectangle((4.7, 1.4), 3.6, 0.35, color="#334155"))
    ax.text(6.5, 1.15, "Rotor-Grundplatte (dreht um Z)", ha="center", color="#94a3b8", fontsize=8)

    # Rückplatte (senkrecht)
    ax.add_patch(Rectangle((6.2, 1.75), 0.3, 3.6, color="#475569"))
    ax.text(5.0, 5.2, "senkrechte\nRückplatte", color="#94a3b8", fontsize=8, ha="center")

    # LiDAR (auf der Seite) – Körper
    ax.add_patch(Rectangle((6.5, 2.4), 1.7, 2.4, color=C_LID, alpha=0.9))
    ax.text(7.35, 4.95, "STL27L (auf der Seite)", ha="center", color="white", fontsize=9, fontweight="bold")
    ax.text(7.35, 2.2, "Bodenplatte an Rückplatte", ha="center", color="#cbd5e1", fontsize=8)

    # Spinachse Y (horizontal)
    ax.annotate("", xy=(9.6, 3.6), xytext=(6.6, 3.6),
                arrowprops=dict(arrowstyle="->", color="#f59e0b", lw=2))
    ax.text(9.7, 3.6, "Y = Spinachse\n(horizontal)", color="#f59e0b", fontsize=9, va="center")

    # vertikale Scan-Ebene (X-Z) als gestrichelte Ellipse
    from matplotlib.patches import Ellipse
    ax.add_patch(Ellipse((7.35, 3.6), 0.5, 4.6, fill=False, edgecolor="#22d3ee",
                         lw=1.8, linestyle="--"))
    ax.text(7.35, 6.05, "vertikale Scan-Ebene (X-Z)\n— 360° um die Spinachse",
            ha="center", color="#22d3ee", fontsize=8.5)

    txt = ("Merksätze:\n"
           "• OBEN/UNTEN: Bodenplatte des LiDAR steht SENKRECHT an der Rückplatte.\n"
           "• Spinachse zeigt HORIZONTAL nach außen (Y) → Scan-Ebene ist VERTIKAL.\n"
           "• Optikfenster rundum FREI halten (keine Verdeckung in der X-Z-Ebene).\n"
           "• Stecker ZH1.5T-4P (Tx,PWM,GND,VCC) unten → Kabel mitdrehend, Zugentlastung.\n"
           "• Würde der LiDAR FLACH liegen, wäre die Scan-Ebene horizontal → keine 3D-Wolke!")
    ax.text(0.3, 4.4, txt, fontsize=8.5, color="#e2e8f0", va="top")
    save(fig, "lidar_montage")


# ---------------------------------------------------------------------
def fig_scanprocess():
    fig, ax = new_ax("Scan-Vorgang & Punkt-Berechnung", w=13, h=8)

    # 1) Gehäuse-Aufbau (links)
    ax.text(2.2, 6.9, "1) Aufbau", color="white", fontsize=11, fontweight="bold", ha="center")
    ax.add_patch(Rectangle((1.2, 1.6), 2.0, 0.3, color="#334155"))      # Sockel
    ax.add_patch(Rectangle((1.9, 1.9), 0.6, 0.8, color="#1f2937"))      # Motor+Getriebe
    ax.text(2.2, 2.05, "Motor+\nGetriebe", color="#94a3b8", fontsize=6.5, ha="center")
    ax.add_patch(Rectangle((2.05, 2.7), 0.3, 1.8, color="#475569"))     # Welle/Platte
    ax.add_patch(Rectangle((2.0, 4.5), 0.9, 1.1, color=C_LID))          # LiDAR
    ax.text(2.45, 5.05, "LiDAR", color="white", fontsize=7, ha="center")
    ax.annotate("", xy=(2.2, 6.4), xytext=(2.2, 1.6),
                arrowprops=dict(arrowstyle="->", color="#e5e7eb", lw=1.5))
    ax.text(2.35, 6.25, "Z (Drehachse)", color="#e5e7eb", fontsize=7)

    # 2) Platzierung + Offset (mitte)
    ax.text(6.5, 6.9, "2) Platzierung + Offset", color="white", fontsize=11,
            fontweight="bold", ha="center")
    cx, cz = 6.0, 3.7
    ax.annotate("", xy=(6.0, 6.2), xytext=(6.0, 1.6),
                arrowprops=dict(arrowstyle="->", color="#e5e7eb", lw=1.5))  # Z-Achse
    ax.text(6.05, 6.05, "Z", color="#e5e7eb", fontsize=8)
    ax.add_patch(Circle((cx, cz), 0.10, color="#e5e7eb"))
    ax.text(6.0, 1.45, "Drehachse", color="#94a3b8", fontsize=7, ha="center")
    # LiDAR-Zentrum versetzt
    lx, lz = 7.2, 4.5
    ax.add_patch(Circle((lx, lz), 0.12, color=C_LID))
    ax.text(lx + 0.2, lz, "LiDAR-Zentrum", color=C_LID, fontsize=7.5, va="center")
    ax.annotate("", xy=(lx, cz), xytext=(cx, cz),
                arrowprops=dict(arrowstyle="<->", color="#f59e0b", lw=1.5))
    ax.text((cx + lx) / 2, cz - 0.25, "MODEL_Y = -37,5 mm", color="#f59e0b", fontsize=7.5, ha="center")
    ax.annotate("", xy=(lx, lz), xytext=(lx, cz),
                arrowprops=dict(arrowstyle="<->", color="#22d3ee", lw=1.5))
    ax.text(lx + 0.15, (cz + lz) / 2, "MODEL_Z\n= -41,9 mm", color="#22d3ee", fontsize=7)
    ax.text(6.5, 2.1, "angle_offset = -1,05°\n(Kippung der Ebene um Y)",
            color="#cbd5e1", fontsize=7.5, ha="center")

    # 3) Punkt-Berechnung (rechts)
    ax.text(10.7, 6.9, "3) Punkt-Berechnung", color="white", fontsize=11,
            fontweight="bold", ha="center")
    ax.add_patch(Circle((10.7, 4.2), 0.08, color="#e5e7eb"))
    ax.text(10.7, 4.0, "Sensor", color="#94a3b8", fontsize=7, ha="center")
    # ein Strahl
    px, pz = 12.2, 5.2
    ax.annotate("", xy=(px, pz), xytext=(10.7, 4.2),
                arrowprops=dict(arrowstyle="->", color=C_LID, lw=1.8))
    ax.add_patch(Circle((px, pz), 0.08, color="#f87171"))
    ax.text(px + 0.1, pz, "Punkt", color="#f87171", fontsize=7.5, va="center")
    ax.text(11.2, 4.55, "α (Winkel),\nr (Distanz)", color=C_LID, fontsize=7.5)

    formula = (r"Pro Punkt:" "\n"
               r"1) Ebene:  x=r·cos α,  z=r·sin α,  y=0" "\n"
               r"2) Y-Rotation um angle_offset" "\n"
               r"3) + Offset (0, MODEL_Y, MODEL_Z)" "\n"
               r"4) Z-Rotation um -z_angle" "\n"
               r"   Modus A: z_angle pro Schritt" "\n"
               r"   Modus B: z_angle = ω·Δt (kontinuierlich)")
    ax.text(8.9, 3.4, formula, color="#e2e8f0", fontsize=8, va="top",
            family="monospace")
    save(fig, "scan_vorgang")


if __name__ == "__main__":
    fig_wiring("A4988")
    fig_wiring("TMC2209")
    fig_gpio()
    fig_mounting()
    fig_scanprocess()
    print("\nAlle Figuren in", OUT)
