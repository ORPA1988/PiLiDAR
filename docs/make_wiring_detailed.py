"""Detailgetreue, pin-genaue Verkabelungspläne (A4988 & TMC2209) im Schaltplan-
Stil — mit vollständiger Raspberry-Pi-40-Pin-Leiste, kompletten Treiber-Pinouts,
farbcodierten Adern, Kondensatoren an den korrekten Knoten, Motor-Spulenpaaren,
Netzteilen und LiDAR-USB. Ausgabe als PNG (300 dpi) und SVG in docs/images/.

Aufruf: python docs/make_wiring_detailed.py
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, Polygon, PathPatch
from matplotlib.path import Path

OUT = os.path.join(os.path.dirname(__file__), "images")
os.makedirs(OUT, exist_ok=True)

# Raspberry Pi 4 — 40-Pin-Header (physische Pinnummer -> (Name, Typ))
PI40 = {
    1: ("3V3", "pwr3"), 2: ("5V", "pwr5"),
    3: ("GPIO2 SDA", "gp"), 4: ("5V", "pwr5"),
    5: ("GPIO3 SCL", "gp"), 6: ("GND", "gnd"),
    7: ("GPIO4", "gp"), 8: ("GPIO14 TXD", "gp"),
    9: ("GND", "gnd"), 10: ("GPIO15 RXD", "gp"),
    11: ("GPIO17", "gp"), 12: ("GPIO18 PWM0", "gp"),
    13: ("GPIO27", "gp"), 14: ("GND", "gnd"),
    15: ("GPIO22", "gp"), 16: ("GPIO23", "gp"),
    17: ("3V3", "pwr3"), 18: ("GPIO24", "gp"),
    19: ("GPIO10 MOSI", "gp"), 20: ("GND", "gnd"),
    21: ("GPIO9 MISO", "gp"), 22: ("GPIO25", "gp"),
    23: ("GPIO11 SCLK", "gp"), 24: ("GPIO8 CE0", "gp"),
    25: ("GND", "gnd"), 26: ("GPIO7 CE1", "gp"),
    27: ("GPIO0 ID_SD", "gp"), 28: ("GPIO1 ID_SC", "gp"),
    29: ("GPIO5", "gp"), 30: ("GND", "gnd"),
    31: ("GPIO6", "gp"), 32: ("GPIO12 PWM0", "gp"),
    33: ("GPIO13 PWM1", "gp"), 34: ("GND", "gnd"),
    35: ("GPIO19 PWM1", "gp"), 36: ("GPIO16", "gp"),
    37: ("GPIO26", "gp"), 38: ("GPIO20", "gp"),
    39: ("GND", "gnd"), 40: ("GPIO21", "gp"),
}

PIN_TYPE_COLOR = {"pwr5": "#dc2626", "pwr3": "#f59e0b", "gnd": "#374151", "gp": "#9ca3af"}

W = {  # Aderfarben
    "STEP": "#ea580c", "DIR": "#eab308", "MS1": "#06b6d4", "MS2": "#0ea5e9",
    "MS3": "#3b82f6", "5V": "#dc2626", "VDD": "#dc2626", "GND": "#111827",
    "VMOT": "#7f1d1d", "A+": "#16a34a", "A-": "#86efac", "B+": "#2563eb",
    "B-": "#93c5fd", "TX": "#d946ef", "RX": "#a21caf", "USB": "#7c3aed",
}


def pi_header(ax, x0, y0):
    """Zeichnet die 40-Pin-Leiste; gibt dict pin-> (x,y) der Pin-Mittelpunkte."""
    pos = {}
    dy = 0.52
    ax.add_patch(FancyBboxPatch((x0 - 0.45, y0 - 20 * dy - 0.35), 3.5, 20 * dy + 0.7,
                 boxstyle="round,pad=0.02,rounding_size=0.1", fc="#0b3d2e",
                 ec="#065f46", lw=1.5))
    ax.text(x0 + 1.3, y0 + 0.25, "Raspberry Pi 4 — 40-Pin GPIO", ha="center",
            fontsize=10, fontweight="bold", color="white")
    for row in range(20):
        y = y0 - row * dy
        for col, pin in ((0, row * 2 + 1), (1, row * 2 + 2)):
            name, typ = PI40[pin]
            px = x0 + (0.0 if col == 0 else 1.6)
            ax.add_patch(Rectangle((px, y - 0.16), 0.32, 0.32, fc="#e5e7eb",
                         ec=PIN_TYPE_COLOR[typ], lw=1.6))
            ax.text(px + 0.16, y, str(pin), ha="center", va="center", fontsize=5.5,
                    color="#111")
            lbl_x = px - 0.12 if col == 0 else px + 0.44
            ha = "right" if col == 0 else "left"
            ax.text(lbl_x, y, name, ha=ha, va="center", fontsize=5.2, color="#cbd5e1")
            pos[pin] = (px + 0.16, y)
    return pos


def module(ax, x0, y0, w, h, title, left, right, fc, ec):
    """Treibermodul mit beschrifteten Pin-Reihen. left/right: Liste von Namen
    (von oben). Gibt dict name->(x,y) zurück."""
    ax.add_patch(FancyBboxPatch((x0, y0), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                 fc=fc, ec=ec, lw=2))
    ax.text(x0 + w / 2, y0 + h + 0.18, title, ha="center", fontsize=11,
            fontweight="bold", color=ec)
    pos = {}
    n = max(len(left), len(right))
    dy = (h - 0.5) / max(1, n - 1) if n > 1 else 0
    for i, name in enumerate(left):
        y = y0 + h - 0.25 - i * dy
        ax.add_patch(Circle((x0, y), 0.07, fc="#fde047", ec="#111", lw=0.6, zorder=5))
        ax.text(x0 + 0.15, y, name, ha="left", va="center", fontsize=6.5, color="white")
        pos[name] = (x0, y)
    for i, name in enumerate(right):
        y = y0 + h - 0.25 - i * dy
        ax.add_patch(Circle((x0 + w, y), 0.07, fc="#fde047", ec="#111", lw=0.6, zorder=5))
        ax.text(x0 + w - 0.15, y, name, ha="right", va="center", fontsize=6.5, color="white")
        pos[name] = (x0 + w, y)
    return pos


def wire(ax, p0, p1, color, lw=2.2, label="", via=None):
    pts = [p0] + (via or []) + [p1]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, lw=lw, solid_capstyle="round", zorder=3)
    if label:
        ax.text((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2 + 0.12, label,
                fontsize=6.5, color=color, ha="center")


def cap_polarized(ax, x, y, label):
    ax.plot([x, x], [y + 0.18, y + 0.06], color="#0891b2", lw=2)
    ax.plot([x - 0.12, x + 0.12], [y + 0.06, y + 0.06], color="#0891b2", lw=3)  # +
    ax.add_patch(Polygon([[x - 0.12, y - 0.02], [x + 0.12, y - 0.02], [x, y - 0.14]],
                 closed=True, fc="#0891b2"))  # - (gewölbt vereinfacht)
    ax.plot([x, x], [y - 0.14, y - 0.26], color="#0891b2", lw=2)
    ax.text(x + 0.18, y, label, fontsize=6.5, color="#0891b2", va="center")


def cap_ceramic(ax, x, y, label):
    ax.plot([x, x], [y + 0.18, y + 0.05], color="#0891b2", lw=2)
    ax.plot([x - 0.1, x + 0.1], [y + 0.05, y + 0.05], color="#0891b2", lw=2.5)
    ax.plot([x - 0.1, x + 0.1], [y - 0.05, y - 0.05], color="#0891b2", lw=2.5)
    ax.plot([x, x], [y - 0.05, y - 0.18], color="#0891b2", lw=2)
    ax.text(x + 0.16, y, label, fontsize=6.5, color="#0891b2", va="center")


def motor(ax, x0, y0):
    ax.add_patch(Circle((x0 + 0.9, y0 + 0.9), 0.9, fc="#4a2a06", ec="#b45309", lw=2))
    ax.text(x0 + 0.9, y0 + 0.9, "NEMA17\n42-23", ha="center", va="center",
            fontsize=8, color="white", fontweight="bold")
    # 4 Anschlüsse
    pins = {"A+": (x0, y0 + 1.4), "A-": (x0, y0 + 0.9),
            "B+": (x0, y0 + 0.4), "B-": (x0, y0 - 0.0)}
    for name, (px, py) in pins.items():
        ax.add_patch(Circle((px, py), 0.06, fc="#fde047", ec="#111", lw=0.6, zorder=5))
        ax.text(px - 0.12, py, name, ha="right", va="center", fontsize=6.5, color="#b45309")
    return pins


def psu(ax, x0, y0, w, h, label, color, sub):
    ax.add_patch(FancyBboxPatch((x0, y0), w, h, boxstyle="round,pad=0.02,rounding_size=0.1",
                 fc="#3f0f0f" if color == W["VMOT"] else "#3a0f0f", ec=color, lw=2))
    ax.text(x0 + w / 2, y0 + h - 0.28, label, ha="center", fontsize=9,
            fontweight="bold", color="white")
    ax.text(x0 + w / 2, y0 + 0.2, sub, ha="center", fontsize=7, color="#cbd5e1")
    return {"+": (x0 + w / 2, y0 + h), "-": (x0 + w / 2, y0)}


def legend(ax, x, y, items):
    ax.text(x, y + 0.35, "Legende / Adern", fontsize=8, fontweight="bold", color="white")
    for i, (lbl, col) in enumerate(items):
        yy = y - i * 0.3
        ax.plot([x, x + 0.4], [yy, yy], color=col, lw=3)
        ax.text(x + 0.5, yy, lbl, fontsize=7, color="#e5e7eb", va="center")


def base_fig(title):
    fig, ax = plt.subplots(figsize=(17, 11))
    fig.patch.set_facecolor("#0e1116"); ax.set_facecolor("#0e1116")
    ax.set_xlim(0, 20); ax.set_ylim(0, 13); ax.axis("off")
    ax.text(0.3, 12.6, title, fontsize=17, fontweight="bold", color="white")
    ax.text(0.3, 12.2, "PiLiDAR 2.0 — detailgetreuer, pin-genauer Verkabelungsplan",
            fontsize=9.5, color="#94a3b8")
    return fig, ax


def draw(driver):
    is_tmc = driver == "TMC2209"
    fig, ax = base_fig(f"Verkabelungsplan {driver} (STEP/DIR" +
                       (" + UART-Option)" if is_tmc else ")"))
    pin = pi_header(ax, 1.2, 11.3)

    # Treibermodul
    if is_tmc:
        left = ["EN", "MS1", "MS2", "PDN/UART", "STEP", "DIR", "CLK", "VREF*"]
        right = ["VM (VMOT)", "GND", "OB1 (B+)", "OB2 (B-)", "OA1 (A+)", "OA2 (A-)",
                 "VIO (3V3)", "GND"]
    else:
        left = ["ENABLE", "MS1", "MS2", "MS3", "RESET", "SLEEP", "STEP", "DIR"]
        right = ["VMOT", "GND", "2B (B-)", "2A (B+)", "1A (A+)", "1B (A-)", "VDD", "GND"]
    dpos = module(ax, 9.0, 5.2, 3.0, 4.8, driver, left, right, "#0c3a22", "#16a34a")

    mpins = motor(ax, 16.2, 7.2)
    p12 = psu(ax, 9.2, 1.0, 2.6, 1.3, "Motor-Netzteil 12 V", W["VMOT"], "≥ 2–3 A")
    p5 = psu(ax, 5.4, 1.0, 2.6, 1.3, "USB-C 5 V", "#dc2626", "Pi ≥ 3 A")

    # LiDAR (USB)
    ax.add_patch(FancyBboxPatch((5.4, 10.1), 3.0, 1.4, boxstyle="round,pad=0.02,rounding_size=0.1",
                 fc="#2e1065", ec=W["USB"], lw=2))
    ax.text(6.9, 11.15, "STL27L LiDAR", ha="center", fontsize=10, fontweight="bold", color="white")
    ax.text(6.9, 10.78, "ZH1.5T-4P → Controllerboard (CP2102)", ha="center", fontsize=6.8, color="#cbd5e1")
    ax.text(6.9, 10.5, "Pin: 1=Tx · 2=PWM · 3=GND · 4=VCC(5V)", ha="center", fontsize=6.5, color="#cbd5e1")
    ax.text(6.9, 10.24, "→ USB an Raspberry Pi (kein GPIO-Pegelwandler)", ha="center",
            fontsize=6.8, color=W["USB"], fontweight="bold")
    # USB-Verbindung als dicke Linie zum Pi-Board
    wire(ax, (5.4, 10.6), (4.2, 10.6), W["USB"], lw=3, via=[(4.7, 10.6)])
    ax.text(4.0, 10.6, "USB", ha="right", fontsize=7, color=W["USB"])

    # --- Signaladern Pi -> Treiber ---
    sig_map = [(35, "STEP", "STEP"), (37, "DIR", "DIR"),
               (29, "MS1", "MS1"), (31, "MS2", "MS2")]
    if not is_tmc:
        sig_map.append((33, "MS3", "MS3"))
    busx = 6.3
    for k, (p, drvpin, col) in enumerate(sig_map):
        x0, y0 = pin[p]
        yb = 9.4 - k * 0.32
        wire(ax, (x0 + 0.18, y0), dpos[drvpin], W[col],
             via=[(busx + k * 0.12, y0), (busx + k * 0.12, dpos[drvpin][1])])
    # STEP-Hinweis (oben, freier Bereich über dem Treiber)
    ax.text(10.5, 11.5, "STEP = GPIO19 = Hardware-PWM1\n(Modus B: konstante Drehung)",
            fontsize=7.5, color=W["STEP"], ha="center")

    # MS-Pegel-Hinweis (1/16 bzw. interne PU bei TMC)
    if is_tmc:
        ax.text(9.0, 4.9, "MS1/MS2 = Adresse/Microstep; UART setzt Strom & 1/256 in Software",
                fontsize=6.8, color="#86efac")
        # UART
        x14, y14 = pin[8]   # GPIO14 TXD
        x15, y15 = pin[10]  # GPIO15 RXD
        wire(ax, (x14 + 0.18, y14), dpos["PDN/UART"], W["TX"],
             via=[(5.0, y14), (5.0, dpos["PDN/UART"][1] + 0.2), (8.4, dpos["PDN/UART"][1] + 0.2)], lw=1.8)
        ax.text(5.05, y14 + 0.15, "TX→PDN (1 kΩ)", fontsize=6, color=W["TX"])
        ax.text(8.6, dpos["PDN/UART"][1] + 0.32, "UART optional", fontsize=6, color=W["RX"])
        ax.text(9.0, 5.0, "VREF* nur falls ohne UART", fontsize=6, color="#86efac")
    else:
        # EN auf GND (immer aktiv), RESET<->SLEEP gebrückt
        wire(ax, dpos["ENABLE"], (8.4, dpos["ENABLE"][1]), W["GND"], lw=1.6,
             via=[(8.4, dpos["ENABLE"][1])])
        ax.text(8.3, dpos["ENABLE"][1], "EN→GND", fontsize=6, color=W["GND"], ha="right")
        ax.plot([dpos["RESET"][0], dpos["RESET"][0] - 0.25, dpos["SLEEP"][0] - 0.25, dpos["SLEEP"][0]],
                [dpos["RESET"][1], dpos["RESET"][1], dpos["SLEEP"][1], dpos["SLEEP"][1]],
                color="#9ca3af", lw=1.6)
        ax.text(dpos["RESET"][0] - 0.3, (dpos["RESET"][1] + dpos["SLEEP"][1]) / 2,
                "RST↔SLP", fontsize=6, color="#9ca3af", ha="right")
        ax.text(9.0, 5.0, "Strom per VREF-Poti:  I = VREF / (8 · Rcs)",
                fontsize=6.8, color="#86efac")

    # --- VMOT 12V + 100µF ---
    vmot = dpos["VM (VMOT)"] if is_tmc else dpos["VMOT"]
    gnd_r = dpos["GND"]  # erste GND rechts (unter VMOT)
    wire(ax, p12["+"], vmot, W["VMOT"], lw=3, via=[(vmot[0] + 0.0, p12["+"][1])])
    ax.text(vmot[0] + 0.25, p12["+"][1] + 0.6, "VMOT 12 V", fontsize=7, color=W["VMOT"])
    # 100µF direkt an VMOT/GND
    cap_polarized(ax, 12.6, vmot[1] - 0.4, "100 µF / ≥25 V")
    wire(ax, vmot, (12.6, vmot[1] - 0.2), W["VMOT"], lw=2, via=[(12.6, vmot[1])])
    wire(ax, (12.6, vmot[1] - 0.66), (12.6, 1.65), W["GND"], lw=2)
    ax.text(13.0, vmot[1] - 0.9, "100 µF DIREKT an VMOT/GND — PFLICHT!",
            fontsize=7, color="#0891b2", rotation=0)

    # --- VDD/VIO + 0.1µF ---
    vdd = dpos["VIO (3V3)"] if is_tmc else dpos["VDD"]
    x1, y1 = pin[1]  # 3V3
    wire(ax, (x1 + 0.18, y1), vdd, W["VDD"], lw=1.8,
         via=[(4.8, y1), (4.8, vdd[1]), (vdd[0] - 0.0, vdd[1])])
    cap_ceramic(ax, 12.55, vdd[1], "0,1 µF")
    wire(ax, vdd, (12.55, vdd[1] + 0.18), W["VDD"], lw=1.5)

    # --- Spulen Treiber -> Motor ---
    coil_map = ([("OA1 (A+)", "A+"), ("OA2 (A-)", "A-"), ("OB1 (B+)", "B+"), ("OB2 (B-)", "B-")]
                if is_tmc else
                [("1A (A+)", "A+"), ("1B (A-)", "A-"), ("2A (B+)", "B+"), ("2B (B-)", "B-")])
    for dn, mn in coil_map:
        wire(ax, dpos[dn], mpins[mn], W[mn], lw=2.4,
             via=[(13.8, dpos[dn][1]), (13.8, mpins[mn][1])])
    ax.text(14.6, 9.4, "Spulenpaare A+/A−, B+/B−\n18–20 AWG, verdrillt\n(Motorfarben herstellerabh.)",
            fontsize=6.8, color="#b45309")

    # --- Sternmasse ---
    gpin = pin[39]  # GND (Pin 39)
    starx, stary = 7.0, 2.6
    ax.add_patch(Circle((starx, stary), 0.09, fc="#e5e7eb", ec="#111"))
    ax.text(starx, stary - 0.3, "Sternmasse (EIN Punkt)", fontsize=7, color="#e5e7eb", ha="center")
    wire(ax, (gpin[0] + 0.18, gpin[1]), (starx, stary), W["GND"], lw=2, via=[(5.0, gpin[1]), (5.0, stary)])
    wire(ax, p5["-"], (starx, stary), W["GND"], lw=2, via=[(p5["-"][0], stary)])
    wire(ax, p12["-"], (starx, stary), W["GND"], lw=2, via=[(p12["-"][0], 0.7), (starx, 0.7)])
    gnd2 = dpos["GND"]
    wire(ax, (gnd2[0], gnd2[1]), (starx, stary), W["GND"], lw=1.6,
         via=[(12.6, 1.65), (12.6, 2.6)])
    # 5V vom Netzteil zum Pi (Pin 2)
    p2 = pin[2]
    wire(ax, p5["+"], (p2[0] + 0.18, p2[1]), W["5V"], lw=2.2, via=[(p5["+"][0], 9.2), (3.55, 9.2), (3.55, p2[1])])
    ax.text(3.7, 9.1, "5 V → Pi (Pin 2/4)", fontsize=6.5, color=W["5V"], ha="left")

    # Legende + Hinweise (oben rechts, freier Bereich)
    legend(ax, 16.4, 12.0, [
        ("STEP (PWM1)", W["STEP"]), ("DIR", W["DIR"]), ("MS1/2/3", W["MS2"]),
        ("VMOT 12 V", W["VMOT"]), ("5V/VDD", W["5V"]), ("GND", W["GND"]),
        ("Spule A", W["A+"]), ("Spule B", W["B+"]), ("USB/UART", W["USB"]),
    ])
    notes = ("Pflicht/Best Practice:  • 100 µF Elko DIREKT an VMOT/GND (LC-Spitzen > 35 V zerstören den Treiber).  "
             "• 0,1 µF an VDD/VIO.  • getrennte Netzteile, EINE Sternmasse.  "
             "• Motor 18–20 AWG verdrillt, Signal 24–28 AWG; Schirm einseitig auf GND; Ferrit auf USB.")
    ax.text(0.3, 0.3, notes, fontsize=7.2, color="#cbd5e1")

    name = f"verkabelung_{driver.lower()}_detail"
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"), dpi=300,
                    facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    print("ok", name)


if __name__ == "__main__":
    draw("A4988")
    draw("TMC2209")
    print("fertig:", OUT)
