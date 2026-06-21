# PiLiDAR 2.0

Optimierte Neuumsetzung des Open-Source-3D-Scanners **PiLiDAR** für Raspberry Pi 4
mit **Web-Steuerung**, **USB-LiDAR (LDROBOT STL27L)** und einer zweiten Messtechnik:
**kontinuierliche, ruckelfreie Drehung**.

> Abgeleitet von [PiLiDAR](https://github.com/PiLiDAR/PiLiDAR) · Lizenz **CC BY-NC-SA 4.0**
> (nicht-kommerziell, Namensnennung, Weitergabe unter gleichen Bedingungen).

## Highlights

- **Keine Hardware-Knöpfe** — komplette Steuerung über den Webserver.
- **LiDAR über USB** (STL27L-Controllerboard, CP2102 → `/dev/ttyUSB0`).
- **Rechenlast am Client**: Der Pi liefert nur Roh-Frames; Polar→Kartesisch, 3D-Revolve
  und Rendering laufen im Browser — **ohne jede Zusatzinstallation** (eigener WebGL-Viewer,
  kein CDN, kein three.js).
- **Live-Ansicht**: 2D-Polarplot (nur LiDAR, Motor aus) und optional 3D-Live während des Scans.
- **Zwei Messmodi**: A = schrittweise (Original), B = kontinuierlich konstant (Hardware-PWM).
- **Ein Ordner pro Scan** mit Rohdaten + Punktwolken in mehreren Formaten (ZIP-Download).
- **Referenzfahrt** nach jedem Scan, **Kalibrierprogramm** und **Qualitätssicherung** (Ampel).

## Architektur

```
Browser-Client (WebGL, kein Install)        Raspberry Pi 4 (FastAPI)
  2D/3D-Live · Punktwolke · PLY-Export  <—WS—  LiDAR-Reader-Thread (CRC8)
  Steuerung (REST)                      —REST→  Motor (Modus A: STEP/DIR,
                                                       Modus B: Hardware-PWM)
```

## Schnellstart

```bash
pip install -r requirements.txt

# Demo OHNE Hardware (synthetischer LiDAR):
PILIDAR_MOCK=1 python -m backend.app
# Browser: http://localhost:8000

# Auf dem Pi (mit Hardware):
#   1) /boot/firmware/config.txt:  dtoverlay=pwm-2chan   (Hardware-PWM, dann reboot)
#   2) Nutzer in dialout-Gruppe:   sudo usermod -a -G dialout $USER
#   3) python -m backend.app   →   http://<pi-ip>:8000
```

## Projektstruktur

```
backend/        FastAPI-Server, Hardware-Treiber (lidar, stepper), QA, Kalibrierung
frontend/       Web-UI (eigener 2D-Canvas- & 3D-WebGL-Viewer, Web Worker)
hardware/       stl27l_mount.scad (parametrischer Halter) + STL/PNG
docs/           Verkabelungspläne, BOM (xlsx), technisches Konzept (docx), Skizzen
tests/          Trockentests (CRC8, Decode, Mock-Scan) ohne Hardware
config.json     zentrale Konfiguration (Single-Source-of-Truth)
```

## Dokumentation (docs/)

- `PiLiDAR_Technisches_Konzept.docx` — Architektur, Mess-Mathematik, Montage, Inbetriebnahme
- `PiLiDAR_BOM_Specs.xlsx` — Stückliste, Specs, GPIO-Belegung, Kondensatoren/Kabel, Treibervergleich
- `images/` — Verkabelungspläne (A4988 & TMC2209), GPIO, Montage- und Scan-Vorgang-Skizzen

## Tests

```bash
python tests/test_core.py
```

## Hinweis zur Hardware

IMU (MPU6050) und Kamera sind vorbereitet, aber bewusst **noch nicht** umgesetzt.
Verkabelung/BOM dokumentieren **A4988** (Ist-Zustand) und **TMC2209** (Upgrade für glattere
kontinuierliche Drehung). **100 µF an VMOT/GND ist Pflicht.**
