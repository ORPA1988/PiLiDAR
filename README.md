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

## Variante B: ESP32-Stepper und Betrieb (Stand 27.11.2026)

**Datenfluss.** Der Pi speichert je Scan nur `raw/lidar_raw.npz` (z-Winkel, 12 Winkel/Distanzen/
Intensitäten je Frame, Drehzahl, LiDAR-Zeitstempel, Empfangszeit) und eine `.ply`. Der Browser holt
gespeicherte Scans als Rohdaten (`/api/scans/<id>/raw`) und rechnet die Punktwolke im Web-Worker
mit den **aktuellen** Offsets – Kalibrierungen wirken damit auch auf alte Scans. Live-Daten kommen
binär gebündelt über `/ws/scan` (Format in `backend/app.py`, `_pack_frames`).

**Wichtige Einstellungen (`config.json`)**

| Schlüssel | Bedeutung |
|---|---|
| `LIDAR.MOTOR_CONTROL` | `"1"`/`"0"` an das Waveshare-Board senden (Spiegel an/aus). `false`: dort RXD0 kurzgeschlossen |
| `LIDAR.ANGLE_OFFSET`, `GEOMETRY.MODEL_Y/Z_OFFSET` | Geometrie; Winkel und Y per Offset-Kalibrierung, Z nicht bestimmbar |
| `STEPPER.BACKEND` | `"esp32"` (USB-Seriell, Firmware `firmware/esp32_stepper`) oder `"gpio"` |
| `STEPPER.RELEASE_AFTER_SCAN` | Treiber nach Scan/Kalibrierung abschalten (kein Fiepen, keine Wärme) |
| `STEPPER.OPEN_MODE` | `"reset"`: ESP32 startet beim Öffnen des Ports neu; `"keep"` nach Prüfung mit `scripts/esp32_reset_probe.py` |
| `STORAGE.RAW_FORMATS` / `POINTCLOUD_FORMATS` | Standard `["npy"]` / `["ply"]`; `jsonl`, `bin`, `xyz` nur auf Wunsch |

**Positionssperre.** Erkennt der Server einen Neustart des ESP32 (Startmeldung), sind Scans gesperrt,
bis in der Oberfläche „Ausgangslage bestätigen“ geklickt wird (`POST /api/stepper/confirm_home`).

**Kalibrierung ohne Motor.** In der Scanliste rechnet ⚙ die Offset-Kalibrierung aus den Rohdaten des
Scans (`POST /api/calibrate/offset_from_scan`, ~3 s). Der Messlauf „Offset (>180°)“ macht dasselbe
mit einem neuen Scan.

**Oberfläche (3D).** Farbe nach Intensität/Höhe/Entfernung/Drehwinkel/Scan-Nummer, Höhenschnitt
(„Decke aus“), Ansichten, Verschieben, Messen (zwei Punkte), Scans überlagern („+“ in der Liste),
Fortschritt mit Restzeit, Selbsttest (`GET /api/selftest`).

**Export (Browser).** PLY und LAS 1.2 der sichtbaren Punkte (optional ausgedünnt), PNG der Ansicht,
**Grundriss** (`frontend/js/floorplan.js`): nur senkrechte Flächen, Wände per RANSAC, Raummaße,
Fläche, Höhe; DXF (Schichten WAENDE, SCANPUNKTE, MASSE; Meter) und SVG 1:10.

**Schnittansicht** (`frontend/js/section.js`): senkrechter Schnitt durch einen Punkt in wählbarer Richtung
und Dicke (auch aus zwei Messpunkten oder an den Wänden ausgerichtet), Export DXF (Schicht SCHNITT), SVG 1:10, PNG.
**Flächen** (Farbmodus, `frontend/js/surfaces.js`): Boden, Decke, Wände, Schrägen und waagerechte Flächen je
10-cm-Würfel erkannt, gesehene Fläche je Art in m².
**Abweichung zwischen Scans** (Farbmodus): Abstand jedes Punkts zum jeweils anderen geladenen Scan, 0–10 cm.

**Mehrere Scans** (`frontend/js/registration.js`, eigener Worker): weitere Scans mit „+“ dazuladen,
„Scans ausrichten“ legt sie an den ersten (Drehung um z + Verschiebung; Scanner waagerecht). Grob:
Wandzellen, FFT-Kreuzkorrelation für alle Drehwinkel (2°); fein: ICP mit Fangradius 0,5 → 0,08 m.
Güte: Überdeckung (Punkte mit Nachbar < 10 cm) und Abweichung. Als **Projekt** auf dem Pi speichern
(`/api/projects`, Datei `projects/<name>.json`: Scan-IDs + Lage); Export und Grundriss nutzen die
ausgerichtete Wolke, der Grundriss zeigt alle Scannerstandorte. **Von Hand:** Scan wählen, mit
Pfeilen/Drehknöpfen (1 cm/0,5° bis 20 cm/10°) oder Zahlen verschieben, dann „Feinausrichten ab hier“.

**Offsets von Hand** (Kalibrierung → „Offsets von Hand“): Winkel, y, z eingeben, mit „Mit Scan prüfen“
die Überlappung (mm) an einem gespeicherten Scan berechnen (`/api/calibrate/check`), dann übernehmen.
y ist der Abstand der Scan-Ebene von der Abtriebsachse, nicht von der Motorwelle.
„Naht zeigen“ stellt nur Anfang (orange) und Ende (türkis) eines Scans dar und rechnet bei jeder
Änderung von Winkel, y, z und **Drehfaktor** (`GEOMETRY.Z_SCALE`, wahrer Drehwinkel = gemeldet · Faktor)
sofort neu – Kanten müssen deckungsgleich werden. **Hälften-Verdrehung** (`GEOMETRY.HALF_YAW`): feste Verdrehung der
vorderen gegen die hintere Fächerhälfte des LiDAR um die Drehachse (Geräteeigenschaft, im Original-PiLiDAR
`FAN_SPLIT`, dort ca. 2,4°). „Hälften zeigen“ stellt den ganzen Scan dar (vordere Hälfte orange, hintere türkis);
bei einem 360°-Scan sehen beide Hälften den ganzen Raum, „Mit Scan prüfen“ liefert dazu die Abweichung in mm. y und Verdrehung sind aus den Daten allein kaum trennbar.
**Kantenpunkte filtern** (3D-Darstellung): Punkte, die von beiden Winkel-Nachbarn weit abweichen, weglassen.

**Bedienfehler** kommen als HTTP 409 mit `{"error": "…"}` und erscheinen unten rechts.

**Frontend-Updates:** Versionskennung `?v=…` in `frontend/index.html` und `ASSET_VERSION` in
`frontend/js/app.js` gemeinsam erhöhen, sonst liefert der Browser-Cache alte Skripte.

**Dienst:** `pilidar.service`; Stopp-Frist per Drop-in `pilidar.service.d/stop.conf`
(`TimeoutStopSec=15`), uvicorn beendet sich nach 3 s selbst.

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
