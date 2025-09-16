# PiLiDAR – Von der Installation bis zur fertigen Punktwolke

Willkommen bei PiLiDAR!  Dieses Projekt kombiniert einen LiDAR-Sensor und eine
Kamera, um mit einem Raspberry Pi eine drehbare 3D-Scanstation aufzubauen.  Die
Software wurde in diesem Arbeitsschritt komplett überarbeitet, damit auch
Einsteigerinnen und Einsteiger jeden Schritt nachvollziehen können.  Außerdem
gibt es jetzt einen Simulationsmodus, der ohne echte Hardware sofort eine
Punktwolke, Bilder und eine Fusion beider Datensätze erzeugt.

## Inhaltsverzeichnis

1. [Überblick](#überblick)
2. [Voraussetzungen](#voraussetzungen)
3. [Installation](#installation)
4. [Simulation ohne Hardware](#simulation-ohne-hardware)
5. [Einen Scan durchführen](#einen-scan-durchführen)
6. [Ausgabeordner und Dateien](#ausgabeordner-und-dateien)
7. [Projektübersicht: Dateien & Skripte erklärt](#projektübersicht-dateien--skripte-erklärt)
8. [Hardware-Hinweise](#hardware-hinweise)
9. [Fehlerbehebung](#fehlerbehebung)

## Überblick

PiLiDAR automatisiert einen kompletten 3D-Scan:

* Der Schrittmotor dreht die Plattform gleichmäßig.
* Die Kamera schießt automatisch HDR-Fotos.
* Der LiDAR-Sensor misst Entfernungen.
* Die Software erstellt eine Punktwolke, eine Panoramaaufnahme und eine
  Fusion aus Geometrie und Farbe.

Jeder Scan wird in einem eigenen Ordner im Verzeichnis `scans/` abgelegt, damit
die Ergebnisse sauber getrennt bleiben.

## Voraussetzungen

Damit die Installation reibungslos funktioniert, sollten folgende Punkte
erfüllt sein:

### Hardware

* Raspberry Pi 4 mit 64‑bit Raspberry Pi OS (Lite reicht aus).
* LiDAR-Sensor STL27L von LDRobot.
* Raspberry Pi HQ Kamera (oder eine kompatible Kamera).
* A4988 Schrittmotor-Treiber + NEMA17 Motor.
* Stabiles Netzteil (mind. 3 A) oder ein leistungsfähiger Akku.

### Software/Tools

* Git (wird für das Klonen des Repositories benötigt).
* Python 3.11 (unter Raspberry Pi OS bereits vorinstalliert).
* Optional: `hugin` für das automatische Panorama-Stitching (nur nötig, wenn
  echte Kamerafotos verwendet werden).

## Installation

Die folgenden Schritte richten das Projekt vollständig ein.  Jeder Schritt ist
bewusst ausführlich kommentiert.

1. **System aktualisieren** – veraltete Pakete können Fehler verursachen.

   ```bash
   sudo apt update
   sudo apt full-upgrade -y
   ```

2. **Wichtige Zusatzpakete installieren** – sie werden später von der Software
   benötigt (z. B. zum Bauen von Open3D oder für das Panorama-Tool Hugin).

   ```bash
   sudo apt install -y git python3-venv python3-pip libatlas-base-dev \
       libopenblas-dev liblapack-dev libhdf5-dev libgl1-mesa-glx hugin-tools
   ```

   *Hinweis:* `hugin-tools` ist für die Panoramaerstellung zuständig.  Wer nur
   den Simulationsmodus nutzen möchte, kann dieses Paket weglassen.

3. **Projekt herunterladen** – legt das Repository im Home-Verzeichnis an.

   ```bash
   cd ~
   git clone https://github.com/<Ihr-Git-Benutzername>/PiLiDAR.git
   cd PiLiDAR
   ```

4. **Python-Umgebung erstellen** – trennt Projektabhängigkeiten vom restlichen
   System.

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

5. **Python-Abhängigkeiten installieren** – alle benötigten Bibliotheken werden
   aus `requirements.txt` geladen.

   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

6. **Serielle Schnittstelle freischalten** – nur auf echter Hardware nötig, um
   den LiDAR anzusprechen.

   ```bash
   sudo usermod -a -G dialout $USER
   sudo reboot
   ```

   Durch den Neustart wird die neue Gruppenberechtigung aktiv.

Nach diesen Schritten ist die Software einsatzbereit.  Aktivieren Sie bei
jeder neuen Sitzung wieder die virtuelle Umgebung (`source .venv/bin/activate`).

## Simulation ohne Hardware

Für erste Tests oder Demonstrationen lässt sich der komplette Ablauf simulieren
– keine Kamera, kein LiDAR nötig.  Die Software erzeugt dabei künstliche Daten
und legt sie genau wie echte Messwerte ab.

1. Öffnen Sie `config.json` und setzen Sie `"SIMULATION_MODE": true`.
2. Starten Sie das Programm wie gewohnt (`python PiLiDAR.py`).
3. Nach wenigen Sekunden liegt im Ordner `scans/<Datum-Zeit>/` eine vollständige
   Datensammlung mit Bildern, Rohdaten, Punktwolke und Fusion.

Der Simulationsmodus ist hervorragend geeignet, um den Workflow zu testen oder
Neulingen das Ergebnis eines Scans zu zeigen.

## Einen Scan durchführen

1. **Hardware anschließen** – verbinden Sie LiDAR, Kamera und Stepper mit dem
   Raspberry Pi und prüfen Sie die Stromversorgung.
2. **Konfiguration prüfen** – in `config.json` lassen sich beispielsweise
   Bildanzahl (`PANO.IMGCOUNT`) oder LiDAR-Auflösung (`LIDAR.TARGET_RES`)
   anpassen.
3. **Programm starten** – in der virtuellen Umgebung einfach den
   Hauptbefehl ausführen:

   ```bash
   python PiLiDAR.py
   ```

4. **Scan beobachten** – die Konsole erklärt jeden Schritt (Kamera,
   LiDAR, Speicherung).  Nach Abschluss erscheint eine Zusammenfassung mit den
   wichtigsten Dateipfaden.

5. **Ergebnisse anschauen** – die wichtigsten Dateien finden Sie direkt im
   Ausgabeverzeichnis des Scans (siehe nächster Abschnitt).

## Ausgabeordner und Dateien

Jeder Lauf erzeugt einen Zeitstempel-Ordner in `scans/`, z. B. `scans/240901-1015/`.
Darunter befinden sich mehrere Unterordner:

| Ordner          | Inhalt                                                                    |
|-----------------|---------------------------------------------------------------------------|
| `raw/`          | Rohdaten des LiDAR (`*_lidar.pkl`) und die aufgenommenen Kamerabilder.    |
| `pointcloud/`   | Exportierte Punktwolken (`.ply`, zusätzlich gefilterte Varianten).        |
| `fusion/`       | Visualisierungen: LiDAR-Intensität als Panorama und Fusion mit dem Foto.  |
| `tmp/`          | Temporäre Dateien (z. B. Zwischenschritte von Hugin).                     |

Die Zusammenfassung am Ende des Programms nennt die wichtigsten Dateien noch
einmal explizit.

## Projektübersicht: Dateien & Skripte erklärt

Dieser Abschnitt fasst die wichtigsten Komponenten in einfachen Worten
zusammen.

| Datei/Ordner              | Erklärung                                                                 |
|---------------------------|---------------------------------------------------------------------------|
| `PiLiDAR.py`              | Hauptprogramm.  Steuert Kamera, LiDAR, Stepper oder startet die Simulation.|
| `config.json`             | Zentrale Einstellungen (Auflösung, Anzahl Fotos, Simulation, Verzeichnisse).|
| `lib/`                    | Python-Module mit den einzelnen Bausteinen (Treiber, Punktwolkenlogik).   |
| `lib/a4988_driver.py`     | Steuerung des Schrittmotors.  Enthält jetzt auch eine anschauliche Simulation.|
| `lib/lidar_driver.py`     | Kommuniziert mit dem STL27L-LiDAR und speichert Rohdaten.                 |
| `lib/pointcloud.py`       | Verarbeitet Rohdaten zu einer Punktwolke und erzeugt Visualisierungen.    |
| `lib/simulation.py`       | Erstellt Demo-Bilder und Demo-LiDAR-Daten, falls keine Hardware genutzt wird.|
| `lib/rpicam_utils.py`     | Hilfsfunktionen für die Raspberry-Pi-Kamera (HDR-Aufnahmen, Weißabgleich).|
| `lib/pano_utils.py`       | Startet Hugin zum Zusammensetzen der Panoramaaufnahme.                    |
| `requirements.txt`        | Liste aller benötigten Python-Pakete.                                    |
| `old/`                    | Archiv mit älteren Skripten und Tests.  Sie bleiben erhalten, beeinflussen aber den neuen Ablauf nicht.|
| `images/`, `hugin/`       | Beispielbilder bzw. Vorlagen für das Panorama-Stitching.                 |

## Hardware-Hinweise

* **Verkabelung:** Der LiDAR kommuniziert über UART.  TX/RX müssen mit den
  seriellen Pins des Raspberry Pi verbunden werden.  Der A4988 benötigt eine
  separate Motorstromversorgung.
* **Stromversorgung:** Kamera, LiDAR und Motor ziehen gemeinsam einige Ampere.
  Ein hochwertiges Netzteil verhindert Ausfälle während des Scans.
* **Mechanik:** Je gleichmäßiger der Aufbau rotiert, desto sauberer wird die
  resultierende Punktwolke.

## Fehlerbehebung

| Problem                                   | Lösungsansatz                                                     |
|-------------------------------------------|-------------------------------------------------------------------|
| `Permission denied` bei `/dev/ttyUSB0`    | Prüfen, ob der Benutzer in der Gruppe `dialout` ist (siehe Installation). |
| Hugin meldet fehlende Programme           | `sudo apt install hugin-tools` ausführen und den Scan erneut starten. |
| Programm startet Simulation unerwartet    | In der Konsole nach Warnungen suchen.  Meist fehlt Hardware oder ein Treiber. |
| Punktwolke fehlt                          | Sicherstellen, dass `ENABLE_3D` in `config.json` auf `true` steht.        |

Viel Spaß beim Scannen!  Für Feedback oder Fragen lohnt sich ein Blick in die
Issues des Repositories.

