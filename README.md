# PiLiDAR – 3D Scanner für den Raspberry Pi 5

Diese Version von **PiLiDAR** richtet sich explizit an den Raspberry Pi 5 und
setzt auf einen vollständig automatisierten Workflow für den STL27L LiDAR und
eine Pi Camera Module 2.  Die Software liest den LiDAR über USB aus, steuert den
Schrittmotor über einen A4988, synchronisiert pro Z-Schritt ein Foto und
erzeugt eine farbige Punktwolke im ``PLY``-Format.  Alle Verarbeitungsschritte
bauen auf der **Point Cloud Library (PCL)** auf.  Eine pybind11-Erweiterung
bindet die kompilierten PCL-Funktionen an Python; in Entwicklungsumgebungen ohne
PCL greift eine reine NumPy-Implementierung, damit Unit-Tests weiterhin laufen.

---

## 1. Hardwareübersicht

| Komponente | Zweck |
|------------|-------|
| Raspberry Pi 5 | Steuerrechner und Datenspeicher |
| STL27L LiDAR (USB, 921 600 Baud) | liefert 21 600 Messungen/s inkl. Intensität |
| Pi Camera Module 2 mit Fischaugenoptik | Farbaufnahme pro Z-Schritt |
| Z-Achse mit identischer Getriebeübersetzung wie im ursprünglichen Projekt | reproduzierbare Drehung |
| A4988 Treiber | Mikrostepping für den Schrittmotor |

Die Stromversorgung des LiDAR lässt sich per GPIO zuschalten, wodurch der Sensor
softwareseitig abgeschaltet werden kann.  Die Motorsteuerung erfolgt mit einem
DMA-fähigen GPIO-Treiber (``rpi-lgpio``) – ``pigpio`` wird auf dem Pi 5 nicht
mehr verwendet.

---

## 2. Installation

### 2.1 Betriebssystem vorbereiten

1. Raspberry Pi OS (64‑bit) mit aktiviertem Kamera-Stack installieren.
2. ``sudo raspi-config`` ausführen und die Kamera aktivieren.
3. Seriellen Zugriff freischalten:

   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod a+rw /dev/ttyUSB0   # Anpassung, falls der LiDAR an anderem Port hängt
   ```

### 2.2 Repository und Python-Umgebung

```bash
git clone https://github.com/<dein-account>/PiLiDAR.git
cd PiLiDAR
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 2.3 PCL-Anbindung kompilieren

Auf dem Pi wird die native Erweiterung nur einmal gebaut.  Sie verbindet die
Python-Schicht mit PCL:

```bash
sudo apt install libpcl-dev pybind11-dev
cmake -S cpp -B build -DPYTHON_EXECUTABLE=$(which python)
cmake --build build --target pilidar_pcl
cp build/pilidar_pcl$(python -c 'import sysconfig;print(sysconfig.get_config_var("EXT_SUFFIX"))') lib/
```

Fehlt die Erweiterung, nutzt die Software automatisch den NumPy-basierten
Fallback.  Dadurch bleibt die Entwicklung auf Nicht-Pi-Systemen möglich.

---

## 3. Konfiguration

Alle Einstellungen finden sich in ``config.json``.  Wichtige Werte:

* ``ENABLE_LIDAR`` / ``ENABLE_CAM`` / ``ENABLE_3D``: steuern die Pipeline.
* ``STEPPER``: Pins, Mikrostepping und Verzögerungen für den A4988.
* ``3D``: Skalierung, Z‑Offset sowie Dateiendung (``ply`` oder ``pcd``).
* ``VERTEXCOLOUR``: Parameter zur Farbprojektion des Panoramas.

Vor jedem Scan erzeugt die Software automatisch eine Ordnerstruktur unter
``scans/<SCAN-ID>/`` mit Unterordnern für Bilder, Logs und Rohdaten.  Die
Ordnernamen lassen sich über die GUI oder die Kommandozeile beeinflussen.

---

## 4. Workflow

1. **Scan starten** – via GUI (`python PiLiDAR.py --gui`) oder direkt auf der
   Kommandozeile (`python PiLiDAR.py`).
2. **LiDAR aktivieren** – der Controller schaltet die Versorgung und den Motor
   ein.  Die Stromabschaltung erfolgt ebenfalls softwareseitig.
3. **Kameraschuss je Z-Schritt** – nach jedem Stepper-Schritt löst die Kamera
   genau ein Foto aus, das später für die Farbzuordnung genutzt wird.
4. **Rohdaten erfassen** – LiDAR-Pakete werden mit Zeitstempel, Winkel,
   Distanz, Intensität und aktuellem Z-Winkel gespeichert.
5. **Punktwolke berechnen** – die PCL-Backend-Funktionen erzeugen eine farbige
   Punktwolke inkl. Intensitätskanal.
6. **Ablage** – alle Ergebnisse werden in einem Scan-Ordner gesichert:

   ```
   scans/<SCAN-ID>/
   ├── img/                      # Fotos der Pi-Cam
   ├── tmp/                      # temporäre HDR-Dateien
   ├── logs/                     # JSON-Log & Plausibilitätscheck
   ├── <SCAN-ID>_lidar.pkl       # Rohdaten (NumPy serialisiert)
   ├── <SCAN-ID>_intensity.ply   # Punktwolke mit Intensitätsfärbung
   ├── <SCAN-ID>_vertex.ply      # Punktwolke mit Kamerafarben
   └── <SCAN-ID>_blended_fused.jpg
   ```

Die erzeugte ``PLY``-Datei enthält XYZ, RGB und einen Intensitätswert pro
Punkt.  Für ``PCD``-Ausgaben muss lediglich die Dateiendung in ``config.json``
angepasst werden.

---

## 5. Plausibilitätsprüfung

Während des Scans speichert der Controller folgende Kennzahlen:

* Start- und Endzeit des Scans sowie die Gesamtdauer.
* Gewünschte vs. tatsächlich gefahrene Schrittzahl.
* Liste aller Schrittkommandos inklusive Zeitstempel und aktuellem Z-Winkel.
* Anzahl der empfangenen LiDAR-Pakete.

Die Daten landen in ``logs/scan_summary.json``.  Eine zusätzliche Datei
``scans/scan_history.json`` speichert aggregierte Werte vergangener Läufe.
Beim Abschluss eines Scans wird automatisch geprüft, ob die Schrittzahl im
1 %-Toleranzfenster liegt und wie stark die Ergebnisse vom historischen Mittel
abweichen.  So lassen sich Ausreißer (z. B. verpasste Schritte) nachträglich
identifizieren.

---

## 6. PCL-gestützte Nachbearbeitung

* **Normalenabschätzung** – PCL berechnet Oberflächennormalen für alle Punkte.
* **Intensitätsfärbung** – Intensitäten werden in ein RGB-Farbschema
  (``viridis``) überführt.
* **Vertex-Farben** – Falls ein Panorama vorliegt, werden RGB-Werte über eine
  sphärische Projektion auf die Punkte gemappt.
* **Filter** – optionales Voxel-Downsampling und Radius-Outlier-Filter.

Alle Operationen laufen headless und benötigen keine Open3D-GUI.  Die Dateien
lassen sich anschließend in CloudCompare, MeshLab oder ROS2 integrieren.

---

## 7. Tipps für den Pi 5

* ``rpi-lgpio`` stellt stabile Pulsweiten für den Steppertreiber bereit.  Die
  Pins sind in ``config.json`` frei wählbar.
* Der USB-LiDAR sollte an einem aktiven Hub hängen, damit das stromlose
  Schalten zuverlässig funktioniert.
* Ein Kühler für die CPU verhindert Throttling bei langen Scans.

---

## 8. Tests & Entwicklung

Die Python-Unit-Tests laufen plattformunabhängig.  Dank der Fallback-Klassen in
``lib/pcl_bindings.py`` werden keine nativen Bibliotheken benötigt.  Auf dem Pi
empfiehlt es sich, die Tests nach jeder Anpassung auszuführen:

```bash
pytest
```

---

## 9. Lizenz

PiLiDAR steht unter der MIT-Lizenz (siehe ``LICENSE.md``).  Beiträge sind
willkommen – insbesondere Verbesserungen an der PCL-Erweiterung oder neue
Kalibrier-Workflows.

