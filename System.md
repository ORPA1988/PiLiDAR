# Systemübersicht

Dieses Projekt verarbeitet LiDAR-Daten des STL27L Sensors in Kombination mit einer Schrittmotorsteuerung und optionaler Kamerafunktion. Die wichtigsten Programme und Bibliotheken im Repository werden im Folgenden beschrieben.

## Hauptskripte

### `PiLiDAR.py`
Startet einen kompletten Scanvorgang. Das Skript initialisiert den Schrittmotor, den STL27L‑LiDAR sowie optional die Kamera. Nach Abschluss werden Panorama‑Bilder zusammengesetzt und aus den Messdaten eine 3D‑Punktwolke erzeugt.

### `gpio_interrupt.py`
Überwacht einen Taster an GPIO17. Bei Betätigung wird `PiLiDAR.py` mit erhöhter Priorität gestartet.

### `process_3D.py`
Lädt gespeicherte Rohdaten und erzeugt daraus eine 3D‑Punktwolke. Dieses Skript dient zur Nachbearbeitung ohne erneuten Scan.

## Bibliotheken im Verzeichnis `lib`

### `config.py`
Lädt Einstellungen aus `config.json` und erzeugt Pfade für neue Scans. Enthält Hilfsfunktionen zum Formatieren von Werten.

### `a4988_driver.py`
Steuert den A4988‑Treiber für den Schrittmotor. Unterstützt Mikro­schrittbetrieb und Berechnung der aktuellen Winkelposition.

### `lidar_driver.py`
Kommuniziert über die serielle Schnittstelle mit dem STL27L‑Sensor. Die Rohpakete werden dekodiert und optional per PWM gesteuerte Motordrehzahl geregelt.

### `pointcloud.py`
Enthält Funktionen zum Zusammenfügen der 2D‑Messpunkte zu einer 3D‑Punktwolke sowie zum Speichern und Laden der Rohdaten.

### `pano_utils.py`
Automatisiert das Zusammenfügen der aufgenommenen Fotos mit Hugin zu einem Panorama.

### `rpicam_utils.py`
Hilfsfunktionen für die Raspberry‑Pi‑Kamera, z. B. Belichtungswerte ermitteln und HDR‑Aufnahmen erstellen.

### `file_utils.py`
Allgemeine Funktionen zum Erzeugen von Verzeichnissen oder Speichern von Dateien.

