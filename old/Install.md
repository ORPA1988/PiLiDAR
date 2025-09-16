# Installation auf dem Raspberry Pi 5

Diese Anleitung beschreibt die Einrichtung von PiLiDAR auf einem Raspberry Pi 5 mit PiOS 64bit.

## Voraussetzungen
* Raspberry Pi 5 (64bit PiOS)
* Internetzugang
* Benutzer mit sudo‑Rechten

## System aktualisieren
```bash
sudo apt update
sudo apt upgrade -y
```

## Benötigte Pakete installieren
```bash
sudo apt install -y git python3 python3-pip python3-venv \
    libatlas-base-dev libopenblas-dev liblapack-dev libhdf5-dev \
    rpi-lgpio
```

Falls `python3-rpi.gpio` bereits installiert ist, sollte es entfernt und
durch `python3-rpi-lgpio` ersetzt werden:

```bash
sudo apt remove python3-rpi.gpio
sudo apt install python3-rpi-lgpio
```

## Projekt herunterladen
```bash
git clone https://github.com/ORPA1988/PiLiDAR.git
cd PiLiDAR
```

## Virtuelle Umgebung erstellen
```bash
python3 -m venv venv
source venv/bin/activate
```

## Python-Abhängigkeiten installieren
```bash
pip install --upgrade pip
pip install -r requirements.txt
# Falls die Installation von `open3d` fehlschlägt, kann stattdessen das
# vorgebaute Rad `open3d-cpu` verwendet werden:
# `pip install open3d-cpu`
```

## Programm starten
```bash
python PiLiDAR.py
```

Zum erneuten Aktivieren der Umgebung nach einem Neustart:
```bash
cd ~/PiLiDAR
source venv/bin/activate
```

## Autostart von `gpio_interrupt`
Mache das Skript ausführbar:
```bash
chmod +x gpio_interrupt.py
```

Erstelle eine neue Systemd‑Service-Datei:
```bash
sudo nano /etc/systemd/system/pilidar.service
```
Mit folgendem Inhalt:
```
[Unit]
Description=PiLiDAR-Button
After=network.target

[Service]
Type=simple
User=pi
Environment=LG_WD=/tmp
ExecStart=/usr/bin/python3 /home/pi/PiLiDAR/gpio_interrupt.py
Restart=no

[Install]
WantedBy=multi-user.target
```

Dienst laden und aktivieren:
```bash
sudo systemctl daemon-reload
sudo systemctl enable pilidar.service
sudo systemctl start pilidar.service
```

Status prüfen (optional):
```bash
sudo systemctl status pilidar.service
```
