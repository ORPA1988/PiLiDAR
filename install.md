Diese Schritt-für-Schritt-Anleitung hilft Anfängern, PiLiDAR auf einem Raspberry Pi 5 mit 8GB RAM und Pi OS 64bit zu installieren.

1. Voraussetzungen
Raspberry Pi 5 mit 8GB RAM
Pi OS 64bit (aktuelle Version empfohlen)
Internetverbindung
Grundkenntnisse im Umgang mit dem Terminal
2. System aktualisieren
Öffne das Terminal und führe folgende Befehle aus:

bash
sudo apt update
sudo apt upgrade -y
sudo reboot
3. Git und Python installieren
PiLiDAR benötigt Python und Git. Installiere diese mit:

bash
sudo apt install -y git python3 python3-pip python3-venv
4. Projekt herunterladen
Kopiere das Projekt auf deinen Raspberry Pi:

bash
git clone https://github.com/ORPA1988/PiLiDAR.git
cd PiLiDAR
5. Virtuelle Python-Umgebung erstellen
Erstelle und aktiviere eine virtuelle Umgebung:

bash
python3 -m venv venv
source venv/bin/activate
Das Terminal zeigt nun ein (venv) am Anfang der Zeile.

6. Abhängigkeiten installieren
Installiere alle benötigten Bibliotheken aus der requirements.txt:

bash
pip install --upgrade pip
pip install -r requirements.txt
7. Zusätzliche Systempakete (falls benötigt)
Falls Fehler bei der Installation auftreten, installiere diese Pakete:

bash
sudo apt install -y libatlas-base-dev libopenblas-dev liblapack-dev libhdf5-dev
8. Programm ausführen
Starte das Hauptprogramm (ersetze ggf. main.py durch das passende Skript):

bash
python main.py
9. Virtuelle Umgebung erneut aktivieren
Nach jedem Neustart oder Terminal-Neuöffnung:

bash
cd ~/PiLiDAR
source venv/bin/activate
10. Fehlerbehebung
Prüfe auf Fehlermeldungen im Terminal.
Stelle sicher, dass alle Schritte korrekt durchgeführt wurden.
Suche in den GitHub-Issues nach ähnlichen Problemen oder erstelle eine neue Anfrage.
11. Weitere Hilfe
Lies die README.md im Projekt.
Stelle Fragen im GitHub-Repository unter "Discussions" oder "Issues".
Viel Erfolg mit PiLiDAR auf deinem Raspberry Pi 5!
