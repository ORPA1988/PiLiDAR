"""Prüft, ob das Öffnen des ESP32-Ports einen Neustart auslöst (Position geht dann verloren).

Vorher den Dienst stoppen:  sudo systemctl stop pilidar
Aufruf:                     .venv/bin/python scripts/esp32_reset_probe.py
Danach:                     sudo systemctl start pilidar

Bleibt bei "keep" der Neustart aus, in config.json STEPPER.OPEN_MODE = "keep" setzen.
Nur der ESP32 muss am USB hängen; Motor und LiDAR werden nicht benutzt.
"""
import glob
import time

import serial

PORTS = glob.glob("/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB*")
if not PORTS:
    raise SystemExit("ESP32 (CP2102) nicht gefunden")


def probe(label: str, lines_on: bool) -> bool:
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORTS[0], 115200, 0.2
    s.dtr = lines_on
    s.rts = lines_on
    s.open()
    t, txt = time.time(), b""
    while time.time() - t < 1.5:
        txt += s.read(256)
    s.write(b"STATUS\n"); time.sleep(0.3)
    status = s.read(400).decode("ascii", "replace").strip().splitlines()
    s.close()
    restarted = b"bereit" in txt
    print(f"{label:28s} Neustart beim Öffnen: {'JA' if restarted else 'nein'} | {status[-1][:70] if status else '-'}")
    time.sleep(1.0)
    return restarted


probe("reset (DTR/RTS aus)", False)
keep = [probe(f"keep (DTR/RTS an) #{i}", True) for i in (1, 2)]
print("Empfehlung:", 'OPEN_MODE "keep"' if not any(keep) else 'OPEN_MODE "reset" lassen (Neustart auch bei keep)')
