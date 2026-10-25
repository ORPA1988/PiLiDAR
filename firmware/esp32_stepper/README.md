# ESP32-Stepper-Firmware (Variante B)

ESP32 DevKit (ESP-WROOM-32, CP2102) + A4988 + NEMA17 + Planetengetriebe 3,714:1.
Protokoll und Verdrahtung: Kopf von `esp32_stepper.ino` bzw. `docs/UEBERGABE_variante_b.md`.
Host-Seite: `backend/hardware/stepper_esp32.py` (`config.json` → `STEPPER.BACKEND = "esp32"`).

## Flashen

`build/esp32_stepper_merged_0x0.bin` (Bootloader + Partitionen + App, nicht im Git) ab 0x0:

```bash
.venv/bin/esptool --port /dev/serial/by-id/<ESP32> --baud 460800 write-flash 0x0 firmware/esp32_stepper/build/esp32_stepper_merged_0x0.bin
```

Vorher den PiLiDAR-Dienst stoppen, falls er den ESP-Port belegt (`sudo systemctl stop pilidar`).

## Selbst bauen

arduino-cli, Core arduino-esp32 3.3.12, FQBN `esp32:esp32:esp32`:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --export-binaries firmware/esp32_stepper
```
