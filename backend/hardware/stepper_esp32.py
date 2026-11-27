"""Schrittmotor über ESP32 per USB-Seriell (Variante B).

Der ESP32 erzeugt und zählt die Schritte selbst (Firmware
firmware/esp32_stepper/esp32_stepper.ino, 115200 Bd, Zeilenprotokoll). Diese
Klasse bietet dieselbe Schnittstelle wie hardware.stepper.Stepper, damit der
Controller unverändert bleibt.

Winkel für die LiDAR-Frames
---------------------------
get_current_angle() wird vom LiDAR-Thread für jedes Paket (~1800/s) aufgerufen
und darf den ESP nicht fragen. Der ESP meldet alle REPORT_MS eine Zeile
"P <micros> <schritte> <grad>". Daraus:
  * ESP-Zeit -> Host-Zeit (time.monotonic) per linearer Regression über die
    letzten Sekunden (Versatz + Gangabweichung, 32-bit-Überlauf von micros)
  * Position zum Abfragezeitpunkt = Geradenfit der jüngsten Meldungen,
    ausgewertet zur aktuellen (umgerechneten) ESP-Zeit.

Positionen werden intern in ESP-Mikroschritten geführt; Winkel ergeben sich mit
dem Host-Wert steps_per_degree (config: MICROSTEPS, GEAR_RATIO, STEP_ANGLE), so
dass ein kalibriertes Getriebeverhältnis wirkt, ohne die Firmware zu ändern.
"""

from __future__ import annotations

import glob
import logging
import os
import re
import threading
import time
from collections import deque
from typing import Callable, Optional

try:
    import serial  # pyserial
except Exception:  # pragma: no cover
    serial = None

log = logging.getLogger("pilidar.esp32")

FULL_STEPS = 200
FW_GEAR_DEFAULT = 1.0 + 38.0 / 14.0
WRAP = 1 << 32

_STATUS_RE = re.compile(
    r"en=(?P<en>\d)\s+ms=(?P<ms>\d+)\s+mode=(?P<mode>\w+)\s+pos=(?P<pos>-?\d+).*?"
    r"steps/rev=(?P<spr>[\d.]+)")


def resolve_port(port: str, exclude: tuple[str, ...] = ()) -> Optional[str]:
    """by-id-Pfad bestimmen. "auto": einziges serielles Gerät, das nicht in
    exclude steht (z. B. der LiDAR)."""
    if port and port.lower() != "auto":
        return port
    excl = set()
    for p in exclude:
        if p:
            try:
                excl.add(os.path.realpath(p))
            except OSError:
                pass
    cands = [p for p in sorted(glob.glob("/dev/serial/by-id/*"))
             if os.path.realpath(p) not in excl]
    return cands[0] if len(cands) == 1 else None


class Esp32Stepper:
    def __init__(
        self,
        port: str = "auto",
        step_angle: float = 1.8,
        microsteps: int = 16,
        gear_ratio: float = 1.0,
        baudrate: int = 115200,
        move_speed_dps: float = 20.0,
        move_accel_dps2: float = 40.0,
        dir_invert: bool = False,
        report_ms: int = 20,
        exclude_ports: tuple[str, ...] = (),
        open_mode: str = "reset",
        serial_factory: Optional[Callable[[], object]] = None,
        sync_window_s: float = 10.0,
    ):
        self.port_cfg = port
        self.step_angle = step_angle
        self.microsteps = int(microsteps)
        self.gear_ratio = gear_ratio
        self.baudrate = baudrate
        self.move_speed_dps = move_speed_dps
        self.move_accel_dps2 = move_accel_dps2
        self.dir_sign = -1 if dir_invert else 1
        self.report_ms = int(report_ms)
        self.exclude_ports = tuple(exclude_ports)
        # "reset": DTR/RTS aus (bisher; ESP32-DevKit startet beim Öffnen neu -> Position unbekannt)
        # "keep":  DTR/RTS beide an lassen (Auto-Reset-Schaltung bleibt in Ruhe). Vor dem Umstellen
        #          mit scripts/esp32_reset_probe.py prüfen (Dienst vorher stoppen).
        self.open_mode = str(open_mode).lower()
        self._serial_factory = serial_factory
        self.port: Optional[str] = None

        self._ser = None
        self._reader: Optional[threading.Thread] = None
        self._cmd_lock = threading.RLock()
        self._replies: deque[tuple[str, int]] = deque()
        self._cv = threading.Condition()       # für Antworten und DONE
        self._done_seq = 0
        self._connected = False
        self._need_init = True
        self._closing = False

        # Firmware-Zustand (ESP-Schritte, ESP-Vorzeichen)
        self._fw_spd = FULL_STEPS * self.microsteps * FW_GEAR_DEFAULT / 360.0
        self._fw_enabled = False
        self._fw_moving = False                # MOVE/GOTO/RUN aktiv (bis DONE)
        self._running = False                  # Modus B aktiv
        self._run_sign = 1
        self._run_dps = 0.0
        self._sent_speed: Optional[float] = None
        self._sent_acc: Optional[float] = None

        # Positionsverfolgung
        self._pos_lock = threading.Lock()
        self._steps = 0                        # letzte bekannte ESP-Schritte
        # True nach einem ESP32-Neustart (auch beim Öffnen des Ports): Firmware
        # steht auf pos=0, der Kopf aber evtl. woanders. Scans erst nach
        # confirm_home() (Benutzer bestätigt Ausgangslage).
        self.position_lost = False
        self._samples: deque[tuple[float, float, int]] = deque()  # (esp_s, host_t, steps)
        self._sync_window = sync_window_s
        self._last_raw_us: Optional[int] = None
        self._wrap_us = 0
        self._sync_a = 1.0                     # host_t = a * esp_s + b
        self._sync_b = 0.0
        self._sync_ok = False

        self._supervisor = threading.Thread(target=self._supervise, daemon=True,
                                            name="Esp32Supervisor")
        try:
            self._connect()
        except Exception as exc:
            log.warning("ESP32 nicht verbunden: %s", exc)
        self._supervisor.start()

    # ------------------------------------------------------------------
    @property
    def steps_per_degree(self) -> float:
        return (self.microsteps * self.gear_ratio) / self.step_angle

    @property
    def connected(self) -> bool:
        return self._connected

    # --- Verbindung ---------------------------------------------------
    def _open_serial(self):
        if self._serial_factory is not None:
            return self._serial_factory()
        if serial is None:
            raise RuntimeError("pyserial nicht installiert")
        port = resolve_port(self.port_cfg, self.exclude_ports)
        if not port or not os.path.exists(port):
            raise RuntimeError(f"ESP32-Port nicht gefunden ({self.port_cfg})")
        self.port = port
        s = serial.Serial()
        s.port = port
        s.baudrate = self.baudrate
        s.timeout = 0.2
        keep = self.open_mode == "keep"
        s.dtr = keep
        s.rts = keep
        s.open()
        return s

    def _connect(self) -> None:
        with self._cmd_lock:
            if self._connected:
                return
            ser = self._open_serial()
            self._ser = ser
            self._connected = True
            self._reset_sync()
            self._reader = threading.Thread(target=self._read_loop, args=(ser,),
                                            daemon=True, name="Esp32Reader")
            self._reader.start()
            time.sleep(0.3 if self._serial_factory is None else 0.0)
            self._init_device()

    def _disconnect(self) -> None:
        self._connected = False
        self._need_init = True
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        with self._cv:
            self._fw_moving = False
            self._running = False
            self._cv.notify_all()

    def _init_device(self) -> None:
        """Nach Verbindungsaufbau oder ESP-Neustart: Zustand abgleichen."""
        st = None
        for _ in range(3):
            try:
                st = self._cmd("STATUS", timeout=1.0)
                break
            except TimeoutError:
                continue
        if st is None:
            raise TimeoutError("ESP32 antwortet nicht auf STATUS")
        m = _STATUS_RE.search(st)
        if not m:
            raise RuntimeError(f"unerwartete STATUS-Antwort: {st!r}")
        if m["mode"] != "IDLE":
            self._cmd("STOP")
            self._wait_until_idle(10.0, assume_moving=True)
        if int(m["ms"]) != self.microsteps:
            self._cmd(f"MS {self.microsteps}")
            st = self._cmd("STATUS")
            m = _STATUS_RE.search(st)
        spr = float(m["spr"])
        exact = FULL_STEPS * int(m["ms"]) * FW_GEAR_DEFAULT
        # STATUS gibt nur eine Nachkommastelle aus -> exakten Wert nehmen, wenn passend
        self._fw_spd = (exact if abs(spr - exact) < 0.5 else spr) / 360.0
        self._fw_enabled = m["en"] == "1"
        with self._pos_lock:
            self._steps = int(m["pos"])
        self._sent_speed = self._sent_acc = None
        self._cmd(f"REP {self.report_ms}")
        self._need_init = False
        log.info("ESP32 verbunden (%s): %.1f Schritte/U Kopf (Firmware), pos=%d",
                 self.port or "mock", self._fw_spd * 360, int(m["pos"]))

    def _supervise(self) -> None:
        while not self._closing:
            time.sleep(1.0)
            if self._closing:
                break
            try:
                if not self._connected:
                    self._connect()
                elif self._need_init:
                    with self._cmd_lock:
                        self._init_device()
            except Exception as exc:
                log.debug("ESP32-Verbindung: %s", exc)
                if self._connected and self._need_init:
                    self._disconnect()

    def _ensure_ready(self) -> None:
        if not self._connected:
            self._connect()
        if self._need_init:
            with self._cmd_lock:
                self._init_device()
        if not self._fw_enabled:
            self._cmd("EN 1")
            self._fw_enabled = True

    # --- Lesen ----------------------------------------------------------
    def _read_loop(self, ser) -> None:
        while self._connected and self._ser is ser:
            try:
                raw = ser.readline()
            except Exception as exc:
                log.warning("ESP32-Lesefehler: %s", exc)
                if self._ser is ser:
                    self._disconnect()
                return
            if not raw:
                continue
            t = time.monotonic()
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self._handle_line(line, t)

    def _handle_line(self, line: str, t: float) -> None:
        if line.startswith("P "):
            parts = line.split()
            if len(parts) >= 3:
                try:
                    self._add_sample(int(parts[1]), int(parts[2]), t)
                except ValueError:
                    pass
            return
        if line.startswith("DONE"):
            try:
                deg = float(line.split()[1])
                with self._pos_lock:
                    self._steps = int(round(deg * self._fw_spd))
            except (IndexError, ValueError):
                pass
            with self._cv:
                self._done_seq += 1
                self._fw_moving = False
                self._running = False
                self._cv.notify_all()
            return
        if line.startswith("OK") or line.startswith("ERR"):
            with self._cv:
                self._replies.append((line, self._done_seq))
                self._cv.notify_all()
            return
        if "bereit" in line:  # Startmeldung -> ESP wurde neu gestartet
            log.warning("ESP32-Neustart erkannt – Position unbekannt, Ausgangslage bestätigen")
            self.position_lost = True
            with self._pos_lock:
                self._steps = 0
            self._reset_sync()
            self._fw_enabled = False
            self._need_init = True
            with self._cv:
                self._fw_moving = False
                self._running = False
                self._cv.notify_all()

    # --- Befehle ------------------------------------------------------
    def _cmd(self, text: str, timeout: float = 1.0) -> str:
        return self._cmd_seq(text, timeout)[0]

    def _cmd_seq(self, text: str, timeout: float = 1.0) -> tuple[str, int]:
        """Befehl senden, auf OK/ERR warten. Liefert (Antwort, DONE-Zähler zum
        Zeitpunkt der Antwort)."""
        with self._cmd_lock:
            ser = self._ser
            if ser is None:
                raise RuntimeError("ESP32 nicht verbunden")
            with self._cv:
                self._replies.clear()
            try:
                ser.write((text + "\n").encode())
            except Exception as exc:
                self._disconnect()
                raise RuntimeError(f"ESP32 Schreibfehler: {exc}") from exc
            deadline = time.monotonic() + timeout
            with self._cv:
                while not self._replies:
                    rest = deadline - time.monotonic()
                    if rest <= 0 or not self._connected:
                        raise TimeoutError(f"ESP32: keine Antwort auf {text!r}")
                    self._cv.wait(rest)
                line, seq = self._replies.popleft()
        if line.startswith("ERR"):
            raise RuntimeError(f"ESP32 {text!r}: {line}")
        return line, seq

    def _wait_done(self, after_seq: int, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._done_seq <= after_seq:
                rest = deadline - time.monotonic()
                if rest <= 0:
                    raise TimeoutError("ESP32: Bewegung nicht abgeschlossen (DONE fehlt)")
                if not self._connected:
                    raise RuntimeError("ESP32-Verbindung während der Bewegung verloren")
                self._cv.wait(min(rest, 0.5))

    def _wait_until_idle(self, timeout: float, assume_moving: bool = False) -> None:
        deadline = time.monotonic() + timeout
        with self._cv:
            if assume_moving:
                self._fw_moving = True
            while self._fw_moving and self._connected:
                rest = deadline - time.monotonic()
                if rest <= 0:
                    raise TimeoutError("ESP32: Motor kommt nicht zum Stillstand")
                self._cv.wait(min(rest, 0.5))

    def _set_motion_params(self, speed_dps: float, accel_dps2: float) -> None:
        """Host-°/s in Firmware-°/s umrechnen (abweichendes Getriebeverhältnis)."""
        k = self.steps_per_degree / self._fw_spd
        sp, ac = round(speed_dps * k, 6), round(accel_dps2 * k, 6)
        if sp != self._sent_speed:
            self._cmd(f"SPEED {sp:.6f}")
            self._sent_speed = sp
        if ac != self._sent_acc:
            self._cmd(f"ACC {ac:.6f}")
            self._sent_acc = ac

    def _move_fw_steps(self, fw_steps: int, absolute: bool) -> None:
        with self._cmd_lock:
            self._ensure_ready()
            if self._fw_moving:
                self._wait_until_idle(30.0)
            self._set_motion_params(self.move_speed_dps, self.move_accel_dps2)
            with self._pos_lock:
                cur = self._steps
            dist = abs(fw_steps - cur) if absolute else abs(fw_steps)
            if dist == 0:
                return
            cmd = "GOTO" if absolute else "MOVE"
            with self._cv:
                self._fw_moving = True
            try:
                _, seq = self._cmd_seq(f"{cmd} {fw_steps / self._fw_spd:.6f}")
            except Exception:
                with self._cv:
                    self._fw_moving = False
                raise
        # außerhalb des Befehls-Locks warten, damit STOP möglich bleibt
        v = self.move_speed_dps * self.steps_per_degree
        a = self.move_accel_dps2 * self.steps_per_degree
        t_est = dist / v + v / a
        self._wait_done(seq, timeout=t_est * 1.5 + 3.0)

    # --- Winkel -------------------------------------------------------
    def _reset_sync(self) -> None:
        with self._pos_lock:
            self._samples.clear()
            self._last_raw_us = None
            self._wrap_us = 0
            self._sync_ok = False

    def _add_sample(self, raw_us: int, steps: int, host_t: float) -> None:
        with self._pos_lock:
            if self._last_raw_us is not None:
                d = raw_us - self._last_raw_us
                if d < -(WRAP // 2):
                    self._wrap_us += WRAP          # micros-Überlauf
                elif d < 0:                         # Rücksprung: ESP neu gestartet
                    self._samples.clear()
                    self._wrap_us = 0
            self._last_raw_us = raw_us
            esp_s = (raw_us + self._wrap_us) * 1e-6
            self._samples.append((esp_s, host_t, steps))
            self._steps = steps
            while self._samples and esp_s - self._samples[0][0] > self._sync_window:
                self._samples.popleft()
            self._fit_clock()

    def _fit_clock(self) -> None:
        s = self._samples
        n = len(s)
        if n < 2 or s[-1][0] - s[0][0] < 0.2:
            e, h, _ = s[-1]
            self._sync_a, self._sync_b = 1.0, h - e
            self._sync_ok = n >= 1
            return
        e0, h0 = s[0][0], s[0][1]
        sx = sy = sxx = sxy = 0.0
        for e, h, _ in s:
            x, y = e - e0, h - h0
            sx += x
            sy += y
            sxx += x * x
            sxy += x * y
        den = n * sxx - sx * sx
        if den <= 0:
            return
        a = (n * sxy - sx * sy) / den
        if not 0.99 < a < 1.01:                     # unplausibel -> nur Versatz
            a = 1.0
        b0 = (sy - a * sx) / n
        self._sync_a = a
        self._sync_b = h0 + b0 - a * e0
        self._sync_ok = True

    def _fw_steps_at(self, t: float) -> float:
        with self._pos_lock:
            s = self._samples
            if not self._fw_moving or not self._sync_ok or not s:
                return float(self._steps)
            esp_now = (t - self._sync_b) / self._sync_a
            last_e, _, last_p = s[-1]
            # höchstens 0,2 s über die letzte Meldung hinaus fortschreiben
            esp_now = min(esp_now, last_e + 0.2)
            # Geradenfit über die jüngsten ~100 ms
            k = len(s) - 1
            i = k
            while i > 0 and last_e - s[i - 1][0] <= 0.1:
                i -= 1
            if k - i < 1:
                if k >= 1:
                    i = k - 1
                else:
                    return float(last_p)
            pts = [(s[j][0] - last_e, s[j][2]) for j in range(i, k + 1)]
        m = len(pts)
        mx = sum(p[0] for p in pts) / m
        my = sum(p[1] for p in pts) / m
        sxx = sum((p[0] - mx) ** 2 for p in pts)
        if sxx <= 0:
            return float(last_p)
        v = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
        return my + v * (esp_now - last_e - mx)

    def get_angle_at(self, t: float) -> float:
        return self.dir_sign * self._fw_steps_at(t) / self.steps_per_degree

    def get_current_angle(self, mod: bool = False) -> float:
        angle = self.get_angle_at(time.monotonic())
        return angle % 360.0 if mod else angle

    # --- Modus A ------------------------------------------------------
    def move_steps(self, steps: int) -> None:
        """Host-Mikroschritte relativ (Host- und ESP-Teilung sind gleich)."""
        self._move_fw_steps(self.dir_sign * int(steps), absolute=False)

    def move_angle(self, angle_deg: float) -> None:
        steps = int(round(angle_deg * self.steps_per_degree))
        self.move_steps(steps)

    # --- Modus B ------------------------------------------------------
    def _send_run(self) -> None:
        k = self.steps_per_degree / self._fw_spd
        fw_dps = self.dir_sign * self._run_sign * self._run_dps * k
        self._cmd(f"RUN {fw_dps:.6f}")

    def start_continuous(self, speed_dps: float, accel_dps2: float = 30.0,
                         direction: int = 1) -> None:
        with self._cmd_lock:
            self._ensure_ready()
            if self._fw_moving:
                self._wait_until_idle(30.0)
            self._set_motion_params(self.move_speed_dps, max(1e-3, accel_dps2))
            self._run_sign = 1 if direction >= 0 else -1
            self._run_dps = abs(speed_dps)
            with self._cv:
                self._fw_moving = True
                self._running = True
            try:
                self._send_run()
            except Exception:
                with self._cv:
                    self._fw_moving = self._running = False
                raise

    def set_speed(self, speed_dps: float) -> None:
        with self._cmd_lock:
            self._run_dps = abs(speed_dps)
            if self._running:
                self._send_run()

    def stop_continuous(self, ramp_down: bool = True) -> None:
        """Anhalten – immer mit Rampe (Firmware). ramp_down=False wartet nur
        kurz auf den Stillstand."""
        if not self._connected or not self._fw_moving:
            with self._cv:
                self._running = False
            return
        with self._cmd_lock:
            _, seq = self._cmd_seq("STOP")
        acc = self._sent_acc or 1.0
        v = max(self._run_dps, self.move_speed_dps)
        timeout = (v / acc + 3.0) if ramp_down else 0.5
        try:
            self._wait_done(seq, timeout)
        except TimeoutError:
            if ramp_down:
                raise
        finally:
            with self._cv:
                self._running = False

    # ------------------------------------------------------------------
    def home(self) -> None:
        """Zurück auf Winkel 0 (ESP-Schritt 0)."""
        if self._fw_moving:
            self.stop_continuous(ramp_down=True)
        self._move_fw_steps(0, absolute=True)

    def confirm_home(self) -> None:
        """Benutzer bestätigt: Kopf steht in der Ausgangslage -> Position = 0."""
        with self._cmd_lock:
            self._ensure_ready_no_enable()
            if self._fw_moving:
                raise RuntimeError("Motor fährt noch")
            self._cmd("ZERO")
            with self._pos_lock:
                self._steps = 0
            self._reset_sync()
            self.position_lost = False

    def _ensure_ready_no_enable(self) -> None:
        if not self._connected:
            self._connect()
        if self._need_init:
            self._init_device()

    def release(self) -> None:
        """Treiber abschalten (EN 0): Motor stromlos, kein Fiepen/Erwärmen im
        Stillstand. Die Schrittposition bleibt in der Firmware erhalten; die
        nächste Bewegung schaltet über _ensure_ready() wieder ein."""
        with self._cmd_lock:
            if not self._connected or not self._fw_enabled or self._fw_moving:
                return
            self._cmd("EN 0")
            self._fw_enabled = False

    def close(self) -> None:
        self._closing = True
        try:
            if self._connected:
                if self._fw_moving:
                    try:
                        self.stop_continuous(ramp_down=True)
                    except Exception:
                        pass
                for c in ("REP 0", "EN 0"):
                    try:
                        self._cmd(c)
                    except Exception:
                        pass
                self._fw_enabled = False
        finally:
            self._disconnect()
