"""Simulierter ESP32-Stepper (Protokoll wie firmware/esp32_stepper/esp32_stepper.ino).

Bildet die pyserial-Schnittstelle nach, die Esp32Stepper nutzt (readline, write,
reset_input_buffer, close). Ein Simulationsthread rechnet die Kopfbewegung mit
Trapezrampe in 1-ms-Schritten. Für Tests des Zeitabgleichs:
  * eigene ESP-Uhr (micros) mit Versatz, Gangabweichung und 32-bit-Überlauf
  * Übertragungslatenz mit Zufallsanteil je Zeile
  * true_steps_at(t) liefert die wahre Position zu einem Host-Zeitpunkt
"""

from __future__ import annotations

import bisect
import random
import threading
import time
from collections import deque

FULL_STEPS = 200
FW_GEAR = 1.0 + 38.0 / 14.0


class MockEsp32Serial:
    def __init__(self, micros_start: int = 123_456_789, drift_ppm: float = 40.0,
                 latency_s: float = 0.002, jitter_s: float = 0.003, seed: int = 1,
                 tick_s: float = 0.001):
        self._rng = random.Random(seed)
        self._t0 = time.monotonic()
        self._micros_start = micros_start
        self._drift = drift_ppm * 1e-6
        self._latency = latency_s
        self._jitter = jitter_s
        self._tick = tick_s

        # Firmware-Zustand
        self.ms = 16
        self.enabled = False
        self.speed_deg = 20.0
        self.acc_deg = 40.0
        self.mode = "IDLE"
        self.pos = 0.0           # Schritte (kontinuierlich)
        self.target = 0
        self.v = 0.0             # Schritte/s mit Vorzeichen
        self.run_v = 0.0
        self.rep_ms = 0
        self._last_rep = 0.0

        self._lock = threading.Lock()
        self._out: deque[tuple[float, bytes]] = deque()   # (lesbar ab, Zeile)
        self._cv = threading.Condition()
        self._rx = b""
        self._hist_t: list[float] = []
        self._hist_p: list[float] = []
        self.is_open = True
        self._emit("PiLiDAR-Stepper bereit. STATUS / EN 1 / MOVE 90 ...")
        self._thread = threading.Thread(target=self._sim, daemon=True, name="MockEsp32")
        self._thread.start()

    # --- Hilfen -------------------------------------------------------
    def steps_per_deg(self) -> float:
        return FULL_STEPS * self.ms * FW_GEAR / 360.0

    def micros(self, t: float | None = None) -> int:
        t = time.monotonic() if t is None else t
        return int(self._micros_start + (t - self._t0) * 1e6 * (1 + self._drift)) & 0xFFFFFFFF

    def _emit(self, text: str, t: float | None = None) -> None:
        t = time.monotonic() if t is None else t
        avail = t + self._latency + self._rng.random() * self._jitter
        with self._cv:
            # Reihenfolge bleibt erhalten (serielle Leitung)
            if self._out and self._out[-1][0] > avail:
                avail = self._out[-1][0]
            self._out.append((avail, (text + "\n").encode()))
            self._cv.notify_all()

    def true_steps_at(self, t: float) -> float:
        with self._lock:
            i = bisect.bisect_left(self._hist_t, t)
            if i <= 0:
                return self._hist_p[0] if self._hist_p else self.pos
            if i >= len(self._hist_t):
                return self._hist_p[-1]
            t0, t1 = self._hist_t[i - 1], self._hist_t[i]
            p0, p1 = self._hist_p[i - 1], self._hist_p[i]
            return p0 + (p1 - p0) * (t - t0) / (t1 - t0)

    # --- pyserial-Schnittstelle ----------------------------------------
    def write(self, data: bytes) -> int:
        self._rx += data
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            with self._lock:
                self._handle(line.decode(errors="replace").strip().upper())
        return len(data)

    def readline(self, size: int = -1) -> bytes:
        deadline = time.monotonic() + 0.2
        with self._cv:
            while self.is_open:
                now = time.monotonic()
                if self._out and self._out[0][0] <= now:
                    return self._out.popleft()[1]
                wait = deadline - now
                if self._out:
                    wait = min(wait, self._out[0][0] - now)
                if wait <= 0:
                    return b""
                self._cv.wait(wait)
        return b""

    def reset_input_buffer(self) -> None:
        with self._cv:
            self._out.clear()

    def close(self) -> None:
        self.is_open = False
        with self._cv:
            self._cv.notify_all()

    # --- Firmware-Befehle ---------------------------------------------
    def _handle(self, s: str) -> None:
        if not s:
            return
        cmd, _, arg = s.partition(" ")
        has = bool(arg)
        try:
            x = float(arg) if has else 0.0
        except ValueError:
            x = 0.0
        spd = self.steps_per_deg()
        if cmd == "STATUS":
            self._emit(f"OK en={int(self.enabled)} ms={self.ms} mode={self.mode} "
                       f"pos={int(round(self.pos))} deg={self.pos / spd:.3f} "
                       f"v={self.v / spd:.2f}°/s speed={self.speed_deg:.2f} "
                       f"acc={self.acc_deg:.2f} steps/rev={spd * 360:.1f}")
        elif cmd == "EN" and has:
            self.enabled = x != 0
            if not self.enabled:
                self.mode, self.v = "IDLE", 0.0
            self._emit(f"OK EN {int(self.enabled)}")
        elif cmd == "MS" and has:
            m = int(x)
            if m not in (1, 2, 4, 8, 16):
                self._emit("ERR MS 1|2|4|8|16")
            elif self.mode != "IDLE":
                self._emit("ERR erst STOP")
            else:
                d = self.pos / spd
                self.ms = m
                self.pos = float(round(d * self.steps_per_deg()))
                self._emit(f"OK MS {m} ({self.steps_per_deg() * 360:.1f} Schritte/U Kopf)")
        elif cmd == "SPEED" and has and x > 0:
            self.speed_deg = x
            self._emit(f"OK SPEED {x:.3f}")
        elif cmd == "ACC" and has and x > 0:
            self.acc_deg = x
            self._emit(f"OK ACC {x:.3f}")
        elif cmd in ("MOVE", "GOTO") and has:
            if not self.enabled:
                self._emit("ERR EN 1 fehlt")
                return
            if self.mode == "RUN":
                self._emit("ERR erst STOP")
                return
            t = int(round(x * spd))
            if cmd == "MOVE":
                t += self.target if self.mode == "MOVE" else int(round(self.pos))
            self.target, self.mode = t, "MOVE"
            self._emit(f"OK {cmd} -> {t / spd:.3f}")
        elif cmd == "RUN" and has:
            if not self.enabled:
                self._emit("ERR EN 1 fehlt")
                return
            if self.mode == "MOVE":
                self._emit("ERR erst STOP")
                return
            self.run_v, self.mode = x * spd, "RUN"
            self._emit(f"OK RUN {x:.3f}")
        elif cmd == "STOP":
            if self.mode == "MOVE":
                a = self.acc_deg * spd
                brake = self.v * self.v / (2 * a)
                self.target = int(round(self.pos + (brake if self.v >= 0 else -brake)))
            elif self.mode == "RUN":
                self.run_v = 0.0
            self._emit("OK STOP")
        elif cmd == "ZERO":
            if self.mode != "IDLE":
                self._emit("ERR erst STOP")
            else:
                self.pos = 0.0
                self.target = 0
                self._emit("OK ZERO")
        elif cmd == "REP" and has:
            self.rep_ms = int(x)
            self._emit(f"OK REP {self.rep_ms}")
        else:
            self._emit("ERR unbekannt: STATUS EN MS SPEED ACC MOVE GOTO RUN STOP ZERO REP")

    # --- Kinematik ----------------------------------------------------
    def _sim(self) -> None:
        last = time.monotonic()
        while self.is_open:
            time.sleep(self._tick)
            now = time.monotonic()
            dt = now - last
            last = now
            with self._lock:
                self._step(dt, now)
                self._hist_t.append(now)
                self._hist_p.append(self.pos)
                if len(self._hist_t) > 200_000:
                    del self._hist_t[:100_000]
                    del self._hist_p[:100_000]
                if self.rep_ms and (now - self._last_rep) * 1000 >= self.rep_ms:
                    self._last_rep = now
                    spd = self.steps_per_deg()
                    self._emit(f"P {self.micros(now)} {int(round(self.pos))} "
                               f"{self.pos / spd:.3f}", now)

    def _step(self, dt: float, now: float) -> None:
        if self.mode == "IDLE" or not self.enabled:
            return
        spd = self.steps_per_deg()
        a = self.acc_deg * spd
        if self.mode == "MOVE":
            rem = self.target - self.pos
            if abs(rem) < 0.5 and abs(self.v) <= a * dt * 2:
                self.pos = float(self.target)
                self._finish(now)
                return
            sign = 1.0 if rem > 0 else -1.0
            brake = self.v * self.v / (2 * a)
            vmax = self.speed_deg * spd
            if self.v * sign < 0 or abs(rem) > brake + 1:
                v_t = sign * vmax
            else:
                v_t = 0.0
            # Minimalgeschwindigkeit, damit das Ziel sicher erreicht wird
            dv = a * dt
            self.v = min(self.v + dv, v_t) if self.v < v_t else max(self.v - dv, v_t)
            if abs(self.v) < 2 * dv and abs(rem) >= 0.5:
                self.v = sign * 2 * dv
            new = self.pos + self.v * dt
            if (self.target - new) * sign < 0:   # nicht überschießen
                new = float(self.target)
            self.pos = new
        else:  # RUN
            v_t = self.run_v
            dv = a * dt
            self.v = min(self.v + dv, v_t) if self.v < v_t else max(self.v - dv, v_t)
            self.pos += self.v * dt
            if v_t == 0.0 and abs(self.v) <= dv:
                self.pos = float(round(self.pos))
                self._finish(now)

    def _finish(self, now: float) -> None:
        self.mode, self.v = "IDLE", 0.0
        self.target = int(round(self.pos))
        self._emit(f"DONE {self.pos / self.steps_per_deg():.3f}", now)
