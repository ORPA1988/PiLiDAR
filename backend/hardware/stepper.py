"""Schrittmotor-Steuerung (A4988 / TMC2209 im STEP/DIR-Modus).

Zwei Messtechniken:
  * Modus A (stepwise): klassisches STEP/DIR-Pulsen – kompatibel zum Original.
  * Modus B (continuous): konstante Drehung über Hardware-PWM am STEP-Pin
    (GPIO19 = PWM-Kanal 1). Ruckelfrei, CPU-frei. Der Drehwinkel wird über die
    Zeit integriert (inkl. Beschleunigungsrampe), sodass jeder LiDAR-Frame einen
    kontinuierlichen z_angle erhält.

Der Winkel-Nullpunkt ist der getrackte Ausgangszustand (open-loop). Nach jedem
Scan wird per home() in die Ausgangsposition zurückgefahren.
"""

from __future__ import annotations

import threading
import time

from .gpio_backend import make_backends


class Stepper:
    def __init__(
        self,
        dir_pin: int,
        step_pin: int,
        ms_pins: list[int],
        step_angle: float = 1.8,
        microsteps: int = 16,
        gear_ratio: float = 1.0,
        step_delay: float = 0.0005,
        pwm_channel: int = 1,
        force_mock: bool = False,
    ):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.ms_pins = ms_pins
        self.step_angle = step_angle
        self.microsteps = microsteps
        self.gear_ratio = gear_ratio
        self.step_delay = step_delay

        self.gpio, self.pwm = make_backends(pwm_channel, force_mock=force_mock)

        self.gpio.setup_output(self.dir_pin, 0)
        self.gpio.setup_output(self.step_pin, 0)
        for p in self.ms_pins:
            self.gpio.setup_output(p, 0)
        self._apply_microsteps()

        # Open-loop-Winkelverfolgung
        self._steps = 0                # Modus A: gezählte Mikroschritte
        self._cont_angle = 0.0         # Modus B: integrierter Winkel [°]
        self._angle_lock = threading.Lock()

        # Modus-B-Laufzeitzustand
        self._cont_running = False
        self._cont_dir_sign = 1
        self._target_dps = 0.0
        self._cur_dps = 0.0
        self._accel_dps2 = 30.0
        self._integrator: threading.Thread | None = None

    # ------------------------------------------------------------------
    @property
    def steps_per_degree(self) -> float:
        return (self.microsteps * self.gear_ratio) / self.step_angle

    def _apply_microsteps(self) -> None:
        modes = {1: (0, 0, 0), 2: (1, 0, 0), 4: (0, 1, 0), 8: (1, 1, 0), 16: (1, 1, 1)}
        bits = modes.get(self.microsteps, (1, 1, 1))
        for pin, b in zip(self.ms_pins, bits):
            self.gpio.write(pin, b)

    def get_current_angle(self, mod: bool = False) -> float:
        with self._angle_lock:
            if self._cont_running or self._cont_angle:
                angle = self._cont_angle
            else:
                angle = self._steps / self.steps_per_degree
        return angle % 360.0 if mod else angle

    # --- Modus A: stepwise --------------------------------------------
    def _set_dir(self, sign: int) -> None:
        # DIR low = positiv (CCW), high = negativ – wie im Original (steps<0 -> high)
        self.gpio.write(self.dir_pin, 1 if sign < 0 else 0)

    def _pulse(self) -> None:
        self.gpio.write(self.step_pin, 1)
        time.sleep(0.0001)
        self.gpio.write(self.step_pin, 0)
        time.sleep(self.step_delay)

    def move_steps(self, steps: int) -> None:
        sign = -1 if steps < 0 else 1
        self._set_dir(sign)
        for _ in range(abs(int(steps))):
            self._pulse()
        with self._angle_lock:
            self._steps += sign * abs(int(steps))

    def move_angle(self, angle_deg: float) -> None:
        steps = int(round(angle_deg * self.steps_per_degree))
        self.move_steps(steps)

    # --- Modus B: continuous (Hardware-PWM) ---------------------------
    def start_continuous(self, speed_dps: float, accel_dps2: float = 30.0,
                         direction: int = 1) -> None:
        """Konstante Drehung starten (mit linearer Beschleunigungsrampe)."""
        self._cont_dir_sign = 1 if direction >= 0 else -1
        self._set_dir(self._cont_dir_sign)
        self._target_dps = abs(speed_dps)
        self._accel_dps2 = max(1e-3, accel_dps2)
        self._cur_dps = 0.0
        self._cont_running = True
        self.pwm.start(self._freq_for(self._cur_dps))
        self._integrator = threading.Thread(target=self._integrate, daemon=True,
                                             name="StepperIntegrator")
        self._integrator.start()

    def _freq_for(self, dps: float) -> float:
        return max(1.0, abs(dps) * self.steps_per_degree)

    def _integrate(self) -> None:
        last = time.monotonic()
        dt_nominal = 0.005  # 200 Hz Integrator
        while self._cont_running:
            now = time.monotonic()
            dt = now - last
            last = now
            # Rampe Richtung Zielgeschwindigkeit
            if self._cur_dps < self._target_dps:
                self._cur_dps = min(self._target_dps, self._cur_dps + self._accel_dps2 * dt)
                self.pwm.change_frequency(self._freq_for(self._cur_dps))
            elif self._cur_dps > self._target_dps:
                self._cur_dps = max(self._target_dps, self._cur_dps - self._accel_dps2 * dt)
                self.pwm.change_frequency(self._freq_for(self._cur_dps))
            with self._angle_lock:
                self._cont_angle += self._cont_dir_sign * self._cur_dps * dt
            time.sleep(dt_nominal)

    def set_speed(self, speed_dps: float) -> None:
        self._target_dps = abs(speed_dps)

    def stop_continuous(self, ramp_down: bool = True) -> None:
        if ramp_down and self._cont_running:
            self._target_dps = 0.0
            # warten bis nahezu Stillstand
            for _ in range(400):
                if self._cur_dps <= 0.05:
                    break
                time.sleep(0.005)
        self._cont_running = False
        if self._integrator is not None:
            self._integrator.join(timeout=1.0)
            self._integrator = None
        self.pwm.stop()
        # integrierten Winkel in Schrittzähler überführen (für Modus-A-Homing)
        with self._angle_lock:
            self._steps = int(round(self._cont_angle * self.steps_per_degree))

    # ------------------------------------------------------------------
    def home(self) -> None:
        """Zurück in die Ausgangsposition (Winkel 0)."""
        if self._cont_running:
            self.stop_continuous(ramp_down=True)
        angle = self.get_current_angle()
        if abs(angle) > 1e-3:
            self.move_angle(-angle)
        with self._angle_lock:
            self._cont_angle = 0.0
            self._steps = 0

    def close(self) -> None:
        try:
            if self._cont_running:
                self.stop_continuous(ramp_down=False)
        finally:
            self.gpio.cleanup(self.ms_pins + [self.dir_pin, self.step_pin])
