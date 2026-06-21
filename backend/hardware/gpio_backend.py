"""Dünne Hardware-Abstraktion für GPIO und Hardware-PWM.

Erlaubt Betrieb auf dem Pi (rpi-lgpio / RPi.GPIO + rpi-hardware-pwm) und einen
Mock-Modus für Tests am PC. Die Auswahl erfolgt automatisch per Import-Versuch;
mit force_mock=True kann der Mock erzwungen werden.
"""

from __future__ import annotations

from typing import Optional

OUT = "out"


# ----------------------------------------------------------------------
class _GpioBase:
    def setup_output(self, pin: int, initial: int = 0) -> None: ...
    def write(self, pin: int, value: int) -> None: ...
    def cleanup(self, pins) -> None: ...


class MockGpio(_GpioBase):
    def __init__(self):
        self.state: dict[int, int] = {}

    def setup_output(self, pin, initial=0):
        self.state[pin] = initial

    def write(self, pin, value):
        self.state[pin] = int(value)

    def cleanup(self, pins):
        for p in (pins if isinstance(pins, (list, tuple)) else [pins]):
            self.state.pop(p, None)


class RpiGpio(_GpioBase):  # pragma: no cover - nur auf dem Pi
    def __init__(self):
        import RPi.GPIO as GPIO  # rpi-lgpio stellt diese API bereit
        self.GPIO = GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

    def setup_output(self, pin, initial=0):
        self.GPIO.setup(pin, self.GPIO.OUT, initial=initial)

    def write(self, pin, value):
        self.GPIO.output(pin, bool(value))

    def cleanup(self, pins):
        self.GPIO.cleanup(pins)


# ----------------------------------------------------------------------
class _PwmBase:
    def start(self, freq_hz: float, duty: float = 50.0) -> None: ...
    def change_frequency(self, freq_hz: float) -> None: ...
    def stop(self) -> None: ...


class MockPwm(_PwmBase):
    def __init__(self, channel: int):
        self.channel = channel
        self.freq = 0.0
        self.running = False

    def start(self, freq_hz, duty=50.0):
        self.freq = max(0.0, freq_hz)
        self.running = True

    def change_frequency(self, freq_hz):
        self.freq = max(0.0, freq_hz)

    def stop(self):
        self.running = False
        self.freq = 0.0


class HardwarePwm(_PwmBase):  # pragma: no cover - nur auf dem Pi
    """Setzt dtoverlay=pwm-2chan voraus. Kanal 1 = GPIO19, Kanal 0 = GPIO18."""

    def __init__(self, channel: int):
        from rpi_hardware_pwm import HardwarePWM
        self.channel = channel
        self._cls = HardwarePWM
        self._pwm: Optional[object] = None

    def start(self, freq_hz, duty=50.0):
        freq_hz = max(1.0, freq_hz)
        self._pwm = self._cls(pwm_channel=self.channel, hz=freq_hz, chip=0)
        self._pwm.start(duty)

    def change_frequency(self, freq_hz):
        if self._pwm is not None:
            self._pwm.change_frequency(max(1.0, freq_hz))

    def stop(self):
        if self._pwm is not None:
            self._pwm.stop()
            self._pwm = None


# ----------------------------------------------------------------------
def make_backends(pwm_channel: int = 1, force_mock: bool = False):
    """Liefert (gpio, pwm) – echte Backends auf dem Pi, sonst Mock."""
    if force_mock:
        return MockGpio(), MockPwm(pwm_channel)
    try:  # pragma: no cover
        gpio = RpiGpio()
    except Exception:
        gpio = MockGpio()
    try:  # pragma: no cover
        pwm = HardwarePwm(pwm_channel)
    except Exception:
        pwm = MockPwm(pwm_channel)
    return gpio, pwm
