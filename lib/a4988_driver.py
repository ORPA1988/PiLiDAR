"""High level driver for the A4988 stepper controller.

The original project targeted a Raspberry Pi where :mod:`RPi.GPIO` is
available.  For teaching purposes we now ship a tiny fallback implementation
that mimics the behaviour of the GPIO module.  This means the code can be run
on any computer without crashing which is incredibly helpful for beginners and
for our automated tests inside the container environment.
"""

from __future__ import annotations

import os
from time import sleep
from typing import Iterable


# ``RPi.GPIO`` is only available on the Raspberry Pi.  When the import fails we
# fall back to a small mock that imitates the tiny subset of the API that our
# driver needs.  The mock simply prints actions so that users understand what
# would normally happen on the hardware.
try:  # pragma: no cover - behaviour depends on the runtime environment
    os.environ.setdefault("RPI_LGPIO_REVISION", "0xa020d3")
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_MESSAGE = "Using real RPi.GPIO interface"
except (ModuleNotFoundError, RuntimeError):  # pragma: no cover - executed off Pi

    class _MockGPIO:
        """Minimal stub mimicking the API of :mod:`RPi.GPIO`.

        The class exposes the same constants and functions as the real module
        but does not interact with any hardware.  Each function prints a short
        log message; this double serves as documentation for beginners.
        """

        BCM = "BCM"
        OUT = "OUT"

        @staticmethod
        def setwarnings(state: bool):
            print(f"[SIM] GPIO warnings set to {state}")

        @staticmethod
        def setmode(mode):
            print(f"[SIM] GPIO mode set to {mode}")

        @staticmethod
        def setup(pin, mode):
            print(f"[SIM] Configuring pin {pin} as {mode}")

        @staticmethod
        def output(pin, state):
            print(f"[SIM] Setting pin {pin} to {state}")

        @staticmethod
        def cleanup(pins: Iterable[int] | int):
            print(f"[SIM] Cleaning up pins {pins}")

    GPIO = _MockGPIO()  # type: ignore
    _GPIO_MESSAGE = "Using simulated GPIO interface"


class A4988:
    """Simple wrapper around the A4988 stepper motor driver board.

    Even though the class works with a physical motor, the public interface is
    intentionally high level.  It exposes **angles** instead of raw step counts
    which makes the code that orchestrates the scan much easier to understand.
    In simulation mode we keep track of the state internally so that the rest
    of the project can continue to operate without hardware.
    """

    def __init__(
        self,
        dir_pin: int,
        step_pin: int,
        ms_pins: Iterable[int],
        *,
        delay: float = 0.001,
        step_angle: float = 1.8,
        microsteps: int = 16,
        gear_ratio: float = 1.0,
        verbose: bool = False,
    ) -> None:
        """Create a new controller instance.

        Parameters
        ----------
        dir_pin, step_pin, ms_pins:
            The Raspberry Pi pins connected to the driver.  They are ignored in
            simulation mode but we keep them to document the wiring.
        delay:
            Pause (in seconds) between two micro steps.
        step_angle:
            Angle in degrees a full motor step corresponds to.
        microsteps:
            Microstepping setting of the driver board.  Valid values are
            1, 2, 4, 8 and 16.
        gear_ratio:
            Ratio between the motor shaft and the rotating sensor mount.
        verbose:
            When ``True`` every simulated GPIO call is printed which helps
            debugging custom wiring setups.
        """

        # Disable warnings and let the user know which backend is in use.
        GPIO.setwarnings(verbose)
        GPIO.setmode(GPIO.BCM)
        print(f"[A4988] {_GPIO_MESSAGE}")

        # Store configuration.
        self.dir_pin = dir_pin
        GPIO.setup(self.dir_pin, GPIO.OUT)

        self.step_pin = step_pin
        GPIO.setup(self.step_pin, GPIO.OUT)

        self.ms_pins = list(ms_pins)
        for pin in self.ms_pins:
            GPIO.setup(pin, GPIO.OUT)

        # Internal state that allows us to emulate the motor position.
        self.current_steps = 0

        self.step_angle = step_angle
        self.microsteps = microsteps
        self.gear_ratio = gear_ratio
        self.delay = delay

        # Look-up table configuring the microstepping pins.  Beginners do not
        # have to memorise the required binary states – the driver handles it.
        self.step_modes = {
            1: [False, False, False],
            2: [True, False, False],
            4: [False, True, False],
            8: [True, True, False],
            16: [True, True, True],
        }

        if self.microsteps in self.step_modes:
            for pin, state in zip(self.ms_pins, self.step_modes[self.microsteps]):
                GPIO.output(pin, state)

    # ------------------------------------------------------------------
    # Helper methods operating on angles instead of raw step counts.

    def set_direction(self, direction: bool) -> None:
        """Set the rotation direction of the motor."""

        GPIO.output(self.dir_pin, direction)

    def get_steps_for_angle(self, angle: float) -> int:
        """Translate an angle into micro step counts."""

        return int((angle / self.step_angle) * self.microsteps * self.gear_ratio)

    def get_angle_for_steps(self, steps: int) -> float:
        """Translate micro step counts into a rotation angle."""

        return (steps / (self.microsteps * self.gear_ratio)) * self.step_angle

    def step(self) -> None:
        """Trigger a single micro step on the driver."""

        GPIO.output(self.step_pin, True)
        sleep(0.0001)
        GPIO.output(self.step_pin, False)
        sleep(self.delay)

    def move_steps(self, steps: int) -> None:
        """Move the motor by ``steps`` micro steps."""

        direction = steps < 0
        steps = abs(int(steps))

        self.set_direction(direction)
        for _ in range(steps):
            self.step()

        # Track the current position so that :meth:`get_current_angle` works in
        # simulation mode as well.
        self.current_steps += steps if not direction else -steps

    def move_angle(self, angle: float) -> int:
        """Rotate by a certain angle and return the number of micro steps."""

        steps = self.get_steps_for_angle(abs(angle))
        self.move_steps(steps if angle >= 0 else -steps)
        return steps

    def move_to_angle(self, target_angle: float, mod: bool = True) -> None:
        """Rotate to an absolute angle.

        Parameters
        ----------
        target_angle:
            Desired angle in degrees.
        mod:
            If ``True`` the angle is wrapped into the ``0…360`` range.
        """

        if mod:
            target_angle %= 360
        current_angle = self.get_current_angle()
        angle_difference = target_angle - current_angle

        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360

        steps_difference = self.get_steps_for_angle(angle_difference)
        self.move_steps(steps_difference)

    def get_current_angle(self, mod: bool = True) -> float:
        """Return the current rotation angle."""

        current_angle = (
            self.current_steps / (self.microsteps * self.gear_ratio) * self.step_angle
        )
        if mod:
            current_angle %= 360
        return current_angle

    def close(self) -> None:
        """Release the GPIO pins used by the driver."""

        GPIO.cleanup(self.ms_pins)
        GPIO.cleanup(self.dir_pin)
        GPIO.cleanup(self.step_pin)


__all__ = ["A4988"]

