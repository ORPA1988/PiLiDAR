"""Tests für das ESP32-Stepper-Backend gegen die simulierte Firmware.

Aufruf:  python -m tests.test_esp32
"""

from __future__ import annotations

import time

from backend.hardware.mock_esp32 import MockEsp32Serial
from backend.hardware.stepper_esp32 import Esp32Stepper

GEAR = 1.0 + 38.0 / 14.0


def _make(mock: MockEsp32Serial, **kw) -> Esp32Stepper:
    kw.setdefault("gear_ratio", GEAR)
    kw.setdefault("move_speed_dps", 60.0)
    kw.setdefault("move_accel_dps2", 200.0)
    return Esp32Stepper(serial_factory=lambda: mock, **kw)


def _track_error(st: Esp32Stepper, mock: MockEsp32Serial, duration: float,
                 settle: float = 1.5) -> tuple[float, float]:
    errs = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        t = time.monotonic()
        a = st.get_angle_at(t)
        true = st.dir_sign * mock.true_steps_at(t) / st.steps_per_degree
        if t - t0 > settle:
            errs.append(abs(a - true))
        time.sleep(0.002)
    return max(errs), sum(errs) / len(errs)


def test_move_and_home():
    m = MockEsp32Serial()
    st = _make(m)
    try:
        assert st.connected
        st.move_angle(90)
        assert abs(st.get_current_angle() - 90) < 0.05, st.get_current_angle()
        st.move_steps(-100)
        exp = 90 - 100 / st.steps_per_degree
        assert abs(st.get_current_angle() - exp) < 0.05
        st.home()
        assert abs(st.get_current_angle()) < 1e-9
        assert m.pos == 0
    finally:
        st.close()
    assert not m.enabled, "close() muss den Treiber abschalten"
    print("OK  MOVE/home")


def test_mode_b_tracking():
    m = MockEsp32Serial(drift_ppm=60, latency_s=0.002, jitter_s=0.004)
    st = _make(m)
    try:
        st.start_continuous(1.67, 10.0, direction=1)
        emax, emean = _track_error(st, m, 6.0)
        st.stop_continuous(ramp_down=True)
        a_stop = st.get_current_angle()
        assert abs(a_stop - m.pos / st.steps_per_degree) < 0.05
        st.home()
    finally:
        st.close()
    assert emax < 0.05, f"Winkelfehler {emax:.4f}° >= 0.05°"
    print(f"OK  Modus-B-Winkel bei 1.67 °/s: max {emax:.4f}°, mittel {emean:.4f}°")


def test_micros_wrap():
    # Überlauf von micros() nach ~1 s
    m = MockEsp32Serial(micros_start=(1 << 32) - 1_000_000)
    st = _make(m)
    try:
        st.start_continuous(5.0, 20.0)
        emax, _ = _track_error(st, m, 3.5)
        st.stop_continuous()
    finally:
        st.close()
    assert emax < 0.1, emax
    print(f"OK  micros-Überlauf: max {emax:.4f}°")


def test_gear_and_direction():
    # kalibriertes Getriebe weicht von der Firmware ab, Drehrichtung invertiert
    m = MockEsp32Serial()
    st = _make(m, gear_ratio=3.70, dir_invert=True)
    try:
        st.move_angle(45)
        assert abs(st.get_current_angle() - 45) < 0.05, st.get_current_angle()
        exp_steps = -round(45 * st.steps_per_degree)
        assert abs(m.pos - exp_steps) <= 1, (m.pos, exp_steps)
        st.home()
        assert m.pos == 0
    finally:
        st.close()
    print("OK  Getriebe-Umrechnung und DIR_INVERT")


def test_release():
    m = MockEsp32Serial()
    st = _make(m)
    try:
        st.move_angle(10)
        assert m.enabled
        st.release()
        assert not m.enabled, "release() muss EN 0 senden"
        assert abs(st.get_current_angle() - 10) < 0.05, "Position muss erhalten bleiben"
        st.home()                       # nächste Bewegung schaltet wieder ein
        assert m.enabled and m.pos == 0
    finally:
        st.close()
    print("OK  release(): Treiber aus, Position bleibt, Wiedereinschalten bei Bewegung")


def test_not_connected():
    st = Esp32Stepper(port="/dev/serial/by-id/gibt-es-nicht", gear_ratio=GEAR)
    try:
        assert not st.connected
        assert st.get_current_angle() == 0.0
        st.stop_continuous()            # darf nicht werfen
        try:
            st.move_angle(1)
        except RuntimeError:
            pass
        else:
            raise AssertionError("move_angle ohne Verbindung muss scheitern")
    finally:
        st.close()
    print("OK  ohne ESP32: Server startet, Bewegung wird abgelehnt")


if __name__ == "__main__":
    test_move_and_home()
    test_mode_b_tracking()
    test_micros_wrap()
    test_gear_and_direction()
    test_release()
    test_not_connected()
    print("\nAlle ESP32-Tests bestanden.")
