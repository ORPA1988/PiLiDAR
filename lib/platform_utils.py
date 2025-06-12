import sys
import os


def get_platform():
    platform = sys.platform
    # print("sys.platform:", platform)

    if "win32" in platform:
        platform = 'Windows'
    elif 'darwin' in platform:
        platform = 'Mac'
    else:
        machine = os.uname().machine
        # print("os.uname():",  os.uname())

        if 'linux' in platform:
            if 'aarch64' in machine:
                platform = 'RaspberryPi'
            else:
                platform = 'Linux'
        elif 'Pico W' in machine:
            # 'Raspberry Pi Pico W with rp2040'
            return 'Pico W'
        elif 'Pico' in machine:
            return 'Pico'
        elif 'MIMXRT1011' in machine:
            # 'Metro MIMXRT1011 with IMXRT1011DAE5A'
            return 'Metro M7'
    return platform


platform = get_platform()


def init_serial(port='/dev/ttyUSB0', baudrate=230400):
    ''' USB: "/dev/ttyUSB0"  GPIO: "/dev/ttyS0" '''
    import serial
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=1.0,
        bytesize=8,
        parity='N',
        stopbits=1,
    )


def allow_serial():
    """Try to set read/write permissions for common serial ports."""
    for dev in ("/dev/ttyUSB0", "/dev/ttyS0", "/dev/serial0"):
        if os.path.exists(dev):
            try:
                os.chmod(dev, 0o666)
            except PermissionError:
                print(f"Permission denied when changing permissions for {dev}")


if __name__ == "__main__":
    platform = get_platform()

    print("platform:", platform)
