import sys
import os
import subprocess


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
                platform =  'RaspberryPi'
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

if platform in ['Windows', 'Mac', 'Linux', 'RaspberryPi']:
    import numpy as np

elif platform in ['Pico', 'Pico W', 'Metro M7']:
    import board                    # type: ignore  # noqa: F401
    from ulab import numpy as np    # type: ignore  # noqa: F401
    

def boardpin(pin):
    board = sys.modules['board']
    return getattr(board, pin)

def init_serial(port='/dev/ttyUSB0', baudrate=230400):
    ''' USB: "/dev/ttyUSB0"  GPIO: "/dev/ttyS0" '''
    import serial
    return serial.Serial(port=port, baudrate=baudrate, timeout=1.0, bytesize=8, parity='N', stopbits=1)

def init_serial_MCU(pin='GP1', baudrate=230400):
    from busio import UART          # type: ignore
    return UART(None, boardpin(pin), baudrate=baudrate, bits=8, parity=None, stop=1)
    

def init_pwm_Pi(pwm_channel=0, frequency=30000):
    '''pwm_channel 0: GP18, pwm_channel 1: GP19'''
    from rpi_hardware_pwm import HardwarePWM   # type: ignore
    return HardwarePWM(pwm_channel=pwm_channel, hz=frequency, chip=0)

def init_pwm_MCU(pin="GP2", frequency=30000):
    from pwmio import PWMOut        # type: ignore
    return PWMOut(boardpin(pin), frequency=frequency)


# legacy code: allow access to serial port on Raspberry Pi
def allow_serial():
    if get_platform() == "RaspberryPi":
        # Use subprocess to allow serial communication on Raspberry Pi
        sudo_command = "sudo chmod a+rw /dev/ttyS0"
        process = subprocess.Popen(sudo_command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        return output, error
    else:
        print("[WARNING] platform is no Pi.")


if __name__ == "__main__":
    platform = get_platform()

    print("platform:", platform)
