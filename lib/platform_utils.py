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


def is_raspberry_pi_5():
    """
    Detect if running on Raspberry Pi 5.
    Returns True if Pi 5, False otherwise.
    """
    try:
        with open('/proc/device-tree/model', 'r', encoding='utf-8') as f:
            model = f.read()
            return 'Raspberry Pi 5' in model
    except (FileNotFoundError, IOError):
        return False


platform = get_platform()

if platform in ['Windows', 'Mac', 'Linux', 'RaspberryPi']:
    import numpy as np

elif platform in ['Pico', 'Pico W', 'Metro M7']:
    import board                    # type: ignore
    from ulab import numpy as np    # type: ignore
    

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
    """
    Initialize hardware PWM on Raspberry Pi.
    
    PWM channel mapping differs between Pi 4 and Pi 5:
    - Pi 4 and earlier: channel 0 -> GPIO18, channel 1 -> GPIO19
    - Pi 5: channel 0 -> GPIO12, channel 1 -> GPIO13, 
            channel 2 -> GPIO18, channel 3 -> GPIO19
    
    For GPIO18 (commonly used for LiDAR PWM):
    - Pi 4: use pwm_channel=0
    - Pi 5: use pwm_channel=2
    """
    from rpi_hardware_pwm import HardwarePWM   # type: ignore
    
    # Adjust PWM channel for Raspberry Pi 5
    # On Pi 5, GPIO18 is on channel 2 (not channel 0 like on Pi 4)
    if is_raspberry_pi_5():
        # Map old channel numbers to Pi 5 channel numbers for GPIO18/19
        pi5_channel_map = {0: 2, 1: 3}  # GPIO18 -> ch2, GPIO19 -> ch3
        pwm_channel = pi5_channel_map.get(pwm_channel, pwm_channel)
    
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
