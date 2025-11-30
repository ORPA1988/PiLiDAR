"""
GPIO interrupt handler for PiLiDAR button.

Raspberry Pi 5 Compatibility:
Uses gpiozero library which works on both Pi 4 and Pi 5.
gpiozero uses lgpio as backend on Pi 5 for proper GPIO access.
"""
import subprocess
import time
import signal
import sys

from gpiozero import Button  # type: ignore


# Define the callback function that runs when the button is pressed
def start_callback():
    
    # # ENABLE USB POWER
    # subprocess.run(["sudo", "uhubctl", "-l", "1-1", "-a", "on"])
    
    # Check if there is an existing process
    global process
    if process is not None:
        return_code = process.poll()
        # If the return code is None, the process is still running
        if return_code is None:
            print("Process is still running, skipping button press")
            return
        else:
            print("Process has finished, return code:", return_code)
            
    # Start a new process with the main script with higher priority
    nice_value = -10
    call = ["nice", "-n", str(nice_value), "python3", MAIN_SCRIPT] if CONDA_ENV is None else ["nice", "-n", str(nice_value), CONDA_PATH, "run", "-n", CONDA_ENV, "python", MAIN_SCRIPT]
    process = subprocess.Popen(call)
    print("Started a new process, pid:", process.pid)


START_PIN = 17

CONDA_PATH  = "/home/pi/miniforge3/condabin/conda"
CONDA_ENV   = "py311"
MAIN_SCRIPT = "/home/pi/PiLiDAR/PiLiDAR.py"


# # DISABLE USB POWER
# subprocess.run(["sudo", "uhubctl", "-l", "1-1", "-a", "off"])


# Set up the button with gpiozero (compatible with Pi 4 and Pi 5)
# pull_up=True enables internal pull-up resistor, bounce_time handles debouncing
button = Button(START_PIN, pull_up=True, bounce_time=0.2)
button.when_pressed = start_callback


# Initialize the process variable
process = None


def signal_handler(sig, frame):
    """Handle cleanup on exit"""
    print("\nCleaning up...")
    button.close()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


# Keep the main thread running
print("Waiting for button press...")
signal.pause()
