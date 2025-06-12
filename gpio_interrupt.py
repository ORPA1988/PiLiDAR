import subprocess
import time

# import os
# os.environ['LG_WD'] = '/tmp'  # export LG_WD=/tmp  # set LGPIO tmp directory

import os
os.environ.setdefault("RPI_LGPIO_REVISION", "0xa020d3")
import RPi.GPIO as GPIO  # type: ignore


# Define the callback function that runs when the button is pressed
def start_callback(channel):
    
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
    call = ["nice", "-n", str(nice_value), "python3", MAIN_SCRIPT]
    process = subprocess.Popen(call)
    print("Started a new process, pid:", process.pid)


START_PIN = 17

MAIN_SCRIPT = "/home/pi/PiLiDAR/PiLiDAR.py"


# # DISABLE USB POWER
# subprocess.run(["sudo", "uhubctl", "-l", "1-1", "-a", "off"])


# Set up the GPIO mode, pin direction and event detection
GPIO.setmode(GPIO.BCM)
GPIO.setup(START_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(START_PIN, GPIO.FALLING, callback=start_callback, bouncetime=200)


# Initialize the process variable
process = None

# Keep the main thread running
try:
    print("Waiting for button press...")
    while True:
        time.sleep(0.2)
        
except KeyboardInterrupt:
    GPIO.cleanup()
