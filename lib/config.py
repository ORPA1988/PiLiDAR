'''
SCAN_ANGLE: # 180° CW or -180° CCW
GEAR_RATIO: 3.7142857 = 1 + 38/14
STEP_ANGLE: 1.8 = 360° / STEPPER_RES
TARGET_RES: 1/6 = 0.1666667°
OFFSET: 90° -> np.pi / 2
'''

import json
import os
import datetime
from typing import Optional
# import subprocess

try:
    from lib.file_utils import make_dir
    from lib.platform_utils import get_platform, allow_serial
except:
    from file_utils import make_dir
    from platform_utils import get_platform, allow_serial


platform = get_platform()
if platform == 'RaspberryPi':
    # os.environ['LG_WD'] = '/tmp'  # set LGPIO tmp directory
    import RPi.GPIO as GPIO  # type: ignore


class Config:
    def __init__(self, file_path: str = "config.json", scans_root: Optional[str] = None):
        self.base_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__))) # ../config.json

        # load config.json
        with open(os.path.join(self.base_dir, file_path), 'r') as file:
            self.dict = json.load(file)

        # set scans_root if provided, else use config.json
        if scans_root is not None:
            self.set(scans_root, "SCANS_ROOT")
        self.scans_root = os.path.join(self.base_dir, self.get("SCANS_ROOT"))
        
        # write hex strings back to dict:
        protocol = self.dict["LIDAR"]["protocol"]
        # start_byte
        start_byte = bytes.fromhex(protocol["start_byte"])
        self.set(start_byte, "LIDAR", "protocol", "start_byte")
        # dlength_byte
        dlength_byte = bytes.fromhex(protocol["dlength_byte"])
        self.set(dlength_byte, "LIDAR", "protocol", "dlength_byte")

        # CRC_TABLE
        crc_path = os.path.join(self.base_dir, self.get("LIDAR", "protocol", "CRC_PATH"))
        with open(crc_path, 'r') as file:
            crc_table = json.load(file)["CRC_TABLE"]
        self.crc_table = [int(hex_str, 16) for hex_str in crc_table]
        self.set(self.crc_table, "LIDAR", "protocol", "CRC_TABLE")
        
        
        self.platform = get_platform()
        self.DEVICE = self.get("LIDAR", "DEVICE")
        self.TARGET_SPEED = self.get("LIDAR", "TARGET_SPEED")

        self.SAMPLING_RATE = self.get("LIDAR", self.DEVICE, "SAMPLING_RATE")
        self.BAUDRATE = self.get("LIDAR", self.DEVICE , "BAUDRATE")
        self.PORT = self.get("LIDAR", self.DEVICE , "PORT")


        if self.platform == 'RaspberryPi':
            print("PiLiDAR on Raspberry Pi")

            # BUG legacy: allow access to serial port on Raspberry Pi
            allow_serial()

            self.gpio_setup()  # enable GPIO Ports

            self.PORT = self.get("LIDAR", self.DEVICE , "PORT")
            if not os.path.exists(self.PORT): 
                # if USB controller board is not connected, try serial port
                self.PORT = self.get("LIDAR", "GPIO_SERIAL", "PORT")
            
            # disable filtering on Raspberry Pi as it is computationally too expensive
            if not self.get("FILTERING", "FILTER_ON_PI"):
                self.set(False, "ENABLE_FILTERING")
        
        elif self.platform == 'Windows':
            self.PORT = self.get("LIDAR", self.DEVICE , "PORT_WIN")
            # disable GPIO-serial settings
            self.set(False, "ENABLE_GPIO_SERIAL")
        

        # STEPPER
        self.STEPPER_RES = self.get("STEPPER", "STEPPER_RES")
        self.MICROSTEPS = self.get("STEPPER", "MICROSTEPS")
        self.SCAN_ANGLE = self.get("STEPPER", "SCAN_ANGLE")

        step_angle = 360 / self.get("STEPPER", "STEPPER_RES")
        self.set(step_angle, "STEPPER", "STEP_ANGLE")

        self.gear_ratio = self.evaluate_formula(self.get("STEPPER", "GEAR_RATIO"))
        self.set(self.gear_ratio, "STEPPER", "GEAR_RATIO")

        self.target_res = self.evaluate_formula(self.get("LIDAR", "TARGET_RES"))
        self.update_target_res(self.target_res)


        # PANORAMA
        AEB = self.get("CAM", "AEB")
        AEB_STOPS = self.get("CAM", "AEB_STOPS")
        IMGCOUNT = self.get("PANO", "IMGCOUNT")
        TEMPLATE_DIR = os.path.join(self.base_dir, self.get("PANO", "TEMPLATE_DIR"))
        aeb_name = f"_AEB{AEB}-{AEB_STOPS}" if AEB > 0 else ""

        self.template_path = f"{TEMPLATE_DIR}/template_{IMGCOUNT}{aeb_name}.pto"


    def init(self, scan_id=None):
        ## TODO: fix power control issues
        ## RELAY
        # GPIO.output(relay_pin, 1)  # enable Power relay 
        ## USB HUB
        #bsubprocess.run(["sudo", "uhubctl", "-l", "1-1", "-a", "on"])  # enable USB Hub Power
        ## LIDAR
        # self.serial_connection.write(b'1')  # start STL27L motor
        # self.serial_connection.write(b'0')  # stop STL27L motor
        
        
        if scan_id is not None:
            self.scan_id = scan_id
        else:
            self.scan_id = datetime.datetime.now().strftime("%y%m%d-%H%M")

        self.scan_dir           = os.path.join(self.scans_root, self.scan_id)
        self.pano_path          = os.path.join(self.scan_dir, f'{self.scan_id}{self.get("PANO", "OUTPUT_NAME")}')
        self.raw_path           = os.path.join(self.scan_dir, f"{self.scan_id}{self.get('LIDAR', 'RAW_NAME')}")
        self.pcd_path           = os.path.join(self.scan_dir, f'{self.scan_id}.{self.get("3D", "EXT")}')            # .pcd, .ply, .xyz, .xyzrgb
        self.filtered_pcd_path  = os.path.join(self.scan_dir, f'{self.scan_id}_filtered.{self.get("3D", "EXT")}')

        self.img_dir   = make_dir(os.path.join(self.scan_dir, "img"))
        self.tmp_dir   = make_dir(os.path.join(self.scan_dir, "tmp"))
        
        self.imglist = []


    def get(self, *args):
        value = self.dict
        for key in args:
            value = value[key]
        return value
    
    def set(self, value, *args):
        d = self.dict
        for key in args[:-1]:
            d = d.setdefault(key, {})
        d[args[-1]] = value


    def update_target_res(self, target_res):
        self.target_res = target_res
        self.set(self.target_res, "LIDAR", "TARGET_RES")

        self.microsteps_per_revolution = self.STEPPER_RES * self.MICROSTEPS * self.gear_ratio   # 11886
        self.steps = int(round(self.microsteps_per_revolution * self.target_res / 360))         # 6
        self.h_res = 360 * self.steps / self.microsteps_per_revolution                          # 0.1817°
        self.horizontal_steps = int(self.SCAN_ANGLE / self.h_res)                               # 990
        self.packages_per_revolution = round(self.SAMPLING_RATE / (12 * self.TARGET_SPEED))     # 180
        self.max_packages = self.horizontal_steps * self.packages_per_revolution                # 178200


    def evaluate_formula(self, formula: str):
        return eval(formula)

    def gpio_setup(self, debug=False):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(debug)
        
        # power relay
        relay_pin = self.get("STEPPER", "RELAY_PIN")
        GPIO.setup(relay_pin, GPIO.OUT)


def get_scan_dict(z_angles, angular=None, cartesian=None, scan_id=None, device_id=None, sensor=None, hardware=None, location=None, author=None):
    raw_scan = {
        "header": {
            # "creation_date": None,
            # "modification_date": None,
            "scan_id": scan_id,
            "device_id": device_id,
            "sensor": sensor,
            "hardware": hardware,
            "location": location,
            "author": author,
            },
        "z_angles": z_angles,
        "angular": angular,
        "cartesian": cartesian,
    }
    return raw_scan


def format_value(value, digits):
    formatted_value = f"{round(value, digits):0{4 + digits}.{digits}f}"
    return formatted_value


if __name__ == "__main__":
    config = Config()
    config.init()

    print("init scan:", config.scan_id)
    # print(config.get("LIDAR", "protocol", "CRC_TABLE"))
