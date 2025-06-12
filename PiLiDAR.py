from time import sleep
import os

from lib.lidar_driver import Lidar
from lib.a4988_driver import A4988
from lib.config import Config, format_value
from lib.pointcloud import process_raw, save_raw_scan, get_scan_dict
from lib.rpicam_utils import (
    take_HDR_photo,
    estimate_camera_parameters,
    # take_photo,
)
from lib.pano_utils import hugin_stitch


print(r'''
 ____  _ _     _ ____    _    ____
|  _ \(_) |   (_)  _ \  / \  |  _ \
| |_) | | |   | | | | |/ _ \ | |_) |
|  __/| | |___| | |_| / ___ \|  _ <
|_|   |_|_____|_|____/_/   \_\_| \_\
''')


config = Config()
config.init()
config.relay_on()

enable_cam = config.get("ENABLE_CAM")
enable_lidar = config.get("ENABLE_LIDAR")


# initialize stepper
stepper = A4988(config.get("STEPPER", "pins", "DIR_PIN"),
                config.get("STEPPER", "pins", "STEP_PIN"),
                config.get("STEPPER", "pins", "MS_PINS"),
                delay=config.get("STEPPER", "STEP_DELAY"),
                step_angle=config.get("STEPPER", "STEP_ANGLE"),
                microsteps=config.get("STEPPER", "MICROSTEPS"),
                gear_ratio=config.get("STEPPER", "GEAR_RATIO"))


# initialize lidar
if enable_lidar:
    lidar = Lidar(config, visualization=None)

    # callback function for lidar.read_loop()
    def move_steps_callback():
        stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)
        lidar.z_angle = stepper.get_current_angle()

        # # DEBUG: take photo at each step
        # imgpath = os.path.join(config.scan_dir, f"{format_value(lidar.z_angle, 2)}.jpg")
        # take_photo(path=imgpath,
        #            exposure_time=current_exposure_time,
        #            gain=current_gain,
        #            awbgains=current_awbgains,

    if not enable_cam:
        # wait for lidar to lock rotational speed
        sleep(2)

# MAIN
try:
    # 360° SHOOTING PHOTOS
    if enable_cam:
        # Calibrate camera using EXIF data
        # Optimize red/blue gains for custom AWB
        print("Calibrating Camera...")
        current_exposure_time, current_gain, current_awbgains = estimate_camera_parameters(config)
        # print("[RESULT] AE:", current_exposure_time, "| Gain:", current_gain, "| AWB R:", round(current_awbgains[0],3), "B:", round(current_awbgains[1],3))

        IMGCOUNT = config.get("PANO", "IMGCOUNT")
        for i in range(IMGCOUNT):
            lidar.z_angle = stepper.get_current_angle()
            formatted_angle = format_value(lidar.z_angle, config.get("ANGULAR_DIGITS"))

            msg = f"\nTaking photo {i+1}/{IMGCOUNT} | Angle: {formatted_angle}"
            print(msg)
            imgname = f"image_{formatted_angle}.jpg"
            imgpath = os.path.join(config.img_dir, imgname)

            # take HDR photo
            imgpaths = take_HDR_photo(
                AEB=config.get("CAM", "AEB"),
                AEB_stops=config.get("CAM", "AEB_STOPS"),
                path=imgpath,
                exposure_time=current_exposure_time,
                gain=current_gain,
                awbgains=current_awbgains,
                denoise=config.get("CAM", "denoise"),
                sharpness=config.get("CAM", "sharpness"),
                saturation=config.get("CAM", "saturation"),
                save_raw=config.get("CAM", "raw"),
                blocking=True,
            )

            config.imglist.extend(imgpaths)

            # rotate stepper to next photo angle
            stepper.move_to_angle((360/IMGCOUNT) * (i+1))
            sleep(0.5)
        stepper.move_to_angle(0)
        sleep(0.5)

    # 180° SCAN
    if enable_lidar:
        print("\nLIDAR STARTED...\n")
        lidar.read_loop(callback=move_steps_callback, max_packages=config.max_packages)

        stepper.move_to_angle(0)   # return to 0°

        # Save raw_scan to pickle file
        raw_scan = get_scan_dict(
            lidar.z_angles, cartesian_list=lidar.cartesian_list
        )
        save_raw_scan(lidar.raw_path, raw_scan)

finally:
    print("\nPiLiDAR STOPPED\n")
    if enable_lidar:
        lidar.close()
    stepper.close()
    config.relay_off()
# STITCHING PROCESS
if enable_cam:
    print("\nStitching Pano...")
    project_path = hugin_stitch(config)
# 3D PROCESSING
if config.get("ENABLE_3D"):
    print("\nProcessing 3D Point Cloud...")
    pcd = process_raw(config, save=True)
