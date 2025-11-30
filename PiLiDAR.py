from time import sleep
import os

from lib.lidar_driver import Lidar
from lib.a4988_driver import A4988
try:
    from lib.imu_driver import MPU6050Wrapper  # type: ignore
except Exception:
    MPU6050Wrapper = None
from lib.config import Config, format_value
from lib.raw_utils import save_raw_scan, get_scan_dict
from lib.pointcloud_numpy import process_raw as process_raw_numpy
from lib.rpicam_utils import take_HDR_photo, estimate_camera_parameters
from lib.pano_utils import hugin_stitch


print(r'''
 ____  _ _     _ ____    _    ____ 
|  _ \(_) |   (_)  _ \  / \  |  _ \ 
| |_) | | |   | | | | |/ _ \ | |_) | 
|  __/| | |___| | |_| / ___ \|  _ < 
|_|   |_|_____|_|____/_/   \_\_| \_\ 
''')


config = Config()
# Mitigate OpenBLAS munmap warnings by limiting threads on Pi
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
config.init()

enable_cam   = config.get("ENABLE_CAM")
enable_lidar = config.get("ENABLE_LIDAR")
enable_IMU   = config.get("ENABLE_IMU")

# Initialize optional variables
imu = None
quat_list = []
lidar = None

# initialize IMU (optional)
if enable_IMU and MPU6050Wrapper is None:
    print("IMU module not available; disabling IMU features.")
    enable_IMU = False
if enable_IMU and MPU6050Wrapper is not None:
    imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"), config.get("IMU", "frequency"))


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
    def move_steps_callback() -> None:
        stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)
        if lidar is not None:
            lidar.z_angle = stepper.get_current_angle()
        if enable_IMU and imu is not None:
            quat_values = imu.get_quat_values()
            quat_list.append(quat_values)
else:
    def move_steps_callback() -> None:
        pass
            
    if not enable_cam:
        # wait for lidar to lock rotational speed
        sleep(2)

# MAIN
try:
    # 360° SHOOTING PHOTOS
    if enable_cam:
        print("Calibrating Camera...")
        current_exposure_time, current_gain, current_awbgains = estimate_camera_parameters(config)

        IMGCOUNT = config.get("PANO", "IMGCOUNT")
        for i in range(IMGCOUNT):
            current_angle = stepper.get_current_angle()
            formatted_angle = format_value(current_angle, config.get("ANGULAR_DIGITS"))

            print(f"\nTaking photo {i+1}/{IMGCOUNT} | Angle: {formatted_angle}")            
            imgpath = os.path.join(config.img_dir, f"image_{formatted_angle}.jpg")
            
            if enable_IMU and imu is not None:
                euler = imu.get_euler_angles()
                print(f'\tEuler: x {format_value(euler.x, 2)} y {format_value(euler.y, 2)} z {format_value(euler.z, 2)}')

            # take HDR photo
            imgpaths = take_HDR_photo(AEB           = config.get("CAM", "AEB"), 
                                      AEB_stops     = config.get("CAM", "AEB_STOPS"),
                                      path          = imgpath, 
                                      exposure_time = current_exposure_time, 
                                      gain          = current_gain, 
                                      awbgains      = current_awbgains, 
                                      denoise       = config.get("CAM", "denoise"),
                                      sharpness     = config.get("CAM", "sharpness"),
                                      saturation    = config.get("CAM", "saturation"),
                                      save_raw      = config.get("CAM", "raw"), 
                                      blocking      = True)
            
            config.imglist.extend(imgpaths)

            # rotate stepper to next photo angle
            stepper.move_to_angle((360/IMGCOUNT) * (i+1))
            sleep(0.5)
        
        stepper.move_to_angle(0)
        stepper.move_steps(1)
        sleep(0.5)


    # 180° SCAN
    if enable_lidar:
        assert lidar is not None, "Lidar must be initialized"
        print("\nLIDAR STARTED...\n")
        lidar.read_loop(callback=move_steps_callback, max_packages=config.max_packages)
        
        stepper.move_to_angle(0)   # return to 0°

        # Save raw_scan to pickle file
        raw_scan = get_scan_dict(lidar.z_angles, cartesian_list=lidar.cartesian_list)

        if enable_IMU:
            # inject IMU data into raw_scan dict
            raw_scan["quaternions"] = quat_list

        save_raw_scan(lidar.raw_path, raw_scan)
    

finally:
    print("\nPiLiDAR STOPPED\n")
    if enable_lidar and lidar is not None:
        lidar.close()

    if enable_IMU and imu is not None:
        imu.close()

    stepper.close()
    
    if hasattr(config, 'relay_device'):
        config.close_gpio()


# STITCHING PROCESS
if enable_cam:
    print("\nStitching Pano...")
    project_path = hugin_stitch(config)


# 3D PROCESSING
if config.get("ENABLE_3D"):
    print("\nProcessing 3D Point Cloud...")
    # Force NumPy backend on Raspberry Pi 5 for stability and ASCII PLY output
    _ = process_raw_numpy(config, save=True)
