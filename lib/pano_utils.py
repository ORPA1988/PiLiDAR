"""
http://paulbourke.net/dome/dualfish2sphere/diagram.pdf

Hugin CLI parameters from StereoPi.sh :
    pto_gen --projection=2 --fov=360 -o ./tmp/project.pto $1 $2
    pto_template --output=./tmp/project.pto --template=template.pto ./tmp/project.pto
    hugin_executor --stitching --prefix=$RESULT_NAME ./tmp/project.pto

additional parameters:
    pto_lensstack -o project1.pto --new-lens i1 project.pto
    cpfind -o project.pto --multirow project.pto
    cpclean -o project.pto project.pto
    linefind -o project.pto project.pto
    pto_var -o project.pto --opt TrX,TrY,TrZ,r,Eev,Ra,Rb,Rc,Rd,Re,!TrX0,!TrY0,!TrZ0,!r0,!Eev0,!Ra1,!Rb1,!Rc1,!Rd1,!Re1 project.pto
    autooptimiser -n -o project.pto project.pto
    pano_modify  --projection=1 --fov=AUTO --center --canvas=AUTO --crop=AUTO -o project.pto project.pto
"""

import math
import numpy as np
import subprocess
import os


def hugin_modify(pto_path, new_project_path, width=6800):
    # Load the project file(pto_path)
    with open(pto_path, 'r') as file:
        lines = file.readlines()

    # search for first line that starts with 'p ' -> 'p f2 w6000 h3000 v360 ...'
    for i, line in enumerate(lines):

        if line.startswith("p "):
            # replace w and h arguments
            words = line.split(" ")
            for j, word in enumerate(words):
                if word.startswith("w"):
                    words[j] = f"w{width}"
                elif word.startswith("h"):
                    words[j] = f"h{width//2}"

            lines[i] = " ".join(words)
            break
    
    # Save the project file at new_project_path
    with open(new_project_path, 'w') as file:
        file.writelines(lines)


def hugin_stitch(config, nice=19):
    pto_path = config.pto_path
    output_path  = os.path.join(config.scan_dir, f'{config.scan_id}.jpg')

    # create Hugin project
    subprocess.run(['pto_gen', '--projection=2', '--fov=360', '-o', pto_path, *config.imglist])  # *list -> unpack
    
    # apply template
    subprocess.run(['pto_template', f'--output={pto_path}', f'--template={config.template_path}', pto_path])

    # modify resolution of the temporary project file
    hugin_modify(pto_path, pto_path, width=config.get("PANO", "PANO_WIDTH"))

    # start stitching as non-blocking thread with low priority
    if config.get("ENABLE_PANO"):
        cmd_string = ['nice', '-n', str(nice), 'hugin_executor', '--stitching', f'--prefix={output_path}', pto_path]
        process = subprocess.Popen(cmd_string)

        # Wait for the process to finish
        process.wait()

        # check returncode if stitching was successful
        if process.returncode != 0:
            raise Exception(f"Command failed with return code {process.returncode}")

    return output_path


def hugin_refine(config, nice=19):
    pto_path = config.pto_path
    output_path =  os.path.join(config.scan_dir, f'{config.scan_id}.jpg')

    # Create Hugin project
    subprocess.run(['pto_gen', '--projection=2', '--fov=360', '-o', pto_path, *config.imglist], check=True)

    # Apply template
    subprocess.run(['pto_template', f'--output={pto_path}', f'--template={config.template_path}', pto_path], check=True)

    # Add control points
    subprocess.run(['cpfind', '-o', pto_path, '--multirow', pto_path], check=True)

    # Clean control points
    subprocess.run(['cpclean', '-o', pto_path, pto_path], check=True)

    # Find line features
    subprocess.run(['linefind', '-o', pto_path, pto_path], check=True)

    # Optimize the project
    subprocess.run(['pto_var', '-o', pto_path, '--opt', 'TrX,TrY,TrZ,r,Eev,Ra,Rb,Rc,Rd,Re,!TrX0,!TrY0,!TrZ0,!r0,!Eev0,!Ra1,!Rb1,!Rc1,!Rd1,!Re1', pto_path], check=True)
    subprocess.run(['autooptimiser', '-n', '-o', pto_path, pto_path], check=True)

    # Modify resolution of the temporary project file
    hugin_modify(pto_path, pto_path, width=config.get("PANO", "PANO_WIDTH"))

    # Start stitching as non-blocking thread with low priority
    if config.get("ENABLE_PANO"):
        cmd_string = ['nice', '-n', str(nice), 'hugin_executor', '--stitching', f'--prefix={output_path}', pto_path]
        process = subprocess.Popen(cmd_string)

        # Wait for the process to finish
        process.wait()

        # Check return code if stitching was successful
        if process.returncode != 0:
            raise Exception(f"Command failed with return code {process.returncode}")

    return output_path


def sample_color(img, uv, normalize_color=False):
    longitude, latitude = uv

    if isinstance(longitude, float):
        h, w, c = img.shape
        longitude = int(longitude * w)
        latitude = int(latitude * h)

    # print(longitude, latitude)
    color = img[latitude, longitude]

    if normalize_color:  # convert uint8 to float
        color = color / 255

    return tuple(color)


def vector_to_longlat(point3d):
    Px, Py, Pz = point3d
    longitude = math.atan2(-Py, Px)
    latitude = math.atan2(Pz, math.sqrt(Px**2 + Py**2))
    return longitude, latitude  # CW


def longlat_to_vector(longlat):
    longitude, latitude = longlat
    Px = math.cos(latitude) * math.cos(longitude)
    Py = math.cos(latitude) * math.sin(longitude)
    Pz = math.sin(latitude)
    return Px, Py, Pz


def longlat_to_spherical(longlat):  # horizontal, vertical
    longitude, latitude = longlat
    u = longitude / math.pi / 2
    v = 2 * latitude / math.pi
    return u, v


def spherical_to_longlat(uv):
    u, v = uv
    longitude = u * math.pi
    latitude = v * math.pi / 2
    return longitude, latitude


def deg2rad(deg_list):
    """Convert iterable of degrees to radians."""
    return tuple(map(math.radians, deg_list))


def rad2deg(rad_list):
    """Convert iterable of radians to degrees."""
    return tuple(map(math.degrees, rad_list))


def mod(angles, mod=math.pi*2):
    angles = list(map(math.fmod, angles, [mod] * len(angles)))
    return angles


def polar2cartesian(angles, distances, offset_angle=math.pi/2):
    """
    https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates

    converts list of polar coordinates into cartesian (x/y)
    :param angles: list of radians
    :param distances: list of distances
    :param offset_angle: offset radian
    :return: two lists of x and y coordinates
    """
    angles = list(np.array(angles) + offset_angle)
    x_list = distances * -np.cos(angles)
    y_list = distances * np.sin(angles)
    return x_list, y_list


def cartesian2polar(x_list, y_list, offset_angle=math.pi/2):
    distances = np.sqrt(x_list**2 + y_list**2)
    angles = -np.arctan2(y_list, x_list)
    angles = list(np.array(angles) + offset_angle)
    return angles, distances


if __name__ == "__main__":
    from file_utils import list_files
    from config import Config

    config = Config()
    config.init(scan_id="240915-2326")
    

    # Get all jpg files in the directory
    files = list_files(config.img_dir, filter="_0.jpg")
    print(len(files), "images found.")
    
    config.imglist = files[0:4]
    print("imglist:", config.imglist)

    config.template_path = f'{config.base_dir}/hugin/template_4.pto'

    hugin_stitch(config)


    # files = list_files(config.img_dir)
    # print(len(files), "images found.")
    # print("imglist:", config.imglist)

    # config.template_path = f'{config.base_dir}/hugin/template_4_AEB1-2.pto'
    # hugin_refine(config)
