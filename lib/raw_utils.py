import pickle


def get_scan_dict(z_angles, angular_list=None, cartesian_list=None, scan_id=None, device_id=None, sensor=None, hardware=None, location=None, author=None):
    return {
        "header": {
            "scan_id": scan_id,
            "device_id": device_id,
            "sensor": sensor,
            "hardware": hardware,
            "location": location,
            "author": author,
        },
        "z_angles": z_angles,
        "angular": angular_list,
        "cartesian": cartesian_list,
    }


def save_raw_scan(path, data):
    if isinstance(data, dict):
        with open(path, "wb") as f:
            pickle.dump(data, f)


def load_raw_scan(path):
    with open(path, "rb") as f:
        return pickle.load(f)
