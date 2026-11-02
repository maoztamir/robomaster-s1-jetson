"""Save home poses against exact map contents, and validate startup seeds."""
import hashlib
import math
from pathlib import Path

import yaml


def map_signature(map_path):
    map_path = Path(map_path).resolve()
    data = yaml.safe_load(map_path.read_text())
    image = (map_path.parent / data['image']).resolve()
    return {'yaml_sha256': hashlib.sha256(map_path.read_bytes()).hexdigest(),
            'image_sha256': hashlib.sha256(image.read_bytes()).hexdigest()}


def home_parameters(home_path, map_path):
    if not home_path or not Path(home_path).exists():
        return {}
    data = yaml.safe_load(Path(home_path).read_text())
    if data.get('frame_id') != 'localization_map':
        raise ValueError('Home pose must use localization_map')
    if data.get('map_signature') != map_signature(map_path):
        raise ValueError('Home pose belongs to different map contents; use --manual to realign')
    pose = data['pose']
    if not all(isinstance(pose[k], (int, float)) and math.isfinite(pose[k])
               for k in ('x', 'y', 'yaw')):
        raise ValueError('Invalid home coordinates')
    return {'set_initial_pose': True, 'initial_pose.x': float(pose['x']),
            'initial_pose.y': float(pose['y']), 'initial_pose.z': 0.,
            'initial_pose.yaw': float(pose['yaw'])}
