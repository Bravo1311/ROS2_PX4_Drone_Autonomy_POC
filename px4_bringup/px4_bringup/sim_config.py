"""Which Gazebo world and PX4 vehicle the bringup launch files use.

The values live in one file, ``config/sim.env`` (installed with this package). This module
reads it, so every launch file agrees without repeating a string. A variable already set in
the environment wins over the file, which is handy for a one-off run:

    PX4_GZ_WORLD=vio_rich ros2 launch px4_bringup minimal.launch.py

The Gazebo model name is derived from the vehicle, never typed: the target
``gz_x500_mono_cam_down`` spawns the model instance ``x500_mono_cam_down_0``.
"""
import functools
import os
from pathlib import Path

ENV_FILE_NAME = 'sim.env'
REQUIRED = ('PX4_GZ_WORLD', 'PX4_VEHICLE', 'PX4_GZ_INSTANCE')


def parse_env_file(text):
    """Parse ``KEY=VALUE`` lines (whole-line ``#`` comments allowed) into a dict."""
    values = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, _, value = line.partition('=')
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in '"\'':
            value = value[1:-1]
        values[key.strip()] = value
    return values


def _candidate_files():
    explicit = os.environ.get('SIM_ENV_FILE')
    if explicit:
        yield Path(explicit)
    # source tree: px4_bringup/px4_bringup/sim_config.py -> px4_bringup/config/sim.env
    yield Path(__file__).resolve().parent.parent / 'config' / ENV_FILE_NAME
    try:  # installed package
        from ament_index_python.packages import get_package_share_directory
        yield Path(get_package_share_directory('px4_bringup')) / 'config' / ENV_FILE_NAME
    except Exception:  # not in a ROS environment (e.g. plain pytest)
        return


@functools.lru_cache(maxsize=1)
def file_values():
    """Return the values from the first ``sim.env`` found (raises if none or incomplete)."""
    tried = []
    for path in _candidate_files():
        tried.append(str(path))
        if path.is_file():
            values = parse_env_file(path.read_text())
            missing = [key for key in REQUIRED if not values.get(key)]
            if missing:
                raise ValueError(f'{path} is missing: {", ".join(missing)}')
            return values
    raise FileNotFoundError(
        f'{ENV_FILE_NAME} not found (looked in: {", ".join(tried)}). '
        'Rebuild with: colcon build --packages-select px4_bringup'
    )


def setting(name):
    """Return one setting: the environment variable if set, else the value in sim.env."""
    return os.environ.get(name) or file_values()[name]


def world_name():
    """Return the Gazebo world name."""
    return setting('PX4_GZ_WORLD')


def vehicle_target():
    """Return the PX4 make target, e.g. ``gz_x500_mono_cam_down``."""
    return setting('PX4_VEHICLE')


def instance_number():
    """Return the PX4 instance number (0 for a single drone)."""
    return int(setting('PX4_GZ_INSTANCE'))


def model_name(vehicle=None):
    """Return the Gazebo model name for a vehicle target (``gz_`` prefix removed)."""
    target = vehicle or vehicle_target()
    return target[len('gz_'):] if target.startswith('gz_') else target


def drone_name(vehicle=None, instance=None):
    """Return the Gazebo model *instance* name, e.g. ``x500_mono_cam_down_0``."""
    number = instance_number() if instance is None else instance
    return f'{model_name(vehicle)}_{number}'
