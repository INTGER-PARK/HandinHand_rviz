"""One YAML file owns all visualization defaults."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory


def defaults():
    root = Path(get_package_share_directory('hih_rviz'))
    return yaml.safe_load((root / 'config/visualization.yaml').read_text())['hih_rviz']['ros__parameters']
