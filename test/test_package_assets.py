import ast
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_package_assets_and_rviz_topics_are_consistent():
    assert ET.parse(ROOT / 'package.xml').getroot().findtext('name') == 'hih_rviz'
    config = yaml.safe_load((ROOT / 'config/visualization.yaml').read_text())['hih_rviz']['ros__parameters']
    rviz = yaml.safe_load((ROOT / 'rviz/handinhand.rviz').read_text())['Visualization Manager']
    assert rviz['Global Options']['Fixed Frame'] == config['world_frame']
    for display in rviz['Displays']:
        if 'Topic' in display:
            assert display['Topic']['Value'].startswith(config['output_prefix'] + '/')
    for script in (ROOT / 'launch').glob('*.py'):
        ast.parse(script.read_text())
    for filename in ('README.md', 'resource/hih_rviz', 'setup.cfg', 'LICENSE'):
        assert (ROOT / filename).is_file()
