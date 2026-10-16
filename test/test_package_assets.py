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


def test_installed_launch_and_entry_points_only_start_visualization():
    # Inspect everything setup.py will install, not just the recommended launch.
    scripts = list((ROOT / 'launch').glob('*.launch.py'))
    assert [script.name for script in scripts] == ['visualization.launch.py']
    nodes = []
    for call in ast.walk(ast.parse(scripts[0].read_text())):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'Node':
            kwargs = {kw.arg: ast.literal_eval(kw.value) for kw in call.keywords
                      if kw.arg in ('package', 'executable')}
            nodes.append((kwargs['package'], kwargs['executable']))
    assert sorted(nodes) == [('hih_rviz', 'hih_visualizer'), ('rviz2', 'rviz2')]
    setup = ast.parse((ROOT / 'setup.py').read_text())
    entry_points = next(kw.value for call in ast.walk(setup) if isinstance(call, ast.Call)
                        for kw in call.keywords if kw.arg == 'entry_points')
    assert ast.literal_eval(entry_points) == {'console_scripts': [
        'hih_visualizer = hih_rviz.visualizer:main']}
    rviz = yaml.safe_load((ROOT / 'rviz/handinhand.rviz').read_text())['Visualization Manager']
    assert {tool['Class'] for tool in rviz['Tools']} == {
        'rviz_default_plugins/Interact', 'rviz_default_plugins/MoveCamera', 'rviz_default_plugins/Select'}


def test_runtime_sources_have_no_plant_or_controller_startup_path():
    for path in (ROOT / 'hih_rviz').glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom) and node.module:
                assert node.module.split('.')[0] not in ('plant', 'hih_controller', 'subprocess')
            if isinstance(node, ast.Import):
                assert all(alias.name.split('.')[0] not in ('plant', 'hih_controller', 'subprocess')
                           for alias in node.names)
