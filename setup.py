from glob import glob
from setuptools import find_packages, setup

setup(
    name='hih_rviz', version='0.2.0', packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hih_rviz']),
        ('share/hih_rviz', ['package.xml', 'README.md']),
        ('share/hih_rviz/launch', glob('launch/*.launch.py')),
        ('share/hih_rviz/config', glob('config/*.yaml')),
        ('share/hih_rviz/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
    maintainer='INTGER-PARK', maintainer_email='jjungsoo2022@gmail.com',
    description='Read-only RViz2 visualization for an existing HIH_sim', license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'hih_visualizer = hih_rviz.visualizer:main',
    ]},
)
