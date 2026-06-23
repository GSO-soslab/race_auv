from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'race_auv_sim_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        (os.path.join('share', package_name), ["package.xml"]),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='AprilTag detection bridge for Stonefish-simulated cameras.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apriltag_detector_node = race_auv_sim_pkg.apriltag_detector_node:main',
            'apriltag_fuser_node = race_auv_sim_pkg.apriltag_fuser_node:main',
            'ground_truth_docking_node = race_auv_sim_pkg.ground_truth_docking_node:main',
        ],
    },
)
