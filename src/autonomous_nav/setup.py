from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name, 
        package_name + '.costmap',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='user@example.com',
    description='Autonomous Navigation Package with ZED2i Camera Support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_costmap_generator = autonomous_nav.scripts.local_costmap_generator:main',
        ],
    },
)
