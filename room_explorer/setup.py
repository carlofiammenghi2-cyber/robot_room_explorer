from setuptools import setup
import os
from glob import glob 

package_name = 'room_explorer'
launch_files = [
    path for path in glob(os.path.join('launch', '*'))
    if os.path.isfile(path)
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.modules'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
    ],
    install_requires=['setuptools', 'numpy', 'numpy-quaternion', 'pyyaml', 'robomaster'],
    zip_safe=True,
    maintainer='carlofiammenghi',
    maintainer_email='carlofiammenghi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robomaster_driver = room_explorer.robomaster_driver:main',
            'h264_decoder = room_explorer.decompress_h264:main',
            'play_audio = room_explorer.play_audio:main',
            'play_opus = room_explorer.play_audio_opus:main',
            'display_battery = room_explorer.display_battery:main',
            'connect = room_explorer.connect:main',
            'discover = room_explorer.discover:main',
            'lidar_bridge = room_explorer.lidar_bridge:main',
            'room_explorer_node = room_explorer.room_explorer_node:main',
            'coppeliasim_odom = room_explorer.coppeliasim_odom:main'
        ],
    },
)
