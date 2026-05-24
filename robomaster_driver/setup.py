from setuptools import setup
import os
from glob import glob 

package_name = 'robomaster_driver'
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
    ],
    install_requires=['setuptools', 'numpy', 'numpy-quaternion', 'pyyaml', 'robomaster'],
    zip_safe=True,
    maintainer='carlofiammenghi',
    maintainer_email='carlofiammenghi@todo.todo',
    description='ROS 2 Python package for DJI Robomaster SDK driver',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robomaster_driver = robomaster_driver.robomaster_driver:main',
            'h264_decoder = robomaster_driver.decompress_h264:main',
            'play_audio = robomaster_driver.play_audio:main',
            'play_opus = robomaster_driver.play_audio_opus:main',
            'display_battery = robomaster_driver.display_battery:main',
            'connect = robomaster_driver.connect:main',
            'discover = robomaster_driver.discover:main',
        ],
    },
)
