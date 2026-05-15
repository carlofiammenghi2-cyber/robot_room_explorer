from setuptools import setup

package_name = 'hw1_usi_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlofiammenghi',
    maintainer_email='carlofiammenghi@todo.todo',
    description='Angry Turtle controller: writes US! and chases intruders in turtlesim',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usi_turtle = hw1_usi_turtle.usi_turtle:main',
        ],
    },
)
