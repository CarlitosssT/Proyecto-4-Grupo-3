from setuptools import find_packages, setup

package_name = 'locomotion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='andrescarlos011@gmail.com',
    description='Control de locomoción: 6 motores DC (3 puentes H) vía DualSense o teclado.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_command=locomotion.motor_command:main',
            'rc_control=locomotion.rc_control:main',
            'keyboard_control=locomotion.keyboard_control:main',
        ],
    },
)
