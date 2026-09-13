import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='andrescarlos011@gmail.com',
    description='Nodos de cámara: captura, compresión JPEG y publicación como sensor_msgs/CompressedImage.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Rover
            'camera_node=vision.camera_node:main',
            'rtt_probe_node=vision.rtt_probe_node:main',
            # Estación base
            'image_viewer_node=vision.image_viewer_node:main',
            'latency_probe_node=vision.latency_probe_node:main',
            'link_echo_node=vision.link_echo_node:main',
        ],
    },
)
