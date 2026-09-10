#!/usr/bin/env python3
"""
Lanza una instancia de camera_node por cada device en `devices`
(lista separada por comas, ej: '0,2'), cada una con su propio nombre
de nodo y tópico de salida.

Uso:
    ros2 launch vision cameras.launch.py devices:=0,2 frame_rate:=24
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    devices = LaunchConfiguration("devices").perform(context)
    frame_rate = LaunchConfiguration("frame_rate").perform(context)

    nodes = []
    for device in devices.split(","):
        device = device.strip()
        nodes.append(
            Node(
                package="vision",
                executable="camera_node",
                name=f"camera_node_{device}",
                parameters=[{
                    "video_device": int(device),
                    "frame_rate": int(frame_rate),
                }],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "devices",
            default_value="0",
            description="Índices de /dev/video a lanzar, separados por coma (ej: '0,2')",
        ),
        DeclareLaunchArgument(
            "frame_rate",
            default_value="24",
            description="FPS de captura/publicación para cada cámara",
        ),
        OpaqueFunction(function=launch_setup),
    ])
