#!/usr/bin/env python3
"""
Lanza una instancia de camera_node por cada fuente indicada.

Va en:  src/vision/launch/cameras.launch.py

A diferencia de la versión anterior, `sources` acepta tipos mezclados. Esto
deja de ser un detalle cosmético en cuanto pasas a CSI: una Raspberry Pi 4
tiene UN solo puerto CSI y la Pi 5 tiene dos. Si tu rover necesita cámara
frontal y trasera, la configuración real va a ser mixta.

Uso:
    ros2 launch vision cameras.launch.py sources:=usb:0
    ros2 launch vision cameras.launch.py sources:=csi:0,usb:2 fps:=30
    ros2 launch vision cameras.launch.py sources:=csi:0 encode_mode:=hardware
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGS = [
    ("sources", "usb:0", "Fuentes separadas por coma: usb:N o csi:N"),
    ("width", "640", "Ancho de captura"),
    ("height", "480", "Alto de captura"),
    ("fps", "30", "Cuadros por segundo"),
    ("jpeg_quality", "75", "Calidad JPEG 1-100"),
    ("encode_mode", "cpu", "cpu | passthrough (usb) | hardware (csi)"),
    ("lens_position", "0.0", "Foco fijo en dioptrías, solo CSI (0 = infinito)"),
    ("af_mode", "manual", "manual | auto | continuous, solo CSI"),
]


def launch_setup(context, *args, **kwargs):
    def val(name):
        return LaunchConfiguration(name).perform(context)

    nodes = []
    for spec in val("sources").split(","):
        spec = spec.strip()
        if not spec:
            continue
        # Los nombres de nodo no admiten ':' — usb:0 se vuelve usb_0
        safe = spec.replace(":", "_")
        nodes.append(
            Node(
                package="vision",
                executable="camera_node",
                name=f"camera_node_{safe}",
                parameters=[{
                    "source": spec,
                    "width": int(val("width")),
                    "height": int(val("height")),
                    "fps": int(val("fps")),
                    "jpeg_quality": int(val("jpeg_quality")),
                    "encode_mode": val("encode_mode"),
                    "lens_position": float(val("lens_position")),
                    "af_mode": val("af_mode"),
                }],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h)
         for n, d, h in ARGS]
        + [OpaqueFunction(function=launch_setup)]
    )
