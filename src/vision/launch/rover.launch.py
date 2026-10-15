#!/usr/bin/env python3
"""
Lado ROVER (la Raspberry Pi).

Va en:  src/vision/launch/rover.launch.py

    ros2 launch vision rover.launch.py                     # cámara CSI
    ros2 launch vision rover.launch.py quality:=55         # enlace flojo
    ros2 launch vision rover.launch.py lens:=0.5           # enfocar a 2 m
    ros2 launch vision rover.launch.py probe:=true label:=15m   # medir latencia

Con probe:=true quedan tres archivos en ~/medidas/ con la etiqueta:
    tx_<label>.csv   cada cuadro enviado (número, hora, tamaño)
    rtt_<label>.csv  cada eco que volvió de la base (tiempo ida y vuelta)
    pi_<label>.csv   cada 2 s: temperatura, CPU, señal Wi-Fi

Valores por defecto y de dónde salen:

  640x360       El sensor IMX708 es 16:9. Pedir 4:3 (640x480) hace que se
                recorten los lados y se pierde campo de visión, que es
                justamente lo que se pagó en la versión Wide.
  quality 70    Con la Brio, calidad 70 daba ~3 Mbps a 640x480@30. A
                640x360 son menos píxeles, así que menos todavía.
  lens 0.0      Foco fijo en infinito. El autofoco continuo "caza" el foco
                en cada bache.
  probe false   La sonda de latencia solo se enciende para medir.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGS = [
    ("source", "csi:0", "csi:0 (Camera Module) o usb:0 (webcam)"),
    ("width", "640", "Ancho"),
    ("height", "360", "Alto (usar 16:9 con la CSI para no recortar el campo)"),
    ("fps", "30", "Cuadros por segundo"),
    ("quality", "70", "Calidad JPEG 1-100"),
    ("lens", "0.0", "Foco en dioptrías: 0 = infinito, 1 = 1 m, 2 = 50 cm"),
    ("af_mode", "manual", "manual | continuous | '' (no tocar)"),
    ("sensor_mode", "2304:1296", "Modo del sensor; vacío = automático"),
    ("shutter_us", "0", "0 = automático; 4000-8000 si hay desenfoque por movimiento"),
    ("hflip", "false", "Voltear horizontal"),
    ("vflip", "false", "Voltear vertical (si la cámara va montada al revés)"),
    ("tuning", "", "Ruta a imx708_wide.json si rpicam no detecta la versión Wide"),
    ("wifi_iface", "auto", "Interfaz Wi-Fi a reportar; auto = la que lleva el tráfico"),
    ("probe", "false", "true para medir latencia (requiere echo:=true en la base)"),
    ("label", "", "Etiqueta del CSV de la medición, ej. 15m_carton"),
]


def _bool(s):
    return str(s).strip().lower() in ("true", "1", "yes", "si")


def setup(context, *_):
    v = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    probe = _bool(v("probe"))
    label = v("label") or "sin_etiqueta"
    medidas = "/home/ubuntu/medidas"

    nodes = [
        Node(
            package="vision",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "source": v("source"),
                "width": int(v("width")),
                "height": int(v("height")),
                "fps": int(v("fps")),
                "jpeg_quality": int(v("quality")),
                "lens_position": float(v("lens")),
                "af_mode": v("af_mode"),
                "sensor_mode": v("sensor_mode"),
                "shutter_us": int(v("shutter_us")),
                "hflip": _bool(v("hflip")),
                "vflip": _bool(v("vflip")),
                "tuning_file": v("tuning"),
                "wifi_iface": v("wifi_iface"),
                "csv_path": f"{medidas}/tx_{label}.csv" if probe else "",
            }],
        )
    ]

    if probe:
        nodes.append(
            Node(
                package="vision",
                executable="rtt_probe_node",
                name="rtt_probe_node",
                output="screen",
                parameters=[{
                    "label": label,
                    "expected_fps": float(v("fps")),
                    "csv_path": f"{medidas}/rtt_{label}.csv",
                }],
            )
        )
        nodes.append(
            Node(
                package="vision",
                executable="pi_monitor",
                name="pi_monitor",
                output="screen",
                arguments=["--csv", f"{medidas}/pi_{label}.csv", "--label", label],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [OpaqueFunction(function=setup)]
    )
