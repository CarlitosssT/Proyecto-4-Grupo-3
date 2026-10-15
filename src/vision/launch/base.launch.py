#!/usr/bin/env python3
"""
Lado ESTACIÓN BASE (tu PC).

Va en:  src/vision/launch/base.launch.py

    ros2 launch vision base.launch.py                  # ver la cámara
    ros2 launch vision base.launch.py hud:=false       # sin métricas encima
    ros2 launch vision base.launch.py fullscreen:=true # directo a pantalla completa
    ros2 launch vision base.launch.py scale:=1.5       # ventana inicial más pequeña
    ros2 launch vision base.launch.py viewer:=false echo:=true   # solo medir
    ros2 launch vision base.launch.py rover:=192.168.0.13        # IP a mano

La IP de la Pi ya no va escrita en ningún lado. Al arrancar, este launch la
busca (ver vision/rover_beacon.py) y se la pasa a ROS en ROS_STATIC_PEERS,
solo para los nodos que lanza. Si la Pi todavía no ha arrancado, sigue
buscando hasta `wait` segundos.

Para medir latencia van en pareja: probe:=true en el rover y echo:=true aquí.
Mientras se mide conviene cerrar la ventana (viewer:=false): decodificar y
dibujar compite por CPU y ensucia la medición.
"""
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGS = [
    ("source", "csi:0", "Debe coincidir con el source del rover"),
    ("topic", "", "Tópico explícito; vacío = se deduce de source"),
    ("hud", "true", "Métricas superpuestas en la imagen"),
    ("scale", "2.0", "Tamaño inicial de la ventana = video x scale"),
    ("fullscreen", "false", "Abrir en pantalla completa"),
    ("viewer", "true", "false para no abrir ventana"),
    ("echo", "false", "true para devolver sellos al rover (medir RTT)"),
    ("rover", "auto", "auto = buscar la Pi en la red; o una IP / nombre fijo"),
    ("wait", "20", "Segundos máximos buscando la Pi antes de rendirse"),
]


def _bool(s):
    return str(s).strip().lower() in ("true", "1", "yes", "si")


def _locate(rover, wait_s):
    """Devuelve (ip, explicación) o (None, explicación)."""
    if rover and rover != "auto":
        return rover, "indicada a mano (rover:=...)"

    from vision.rover_beacon import find_rover

    deadline = time.time() + wait_s
    tries = 0
    while True:
        tries += 1
        ip, how, info = find_rover()
        if ip:
            name = (info or {}).get("rover")
            extra = f", {name}" if name else ""
            return ip, f"encontrada por {how}{extra}"
        if time.time() >= deadline:
            return None, f"no encontrada tras {tries} intentos"
        if tries == 1:
            print("[base] Buscando la Pi en la red... (¿ya arrancó rover.launch.py?)", flush=True)
        time.sleep(1.0)


def setup(context, *_):
    v = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    topic = v("topic") or f"/{v('source').replace(':', '')}/image/compressed"

    actions = []
    ip, why = _locate(v("rover").strip(), float(v("wait")))
    if ip:
        actions += [
            SetEnvironmentVariable("ROS_STATIC_PEERS", ip),
            LogInfo(msg=f"[base] Rover en {ip} ({why}). "
                        f"Video también en el navegador: http://{ip}:8080"),
        ]
    else:
        prev = os.environ.get("ROS_STATIC_PEERS", "")
        actions.append(LogInfo(msg=(
            f"[base] AVISO: Pi {why}. Se sigue con ROS_STATIC_PEERS='{prev}'.\n"
            "  Revisa que la Pi esté encendida, con rover.launch.py corriendo,\n"
            "  y en la MISMA red Wi-Fi que este PC. O pásala a mano: rover:=IP")))

    if _bool(v("viewer")):
        actions.append(Node(
            package="vision", executable="image_viewer_node",
            name="image_viewer_node", output="screen",
            parameters=[{"topic": topic, "hud": _bool(v("hud")),
                         "scale": float(v("scale")),
                         "fullscreen": _bool(v("fullscreen"))}],
        ))
    if _bool(v("echo")):
        actions.append(Node(
            package="vision", executable="link_echo_node",
            name="link_echo_node", output="screen",
            parameters=[{"topic": topic}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [OpaqueFunction(function=setup)]
    )
