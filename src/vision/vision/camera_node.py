#!/usr/bin/env python3
"""
Nodo de cámara — publica sensor_msgs/CompressedImage.

Va en:  src/vision/vision/camera_node.py

    ros2 run vision camera_node                                  # CSI, valores por defecto
    ros2 run vision camera_node --ros-args -p jpeg_quality:=55    # menos ancho de banda
    ros2 run vision camera_node --ros-args -p source:=usb:0 -p height:=480

Lo normal es lanzarlo con:  ros2 launch vision rover.launch.py

Diseño
------
- La captura corre en su propio hilo y publica en cuanto llega cada cuadro.
  Con un temporizador, cualquier desfase entre el ritmo del temporizador y el
  de la cámara hace que se acumulen cuadros viejos y la latencia crezca sola.

- QoS BEST_EFFORT / KEEP_LAST(1): un cuadro perdido no se reenvía. Para video
  en vivo es lo correcto; un cuadro retransmitido llega tarde y desplaza a uno
  nuevo.

- El número de secuencia viaja en header.frame_id ("camera_link|1523")
  porque ROS 2 quitó el campo seq. Sin él no se puede medir pérdida.

- Si la cámara se cae (cable CSI flojo por vibración, por ejemplo), el nodo
  lo registra y reintenta abrirla cada 2 s en lugar de morir en silencio.

- Cada segundo publica en <base>/link (std_msgs/String, JSON) el estado del
  Wi-Fi del rover y la configuración de la cámara. La consola del operador lo
  usa para avisar de señal débil antes de que se corte el video.

- Sirve el video por HTTP en el puerto 8080 (http://<ip-de-la-pi>:8080), para
  verlo desde cualquier navegador sin instalar nada (Mac, celular).

- Responde a la baliza en el puerto UDP 47800, para que el PC encuentre la
  Pi aunque cambie su IP (ver rover_beacon.py).
"""
import socket
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from builtin_interfaces.msg import Time

from vision.camera_source import create_source
from vision.link_metrics import pack_frame_id, CsvLogger
from vision.wifi_status import read_wifi
from vision.web_server import FrameHub, WebServer
from vision.rover_beacon import BeaconResponder, PORT as BEACON_PORT


def camera_qos_profile() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def ns_to_time_msg(ns: int) -> Time:
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # --- Qué cámara ---
        self.declare_parameter("source", "csi:0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 360)
        self.declare_parameter("fps", 30)
        self.declare_parameter("jpeg_quality", 70)

        # --- Solo CSI (se ignoran con USB) ---
        self.declare_parameter("sensor_mode", "2304:1296")
        self.declare_parameter("af_mode", "manual")
        self.declare_parameter("lens_position", 0.0)
        self.declare_parameter("shutter_us", 0)
        self.declare_parameter("hflip", False)
        self.declare_parameter("vflip", False)
        self.declare_parameter("tuning_file", "")

        # --- Publicación ---
        self.declare_parameter("topic", "")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("wifi_iface", "auto")    # auto = la que lleva el tráfico
        self.declare_parameter("link_period", 1.0)      # s entre reportes de estado
        self.declare_parameter("http_port", 8080)       # 0 = sin visor web
        self.declare_parameter("beacon_port", BEACON_PORT)  # 0 = sin baliza

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.source_spec_ = p("source")
        self.frame_id_ = p("frame_id")
        self.source_kwargs_ = {
            "width": p("width"),
            "height": p("height"),
            "fps": p("fps"),
            "jpeg_quality": p("jpeg_quality"),
            "sensor_mode": p("sensor_mode"),
            "af_mode": p("af_mode"),
            "lens_position": float(p("lens_position")),
            "shutter_us": int(p("shutter_us")),
            "hflip": bool(p("hflip")),
            "vflip": bool(p("vflip")),
            "tuning_file": p("tuning_file"),
        }

        topic = p("topic") or f"{self.source_spec_.replace(':', '')}/image/compressed"
        self.publisher_ = self.create_publisher(
            CompressedImage, topic, camera_qos_profile()
        )
        self.csv_ = CsvLogger(p("csv_path"), ["seq", "stamp_ns", "size_bytes"])

        # Estado del enlace para la consola del operador. QoS por defecto
        # (RELIABLE, profundidad 1): es un mensaje pequeño y poco frecuente.
        base = topic.rsplit("/image", 1)[0]
        self.link_pub_ = self.create_publisher(String, f"{base}/link", 1)
        self.wifi_iface_ = p("wifi_iface")
        self.last_link_ = {}
        self.create_timer(float(p("link_period")), self._publish_link)

        # Visor web y baliza. Si un puerto está ocupado, se avisa y se sigue:
        # la cámara por ROS funciona igual.
        self.hub_ = FrameHub()
        self.web_ = None
        http_port = int(p("http_port"))
        if http_port > 0:
            try:
                self.web_ = WebServer(self.hub_, self._status, http_port)
                self.web_.start()
            except OSError as exc:
                self.get_logger().warn(f"visor web desactivado (puerto {http_port}): {exc}")
                self.web_ = None
        self.beacon_ = None
        beacon_port = int(p("beacon_port"))
        if beacon_port > 0:
            try:
                self.beacon_ = BeaconResponder(self._beacon_info, beacon_port)
                self.beacon_.start()
            except OSError as exc:
                self.get_logger().warn(f"baliza desactivada (puerto {beacon_port}): {exc}")
                self.beacon_ = None
        self.http_port_ = http_port if self.web_ else 0
        self.topic_ = topic

        # Abrir la cámara aquí para que un fallo al arrancar se vea de
        # inmediato en la terminal, con el motivo.
        self.source_ = create_source(self.source_spec_, **self.source_kwargs_)
        self.source_.open()

        self.seq_ = 0
        self.running_ = True
        self.thread_ = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread_.start()

        self.get_logger().info(
            f"camera_node listo — {self.source_.describe()}  topic=/{topic}"
        )
        if self.web_:
            self.get_logger().info(
                f"video en el navegador: http://{self._my_ip() or '<ip-de-la-pi>'}:{self.http_port_}"
            )

    # ----------------------------------------------------------------------
    @staticmethod
    def _my_ip():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        except OSError:
            return None
        finally:
            s.close()

    def _status(self):
        st = dict(self.last_link_)
        st.update({"seq": self.seq_, "topic": "/" + self.topic_})
        return st

    def _beacon_info(self):
        return {"rover": socket.gethostname(), "http_port": self.http_port_,
                "topic": "/" + self.topic_}

    def _publish_link(self):
        info = read_wifi(self.wifi_iface_)
        info.update({
            "fps": self.source_kwargs_["fps"],
            "quality": self.source_kwargs_["jpeg_quality"],
            "width": self.source_kwargs_["width"],
            "height": self.source_kwargs_["height"],
        })
        self.last_link_ = info
        try:
            self.link_pub_.publish(String(data=json.dumps(info)))
        except Exception:
            pass

    def _reopen(self):
        """Cierra y vuelve a abrir la cámara hasta que funcione o se pida parar."""
        try:
            self.source_.close()
        except Exception:
            pass
        while self.running_ and rclpy.ok():
            time.sleep(2.0)
            try:
                self.source_ = create_source(self.source_spec_, **self.source_kwargs_)
                self.source_.open()
                self.get_logger().info("cámara recuperada")
                return
            except Exception as exc:
                self.get_logger().warn(
                    f"reintentando abrir la cámara: {str(exc).splitlines()[0]}",
                    throttle_duration_sec=10.0,
                )

    def _capture_loop(self):
        misses = 0
        while self.running_ and rclpy.ok():
            try:
                frame = self.source_.read()
            except RuntimeError as exc:
                if not self.running_ or not rclpy.ok():
                    break          # se está cerrando el nodo, no es una caída
                self.get_logger().error(f"la cámara se cayó:\n{exc}")
                self._reopen()
                continue

            if frame is None:
                misses += 1
                if misses in (1, 10, 100):
                    self.get_logger().warn(f"{misses} lecturas sin cuadro nuevo")
                continue
            misses = 0

            msg = CompressedImage()
            msg.header.stamp = ns_to_time_msg(frame.stamp_ns)
            msg.header.frame_id = pack_frame_id(self.frame_id_, self.seq_)
            msg.format = "jpeg"
            msg.data = frame.jpeg
            try:
                self.publisher_.publish(msg)
            except Exception:
                # El contexto de ROS se cerró (Ctrl+C) mientras publicábamos.
                break
            self.hub_.publish(frame.jpeg, self.seq_)

            if self.csv_.enabled:
                self.csv_.write({
                    "seq": self.seq_,
                    "stamp_ns": frame.stamp_ns,
                    "size_bytes": len(frame.jpeg),
                })
            self.seq_ += 1

    # ----------------------------------------------------------------------
    def destroy_node(self):
        self.running_ = False
        if self.thread_.is_alive():
            self.thread_.join(timeout=3.0)
        self.csv_.close()
        self.source_.close()
        if self.web_:
            self.web_.stop()
        if self.beacon_:
            self.beacon_.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
    except Exception as exc:
        print(f"\n[camera_node] no se pudo iniciar la cámara:\n{exc}\n")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy ya cierra el contexto al recibir Ctrl+C; llamarlo otra vez
        # lanza "rcl_shutdown already called".
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
