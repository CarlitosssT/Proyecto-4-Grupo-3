#!/usr/bin/env python3
"""
Nodo de cámara v3 — agnóstico a la fuente (USB o CSI).

Va en:  src/vision/vision/camera_node.py

Cambio estructural respecto a v2: la captura vive en su propio HILO, no en un
timer de ROS.

Por qué importa
---------------
El diseño con timer tenía un defecto de fondo, no un bug puntual. El timer
dispara cada 1/fps segundos y hace un read() bloqueante. Si el consumo va más
lento que la producción, la cola del driver crece y cada read() devuelve un
frame más viejo que el anterior. Ponerle un grab() de descarte encima es un
parche.

Con un hilo dedicado el bucle es: leer, publicar, repetir, sin esperas. La
cámara marca el ritmo y la cola nunca crece porque siempre se está consumiendo.
El problema deja de existir en vez de mitigarse. Es además la única forma
razonable de hablar con Picamera2, que ya gestiona su propio pipeline.

Los publishers de rclpy son thread-safe, así que publicar desde el hilo de
captura es correcto y ahorra un salto de cola.

Uso
---
    # Hoy, con la webcam
    ros2 run vision camera_node --ros-args -p source:=usb:0 -p jpeg_quality:=75

    # Mañana, con la Camera Module 3 Wide, mismo nodo
    ros2 run vision camera_node --ros-args -p source:=csi:0 \\
        -p encode_mode:=hardware -p lens_position:=0.0
"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import CompressedImage, CameraInfo
from builtin_interfaces.msg import Time

from vision.camera_source import create_source
from vision.link_metrics import pack_frame_id, CsvLogger


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

        # --- Qué cámara y cómo ---
        self.declare_parameter("source", "usb:0")      # usb:N o csi:N
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("encode_mode", "cpu")   # cpu|passthrough|hardware

        # --- Solo CSI (se ignoran con USB) ---
        self.declare_parameter("af_mode", "manual")    # manual|auto|continuous
        self.declare_parameter("lens_position", 0.0)   # dioptrías, 0 = infinito
        self.declare_parameter("exposure_max_us", 4000)
        self.declare_parameter("hflip", False)
        self.declare_parameter("vflip", False)

        # --- Solo USB ---
        self.declare_parameter("flush_buffer", False)

        # --- Publicación ---
        self.declare_parameter("topic", "")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("camera_info_url", "")  # YAML de calibración
        self.declare_parameter("csv_path", "")

        source_spec = self.get_parameter("source").value
        self.frame_id_ = self.get_parameter("frame_id").value

        kwargs = {
            "width": self.get_parameter("width").value,
            "height": self.get_parameter("height").value,
            "fps": self.get_parameter("fps").value,
            "jpeg_quality": int(self.get_parameter("jpeg_quality").value),
            "encode_mode": self.get_parameter("encode_mode").value,
            "af_mode": self.get_parameter("af_mode").value,
            "lens_position": float(self.get_parameter("lens_position").value),
            "exposure_max_us": int(self.get_parameter("exposure_max_us").value),
            "hflip": bool(self.get_parameter("hflip").value),
            "vflip": bool(self.get_parameter("vflip").value),
            "flush_buffer": bool(self.get_parameter("flush_buffer").value),
        }

        self.source_ = create_source(source_spec, **kwargs)
        self.source_.open()

        topic = self.get_parameter("topic").value
        if not topic:
            safe = source_spec.replace(":", "")
            topic = f"{safe}/image/compressed"
        base = topic.rsplit("/image/", 1)[0] if "/image/" in topic else topic

        self.publisher_ = self.create_publisher(
            CompressedImage, topic, camera_qos_profile()
        )

        # CameraInfo va con QoS por defecto (RELIABLE) a propósito: es un
        # mensaje pequeño y constante, y quien lo necesita lo necesita
        # correcto, no rápido.
        self.info_ = self._load_camera_info(
            self.get_parameter("camera_info_url").value
        )
        self.info_pub_ = self.create_publisher(
            CameraInfo, f"{base}/camera_info", 1
        ) if self.info_ is not None else None

        self.csv_ = CsvLogger(
            self.get_parameter("csv_path").value,
            ["seq", "stamp_ns", "size_bytes"],
        )

        self.seq_ = 0
        self.running_ = True
        self.thread_ = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread_.start()

        self.get_logger().info(
            f"camera_node listo — {self.source_.describe()}  topic={topic}"
        )

    # ----------------------------------------------------------------------
    def _load_camera_info(self, url: str):
        """
        Carga un YAML de calibración en el formato estándar de ROS (el que
        produce `ros2 run camera_calibration cameracalibrator`).

        Sin calibración no hay medición métrica posible: ni distancia a un
        objeto, ni pose de un ArUco, ni odometría visual. Y con la Module 3
        Wide importa el doble, porque a 102 grados horizontales la distorsión
        de barril es severa en los bordes.
        """
        if not url:
            return None
        try:
            import yaml
            path = url.replace("file://", "")
            with open(path) as fh:
                d = yaml.safe_load(fh)
            info = CameraInfo()
            info.width = int(d["image_width"])
            info.height = int(d["image_height"])
            info.distortion_model = d.get("distortion_model", "plumb_bob")
            info.d = list(map(float, d["distortion_coefficients"]["data"]))
            info.k = list(map(float, d["camera_matrix"]["data"]))
            info.r = list(map(float, d["rectification_matrix"]["data"]))
            info.p = list(map(float, d["projection_matrix"]["data"]))
            self.get_logger().info(f"calibración cargada de {path}")
            return info
        except Exception as exc:
            self.get_logger().warn(f"no se pudo cargar la calibración: {exc}")
            return None

    # ----------------------------------------------------------------------
    def _capture_loop(self):
        consecutive_failures = 0
        while self.running_ and rclpy.ok():
            frame = self.source_.read()
            if frame is None or frame.jpeg is None:
                consecutive_failures += 1
                if consecutive_failures in (5, 50, 500):
                    self.get_logger().warn(
                        f"{consecutive_failures} fallos de captura seguidos"
                    )
                continue
            consecutive_failures = 0

            stamp = ns_to_time_msg(frame.stamp_ns)

            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = pack_frame_id(self.frame_id_, self.seq_)
            msg.format = "jpeg"
            msg.data = frame.jpeg
            self.publisher_.publish(msg)

            if self.info_pub_ is not None:
                self.info_.header.stamp = stamp
                self.info_.header.frame_id = self.frame_id_
                self.info_pub_.publish(self.info_)

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
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
