#!/usr/bin/env python3
"""
Nodo visor — corre en la ESTACIÓN BASE (tu laptop), nunca en la Pi.

Va en:  src/vision/vision/image_viewer_node.py

La Pi corre Ubuntu Server, sin entorno gráfico. Este nodo abre una ventana,
así que allí fallaría. Y está bien: el visor pertenece al lado del operador.

Cambios respecto a la versión original
--------------------------------------
1. Sin cv_bridge. `compressed_imgmsg_to_cv2` hacía internamente un
   `cv2.imdecode` sobre los bytes del mensaje. Llamarlo directo es lo mismo
   con una dependencia menos.

2. HUD opcional sobre la imagen con fps, latencia y tamaño de frame. Así
   puedes ver la salud del enlace mientras conduces, sin una segunda terminal.
   Se apaga con `-p hud:=false`.

3. `q` o `ESC` cierran el nodo limpiamente.

Advertencia: `cv2.imshow` dentro del callback bloquea el ejecutor mientras
dibuja. Con un solo tópico es tolerable. Si algún día abres tres visores a la
vez, el cuello de botella va a ser tu GPU y no el enlace, y las medidas de
este HUD dejarán de ser fiables. Para medir en serio usa latency_probe_node,
que no dibuja nada.

Uso:
    ros2 run vision image_viewer_node --ros-args -p topic:=/usb0/image/compressed
"""
import os

# El backend Qt5 de OpenCV, bajo una sesión Wayland (GNOME), intenta usar el
# plugin nativo "wayland" y no sincroniza el tamaño real de la ventana con el
# compositor: la ventana queda visualmente atascada en 10x10 px aunque reciba
# frames. Forzar el plugin "xcb" (vía XWayland) antes de la primera llamada
# GUI de cv2 evita el problema.
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np

from vision.link_metrics import parse_frame_id, stamp_to_ns, RollingStats


def camera_qos_profile() -> QoSProfile:
    # Debe coincidir con el del publisher. Si el publisher es BEST_EFFORT y
    # aquí pides RELIABLE, el QoS es incompatible y no llega NADA, sin error
    # visible. Diagnóstico: ros2 topic info <tópico> --verbose
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class ImageViewerNode(Node):
    def __init__(self):
        super().__init__("image_viewer_node")

        self.declare_parameter("topic", "usb0/image/compressed")
        self.declare_parameter("hud", True)

        topic = self.get_parameter("topic").value
        self.hud_ = bool(self.get_parameter("hud").value)
        self.window_name_ = topic

        self.stats_ = RollingStats(3.0)
        self.should_quit_ = False

        cv2.namedWindow(self.window_name_, cv2.WINDOW_AUTOSIZE)

        self.subscription_ = self.create_subscription(
            CompressedImage, topic, self.image_callback, camera_qos_profile()
        )

        self.get_logger().info(
            f"image_viewer_node escuchando en {topic}  (q o ESC para salir)"
        )

    # ----------------------------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        t_rx = self.get_clock().now().nanoseconds
        _, seq = parse_frame_id(msg.header.frame_id)
        latency_ms = (t_rx - stamp_to_ns(msg.header.stamp)) / 1e6
        self.stats_.add(t_rx, latency_ms, len(msg.data), seq)

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            # Un JPEG truncado suele significar que se perdió un fragmento UDP
            # en el aire. Es información útil, no un error del código.
            self.get_logger().warn("frame corrupto", throttle_duration_sec=2.0)
            return

        if self.hud_:
            self._draw_hud(frame)

        cv2.imshow(self.window_name_, frame)
        # Procesa el loop de eventos de la ventana; sin esto no se refresca.
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.should_quit_ = True

    def _draw_hud(self, frame):
        s = self.stats_.summary()
        if s.get("n", 0) == 0:
            return

        lines = [
            f"{s['fps_rx']:.1f} fps   {s['kbps']/1000:.1f} Mbps",
            f"lat p50 {s['p50_ms']:.0f} / p95 {s['p95_ms']:.0f} ms",
            f"perdida {s['loss_pct']:.1f}%   {s['avg_kb']:.0f} KB/f",
        ]

        # Fondo semitransparente: sobre una escena clara, el texto blanco solo
        # es ilegible justo cuando más lo necesitas.
        overlay = frame.copy()
        cv2.rectangle(overlay, (6, 6), (270, 18 + 20 * len(lines)), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

        for i, text in enumerate(lines):
            cv2.putText(frame, text, (14, 26 + 20 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    # ----------------------------------------------------------------------
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()

    try:
        # spin_once en bucle en vez de spin(): permite salir con la tecla q
        # sin tener que matar el proceso desde fuera.
        while rclpy.ok() and not node.should_quit_:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
