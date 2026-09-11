#!/usr/bin/env python3
"""
Nodo visor — se suscribe a un tópico sensor_msgs/CompressedImage y muestra
el video en una ventana de OpenCV, sin depender de rqt.

Uso:
    ros2 run vision image_viewer_node --ros-args -p topic:=/camera0/image/compressed
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
from cv_bridge import CvBridge

import cv2


def camera_qos_profile() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class ImageViewerNode(Node):
    def __init__(self):
        super().__init__("image_viewer_node")

        self.declare_parameter("topic", "camera0/image/compressed")
        topic = self.get_parameter("topic").value
        self.window_name_ = topic

        self.bridge_ = CvBridge()
        cv2.namedWindow(self.window_name_, cv2.WINDOW_AUTOSIZE)

        self.subscription_ = self.create_subscription(
            CompressedImage, topic, self.image_callback, camera_qos_profile()
        )

        self.get_logger().info(f"image_viewer_node escuchando en {topic}")

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge_.compressed_imgmsg_to_cv2(msg)
        cv2.imshow(self.window_name_, frame)
        # Procesa el loop de eventos de la ventana; sin esto no se refresca.
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
