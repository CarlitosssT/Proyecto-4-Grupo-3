#!/usr/bin/env python3
"""
Nodo de cámara — captura frames con OpenCV, los comprime a JPEG y los
publica como sensor_msgs/CompressedImage.

Pensado para lanzar varias instancias (una por cámara física) vía launch,
cada una con distinto `video_device` y su propio tópico de salida:
    ros2 run vision camera_node --ros-args -p video_device:=0 -p frame_rate:=24

El QoS se arma BEST_EFFORT / VOLATILE / KEEP_LAST(1): lo más parecido a UDP
que ofrece ROS 2. Para un stream de video en vivo interesa el frame más
reciente ya, no encolar ni reintentar entrega de frames viejos — eso solo
suma latencia sin aportar nada (a diferencia de un tópico de comandos,
donde sí importa que llegue todo).
"""
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


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("video_device", 0)
        self.declare_parameter("frame_rate", 24)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("topic", "")
        self.declare_parameter("frame_id", "camera_link")

        self.video_device_ = self.get_parameter("video_device").value
        frame_rate = self.get_parameter("frame_rate").value
        self.width_ = self.get_parameter("width").value
        self.height_ = self.get_parameter("height").value
        self.frame_id_ = self.get_parameter("frame_id").value

        topic = self.get_parameter("topic").value
        if not topic:
            topic = f"camera{self.video_device_}/image/compressed"

        self.bridge_ = CvBridge()
        self.camera_ = cv2.VideoCapture(self.video_device_)
        self.camera_.set(cv2.CAP_PROP_FRAME_WIDTH, self.width_)
        self.camera_.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height_)

        if not self.camera_.isOpened():
            self.get_logger().error(
                f"No se pudo abrir /dev/video{self.video_device_}"
            )

        self.publisher_ = self.create_publisher(
            CompressedImage, topic, camera_qos_profile()
        )
        self.timer_ = self.create_timer(1.0 / frame_rate, self.publish_frame)

        self.get_logger().info(
            f"camera_node listo — device={self.video_device_}  "
            f"topic={topic}  {frame_rate} fps  QoS=BEST_EFFORT/VOLATILE/KEEP_LAST(1)"
        )

    def publish_frame(self):
        ok, frame = self.camera_.read()
        if not ok:
            self.get_logger().warn(
                f"Frame perdido en /dev/video{self.video_device_}",
                throttle_duration_sec=5.0,
            )
            return

        if (frame.shape[1], frame.shape[0]) != (self.width_, self.height_):
            frame = cv2.resize(frame, (self.width_, self.height_))

        msg = self.bridge_.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id_
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.camera_.release()
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
