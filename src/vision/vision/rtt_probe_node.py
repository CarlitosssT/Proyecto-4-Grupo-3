#!/usr/bin/env python3
"""
Sonda de RTT — corre en el ROVER (la Raspberry), junto al camera_node.

Va en:  src/vision/vision/rtt_probe_node.py

Pareja de link_echo_node. Recibe los headers que la base devuelve y calcula:

    RTT = reloj_del_rover_ahora  -  header.stamp (puesto por el rover)

Un solo reloj en ambos extremos de la resta => cero dependencia de
sincronización. Esta es la medición en la que puedes confiar en campo.

También reporta la tasa de eco: si publicas a 24 fps y solo vuelven 9 ecos
por segundo, has perdido el 62 % de los frames en el enlace, y ESA es tu
señal de que el enlace no da más — mucho antes de que la latencia suba.

Uso
---
    ros2 run vision rtt_probe_node --ros-args \
        -p csv_path:=/tmp/rtt_d030m.csv -p label:=30m -p expected_fps:=24.0
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import Header

from vision.link_metrics import (
    parse_frame_id,
    stamp_to_ns,
    RollingStats,
    CsvLogger,
)


def best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class RttProbeNode(Node):
    def __init__(self):
        super().__init__("rtt_probe_node")

        self.declare_parameter("echo_topic", "link_echo")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("label", "")
        self.declare_parameter("window_sec", 5.0)
        self.declare_parameter("report_sec", 2.0)
        self.declare_parameter("expected_fps", 24.0)

        echo_topic = self.get_parameter("echo_topic").value
        self.label_ = self.get_parameter("label").value
        self.expected_fps_ = float(self.get_parameter("expected_fps").value)

        self.stats_ = RollingStats(self.get_parameter("window_sec").value)
        self.csv_ = CsvLogger(
            self.get_parameter("csv_path").value,
            ["label", "seq", "t_tx_ns", "t_echo_rx_ns", "rtt_ms"],
        )

        self.subscription_ = self.create_subscription(
            Header, echo_topic, self.echo_callback, best_effort_qos()
        )
        self.timer_ = self.create_timer(
            self.get_parameter("report_sec").value, self.report
        )

        self.get_logger().info(
            f"rtt_probe_node escuchando ecos en {echo_topic}"
            + (f"  label={self.label_}" if self.label_ else "")
        )

    # ----------------------------------------------------------------------
    def echo_callback(self, msg: Header):
        t_now_ns = self.get_clock().now().nanoseconds
        t_tx_ns = stamp_to_ns(msg.stamp)
        _, seq = parse_frame_id(msg.frame_id)

        rtt_ms = (t_now_ns - t_tx_ns) / 1e6

        # size_bytes=0: aquí no medimos throughput, solo tiempo de ida y vuelta.
        self.stats_.add(t_now_ns, rtt_ms, 0, seq)

        self.csv_.write({
            "label": self.label_,
            "seq": seq if seq is not None else "",
            "t_tx_ns": t_tx_ns,
            "t_echo_rx_ns": t_now_ns,
            "rtt_ms": round(rtt_ms, 3),
        })

    # ----------------------------------------------------------------------
    def report(self):
        self.stats_.prune(self.get_clock().now().nanoseconds)
        s = self.stats_.summary()
        prefix = f"[{self.label_}] " if self.label_ else ""

        if s.get("n", 0) == 0:
            self.get_logger().warn(
                prefix + "sin ecos — el enlace está caído o link_echo_node no corre"
            )
            return

        # Tasa de entrega: qué fracción de lo publicado sobrevivió el viaje.
        delivery = 100.0 * s["fps_rx"] / self.expected_fps_ if self.expected_fps_ else float("nan")

        self.get_logger().info(
            prefix
            + f"eco={s['fps_rx']:5.1f}/s ({delivery:5.1f}% de {self.expected_fps_:.0f} fps)  "
            + f"RTT p50={s['p50_ms']:6.1f} p95={s['p95_ms']:6.1f} max={s['max_ms']:6.1f} ms  "
            + f"jitter={s['jitter_ms']:5.1f} ms  loss={s['loss_pct']:5.1f}%"
        )

    def destroy_node(self):
        print(
            f"TOTAL ecos={self.stats_.total_rx}  huecos={self.stats_.total_lost}"
        )
        self.csv_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RttProbeNode()
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
