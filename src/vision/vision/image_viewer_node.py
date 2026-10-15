#!/usr/bin/env python3
"""
Consola del operador — corre en la ESTACIÓN BASE (tu PC).

Va en:  src/vision/vision/image_viewer_node.py

    ros2 launch vision base.launch.py

Ventana
-------
Abre a 1280x720 (el doble del video) y se puede redimensionar o maximizar;
el video se escala para llenarla sin deformarse. La barra de herramientas de
Qt que ocupaba espacio arriba ya no aparece.

Teclas
------
    F       pantalla completa / ventana
    H       ocultar / mostrar datos
    G       guías (centro y tercios) para alinear la pala
    S       foto del cuadro actual, a resolución original  -> ~/capturas
    R       empezar / parar grabación en video              -> ~/capturas
    Q, ESC  salir

Qué muestra
-----------
Barra superior: estado general por color (OK / SENAL DEBIL / INESTABLE /
SIN VIDEO), cuadros por segundo que llegan, porcentaje de cuadros perdidos,
ancho de banda del video, y la señal Wi-Fi del rover (barras, dBm, banda y
velocidad de subida), que publica el propio rover cada segundo.

Si deja de llegar video por más de medio segundo, la imagen se oscurece y
aparece "SIN VIDEO" con los segundos que lleva congelada.

No muestra "latencia": medirla desde aquí compara relojes de dos máquinas y
da números falsos (se llegó a ver −51 ms). Para medir latencia está la sonda
RTT (probe:=true en el rover, echo:=true aquí).
"""
import json
import os
import time
from datetime import datetime

# El backend Qt5 de OpenCV bajo Wayland deja la ventana atascada en 10x10.
# Forzar XWayland antes de la primera llamada de GUI lo evita. Solo en Linux:
# en Mac no existe xcb y la ventana no abriría.
import sys
if sys.platform.startswith("linux"):
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
from std_msgs.msg import String

import cv2
import numpy as np

from vision.link_metrics import parse_frame_id, RollingStats
from vision.console_render import ConsoleState, render


def camera_qos_profile() -> QoSProfile:
    # Debe coincidir con el publicador (BEST_EFFORT). Si se pide RELIABLE,
    # no llega nada y no hay error visible.
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def mono_ns():
    return time.monotonic_ns()


class ImageViewerNode(Node):
    def __init__(self):
        super().__init__("image_viewer_node")

        self.declare_parameter("topic", "csi0/image/compressed")
        self.declare_parameter("link_topic", "")        # vacío = <base>/link
        self.declare_parameter("hud", True)
        self.declare_parameter("scale", 2.0)             # tamaño inicial = video x scale
        self.declare_parameter("fullscreen", False)
        self.declare_parameter("snapshot_dir", "~/capturas")

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        topic = p("topic")
        link_topic = p("link_topic") or topic.rsplit("/image", 1)[0] + "/link"
        self.snap_dir_ = os.path.expanduser(p("snapshot_dir"))
        self.scale_ = float(p("scale"))

        self.st_ = ConsoleState(topic=topic, show_hud=bool(p("hud")))
        self.stats_ = RollingStats(3.0)
        self.frame_ = None
        self.last_rx_ = None
        self.link_rx_ = None
        self.new_frame_ = False
        self.msg_until_ = 0.0
        self.writer_ = None
        self.rec_start_ = 0.0
        self.should_quit_ = False
        self.fullscreen_ = False

        self.win_ = "Rover - camara"
        cv2.namedWindow(self.win_, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
        self.default_size_ = (int(640 * self.scale_), int(360 * self.scale_))
        cv2.resizeWindow(self.win_, *self.default_size_)
        if bool(p("fullscreen")):
            self._toggle_fullscreen()

        self.create_subscription(CompressedImage, topic, self._on_image, camera_qos_profile())
        self.create_subscription(String, link_topic, self._on_link, 1)

        self.get_logger().info(
            f"consola del operador: video {topic}  estado {link_topic}  "
            f"(F pantalla completa, H datos, G guias, S foto, R grabar, Q salir)"
        )

    # ------------------------------------------------------------------ ROS
    def _on_image(self, msg: CompressedImage):
        now = mono_ns()
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return  # JPEG incompleto: se cuenta como pérdida por la secuencia
        _, seq = parse_frame_id(msg.header.frame_id)
        self.stats_.add(now, 0.0, len(msg.data), seq)
        self.frame_ = img
        self.last_rx_ = now
        self.new_frame_ = True
        if self.writer_ is not None:
            self.writer_.write(img)

    def _on_link(self, msg: String):
        try:
            self.st_.link = json.loads(msg.data)
            self.link_rx_ = mono_ns()
            fps = self.st_.link.get("fps")
            if fps:
                self.st_.expected_fps = float(fps)
        except Exception:
            pass

    # ---------------------------------------------------------------- teclas
    def _toggle_fullscreen(self):
        self.fullscreen_ = not self.fullscreen_
        cv2.setWindowProperty(
            self.win_, cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN if self.fullscreen_ else cv2.WINDOW_NORMAL,
        )

    def _flash(self, text, seconds=2.5):
        self.st_.message = text
        self.msg_until_ = time.monotonic() + seconds

    def _snapshot(self):
        if self.frame_ is None:
            return
        os.makedirs(self.snap_dir_, exist_ok=True)
        path = os.path.join(self.snap_dir_, datetime.now().strftime("foto_%Y%m%d_%H%M%S.jpg"))
        cv2.imwrite(path, self.frame_)
        self._flash(f"foto guardada: {os.path.basename(path)}")
        self.get_logger().info(f"foto guardada en {path}")

    def _toggle_record(self):
        if self.writer_ is not None:
            self.writer_.release()
            self.writer_ = None
            self.st_.recording = False
            self._flash("grabacion guardada")
            return
        if self.frame_ is None:
            return
        os.makedirs(self.snap_dir_, exist_ok=True)
        path = os.path.join(self.snap_dir_, datetime.now().strftime("video_%Y%m%d_%H%M%S.mp4"))
        h, w = self.frame_.shape[:2]
        self.writer_ = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                                       self.st_.expected_fps, (w, h))
        self.rec_start_ = time.monotonic()
        self.st_.recording = True
        self.get_logger().info(f"grabando en {path}")

    def _handle_key(self, key):
        k = key & 0xFF
        if k in (ord("q"), ord("Q"), 27):
            self.should_quit_ = True
        elif k in (ord("f"), ord("F")):
            self._toggle_fullscreen()
        elif k in (ord("h"), ord("H")):
            self.st_.show_hud = not self.st_.show_hud
        elif k in (ord("g"), ord("G")):
            self.st_.show_guides = not self.st_.show_guides
        elif k in (ord("s"), ord("S")):
            self._snapshot()
        elif k in (ord("r"), ord("R")):
            self._toggle_record()

    # ---------------------------------------------------------------- dibujo
    def draw(self):
        now = mono_ns()
        self.stats_.prune(now)
        s = self.stats_.summary()
        st = self.st_
        st.frame_age_s = None if self.last_rx_ is None else (now - self.last_rx_) / 1e9
        st.link_age_s = None if self.link_rx_ is None else (now - self.link_rx_) / 1e9
        if st.link_age_s is not None and st.link_age_s > 3.0:
            st.link = {}          # dato viejo: mejor no mostrarlo que engañar
        if s.get("n", 0) > 1:
            st.fps, st.loss_pct = s["fps_rx"], s["loss_pct"]
            st.mbps, st.kb_per_frame = s["kbps"] / 1000.0, s["avg_kb"]
        else:
            st.fps = st.loss_pct = st.mbps = st.kb_per_frame = None
        st.rec_seconds = time.monotonic() - self.rec_start_ if st.recording else 0.0
        if st.message and time.monotonic() > self.msg_until_:
            st.message = ""

        try:
            _, _, w, h = cv2.getWindowImageRect(self.win_)
        except Exception:
            w, h = self.default_size_
        if w < 64 or h < 36:
            w, h = self.default_size_

        cv2.imshow(self.win_, render(self.frame_, st, w, h))
        self._handle_key(cv2.waitKey(1))

        # Cerrar con la X de la ventana también sale.
        try:
            if cv2.getWindowProperty(self.win_, cv2.WND_PROP_VISIBLE) < 1:
                self.should_quit_ = True
        except Exception:
            self.should_quit_ = True

    def destroy_node(self):
        if self.writer_ is not None:
            self.writer_.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    period = 1.0 / 30.0
    last_draw = 0.0
    try:
        # Se redibuja al llegar cada cuadro y también sin cuadros, para que
        # el aviso "SIN VIDEO" aparezca aunque no llegue nada.
        while rclpy.ok() and not node.should_quit_:
            rclpy.spin_once(node, timeout_sec=0.01)
            t = time.monotonic()
            if node.new_frame_ or t - last_draw > period:
                node.new_frame_ = False
                last_draw = t
                node.draw()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
