#!/usr/bin/env python3
"""
Fuentes de cámara.

Va en:  src/vision/vision/camera_source.py

Dos implementaciones detrás de la misma interfaz:

    "csi:0"  ->  RpicamSource   Camera Module 3 / 3 Wide (la definitiva)
    "usb:0"  ->  V4L2Source     webcam USB (respaldo)

Por qué la CSI se lee a través de rpicam-vid y no de Python
-----------------------------------------------------------
La cámara CSI no se puede abrir con cv2.VideoCapture: su /dev/video entrega
datos Bayer crudos sin pasar por el ISP, y lo que sale es una imagen verde.
Todo el procesado (debayer, balance de blancos, exposición, autofoco) vive en
libcamera.

La vía "oficial" desde Python es picamera2, pero en Ubuntu 24.04 no está en
los repositorios y depende de los bindings de Python de libcamera compilados
con exactamente la misma versión. Es la pieza que más falla en esta
distribución.

rpicam-vid, en cambio, es un programa de línea de comandos que se compila
junto con libcamera y funciona en cuanto rpicam-hello funciona. Se le pide
que codifique MJPEG y escriba a su salida estándar; este módulo lee esa
salida, separa los cuadros JPEG y se los entrega al nodo.

Ventaja adicional: rpicam-vid usa el codificador JPEG por hardware de la
Pi 4, así que la CPU no comprime nada.

Interfaz común
--------------
    open()          arranca la cámara
    read()          devuelve el cuadro MÁS RECIENTE (Frame) o None
    close()         la detiene
    describe()      texto para el log
"""
import os
import shutil
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass
class Frame:
    stamp_ns: int          # CLOCK_REALTIME, el mismo reloj que usa ROS
    jpeg: bytes            # listo para publicar
    width: int = 0
    height: int = 0


def realtime_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_REALTIME)


class CameraSource:
    def open(self) -> None:
        raise NotImplementedError

    def read(self) -> Optional[Frame]:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def describe(self) -> str:
        return self.__class__.__name__


# ==========================================================================
# Separador de cuadros MJPEG
# ==========================================================================
class MjpegSplitter:
    """
    Recibe bytes en trozos arbitrarios y devuelve cuadros JPEG completos.

    Un flujo MJPEG es simplemente un JPEG detrás de otro. Cada uno empieza
    con el marcador SOI (FF D8) y termina con EOI (FF D9).

    No basta con buscar FF D9: esa secuencia puede aparecer por casualidad
    dentro de las tablas de cuantización de la cabecera. Por eso se recorren
    los segmentos de cabecera usando su longitud declarada hasta llegar a SOS
    (FF DA). A partir de ahí empiezan los datos comprimidos, donde el
    estándar JPEG garantiza que un FF real siempre va seguido de 00 o de un
    marcador de reinicio. Solo en esa zona FF D9 significa "fin de cuadro".
    """

    SOI = b"\xff\xd8"
    EOI = b"\xff\xd9"

    def __init__(self, max_buffer=8 * 1024 * 1024):
        self.buf = bytearray()
        self.max_buffer = max_buffer

    def feed(self, chunk: bytes):
        self.buf += chunk
        frames = []
        while True:
            frame = self._extract_one()
            if frame is None:
                break
            frames.append(frame)
        # Protección: si llega basura y nunca aparece un cuadro, no crecer
        # sin límite.
        if len(self.buf) > self.max_buffer:
            self.buf.clear()
        return frames

    def _extract_one(self):
        buf = self.buf
        start = buf.find(self.SOI)
        if start < 0:
            # Conservar el último byte por si el FF D8 llegó partido.
            if len(buf) > 1:
                del buf[:-1]
            return None
        if start > 0:
            del buf[:start]

        # Recorrer segmentos de cabecera: FF xx LL LL <datos>
        pos = 2
        while True:
            if pos + 4 > len(buf):
                return None                      # cabecera incompleta
            if buf[pos] != 0xFF:
                # Flujo corrupto: descartar este SOI y buscar el siguiente.
                del buf[:2]
                return self._extract_one()
            marker = buf[pos + 1]
            if marker == 0xFF:                   # relleno permitido
                pos += 1
                continue
            seg_len = (buf[pos + 2] << 8) | buf[pos + 3]
            if marker == 0xDA:                   # SOS: empiezan los datos
                data_start = pos + 2 + seg_len
                break
            pos += 2 + seg_len

        end = buf.find(self.EOI, data_start)
        if end < 0:
            return None                          # cuadro aún incompleto
        frame = bytes(buf[: end + 2])
        del buf[: end + 2]
        return frame


# ==========================================================================
# CSI: Camera Module 3 vía rpicam-vid
# ==========================================================================
class RpicamSource(CameraSource):
    """
    Parámetros propios de la Camera Module 3 / 3 Wide:

    sensor_mode
        El sensor IMX708 tiene varios modos de lectura y NO todos ven el
        campo completo. Algunos recortan el centro para ganar velocidad, lo
        que en la versión Wide tira a la basura el ángulo que se pagó.
        "2304:1296" es el modo con binning 2x2 que conserva el campo completo
        y llega a unos 56 fps. Se verifica con `rpicam-hello --list-cameras`
        (columna "crop"). Vacío = que rpicam elija.

    width / height
        El sensor es 16:9. Pedir 640x480 (4:3) hace que libcamera RECORTE
        los lados para cambiar la proporción, y se pierde campo horizontal.
        Por eso el valor por defecto es 640x360.

    af_mode / lens_position
        Autofoco. En un rover en movimiento el autofoco continuo "caza" el
        foco en cada bache y se pierde imagen útil. Por defecto: manual, con
        la lente fija. lens_position está en dioptrías (1 / metros):
            0.0 = infinito    0.5 = 2 m    1.0 = 1 m    2.0 = 50 cm
        Vacío af_mode = no tocar el enfoque (cámaras sin autofoco).

    shutter_us
        0 = exposición automática. Con --framerate fijo, rpicam no puede
        alargar la exposición más allá de la duración de un cuadro, así que
        los fps no caen en poca luz (el problema que tenía la Brio).
        Si hay desenfoque por movimiento, fijarlo en 4000-8000.

    tuning_file
        Calibración de color y sombreado de lente. rpicam la elige sola según
        el modelo que detecta. Si `rpicam-hello --list-cameras` dice
        "imx708" en vez de "imx708_wide", está usando la de la lente normal y
        la imagen sale con las esquinas oscuras; en ese caso pasar la ruta
        del archivo imx708_wide.json. Vacío = automático.
    """

    def __init__(self, camera_num=0, width=640, height=360, fps=30,
                 jpeg_quality=70, sensor_mode="2304:1296", af_mode="manual",
                 lens_position=0.0, shutter_us=0, hflip=False, vflip=False,
                 tuning_file="", **_ignored):
        self.camera_num = int(camera_num)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.jpeg_quality = int(jpeg_quality)
        self.sensor_mode = str(sensor_mode or "")
        self.af_mode = str(af_mode or "")
        self.lens_position = float(lens_position)
        self.shutter_us = int(shutter_us)
        self.hflip = bool(hflip)
        self.vflip = bool(vflip)
        self.tuning_file = str(tuning_file or "")

        self.proc = None
        self._splitter = MjpegSplitter()
        self._cond = threading.Condition()
        self._latest = None          # (stamp_ns, jpeg)
        self._count = 0
        self._last_read = 0
        self._stderr_tail = deque(maxlen=25)
        self._running = False

    # ----------------------------------------------------------------------
    def _executable(self):
        exe = shutil.which("rpicam-vid") or shutil.which("libcamera-vid")
        if not exe:
            raise RuntimeError(
                "no se encontró rpicam-vid. Hay que compilar libcamera y "
                "rpicam-apps (ver instalar_camara_csi.sh)."
            )
        return exe

    def command(self):
        cmd = [
            self._executable(),
            "--camera", str(self.camera_num),
            "-t", "0",                    # sin límite de tiempo
            "--nopreview",
            "--codec", "mjpeg",
            "--width", str(self.width),
            "--height", str(self.height),
            "--framerate", str(self.fps),
            "--quality", str(self.jpeg_quality),
            "--flush",                    # no retener cuadros en el búfer
            "-o", "-",                    # a la salida estándar
        ]
        if self.sensor_mode:
            cmd += ["--mode", self.sensor_mode]
        if self.af_mode:
            cmd += ["--autofocus-mode", self.af_mode]
            if self.af_mode == "manual":
                cmd += ["--lens-position", f"{self.lens_position:g}"]
        if self.shutter_us > 0:
            cmd += ["--shutter", str(self.shutter_us)]
        if self.hflip:
            cmd += ["--hflip"]
        if self.vflip:
            cmd += ["--vflip"]
        if self.tuning_file:
            cmd += ["--tuning-file", self.tuning_file]
        return cmd

    def open(self):
        self._running = True
        self.proc = subprocess.Popen(
            self.command(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            # Grupo de procesos propio: el Ctrl+C de la terminal no le llega
            # directo a rpicam-vid. Lo detiene el nodo al cerrar, en orden.
            start_new_session=True,
        )
        threading.Thread(target=self._read_stdout, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()

        # Esperar el primer cuadro: si la cámara no arranca, fallar aquí con
        # un mensaje claro y no después con un silencio.
        with self._cond:
            ok = self._cond.wait_for(
                lambda: self._count > 0 or self.proc.poll() is not None,
                timeout=10.0,
            )
        if self._count == 0:
            self.close()
            raise RuntimeError(
                "rpicam-vid no entregó ningún cuadro"
                + ("" if ok else " en 10 s")
                + ". Últimas líneas:\n  " + "\n  ".join(self._stderr_tail)
            )

    def _read_stdout(self):
        fd = self.proc.stdout.fileno()
        while self._running:
            try:
                chunk = os.read(fd, 65536)
            except OSError:
                break
            if not chunk:
                break
            for jpeg in self._splitter.feed(chunk):
                stamp = realtime_ns()
                with self._cond:
                    # Solo se guarda el más reciente. Si el consumidor se
                    # atrasa, los cuadros viejos se descartan en vez de
                    # acumularse: la misma lógica que KEEP_LAST(1).
                    self._latest = (stamp, jpeg)
                    self._count += 1
                    self._cond.notify_all()
        with self._cond:
            self._cond.notify_all()

    def _read_stderr(self):
        for raw in iter(self.proc.stderr.readline, b""):
            line = raw.decode(errors="replace").rstrip()
            if line:
                self._stderr_tail.append(line)

    def read(self):
        with self._cond:
            got = self._cond.wait_for(
                lambda: self._count > self._last_read or self.proc.poll() is not None,
                timeout=2.0,
            )
            if self.proc.poll() is not None:
                raise RuntimeError(
                    "rpicam-vid terminó (código %s). Últimas líneas:\n  %s"
                    % (self.proc.returncode, "\n  ".join(self._stderr_tail))
                )
            if not got or self._latest is None:
                return None
            self._last_read = self._count
            stamp, jpeg = self._latest
        return Frame(stamp, jpeg, self.width, self.height)

    def close(self):
        self._running = False
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def describe(self):
        af = f"af={self.af_mode or 'sin tocar'}"
        if self.af_mode == "manual":
            af += f" lente={self.lens_position:g}"
        return (f"CSI cam{self.camera_num}  {self.width}x{self.height}@{self.fps}  "
                f"q={self.jpeg_quality}  modo={self.sensor_mode or 'auto'}  {af}  "
                f"shutter={'auto' if self.shutter_us <= 0 else self.shutter_us}")


# ==========================================================================
# USB: webcam vía OpenCV (respaldo)
# ==========================================================================
class V4L2Source(CameraSource):
    """
    Webcam USB. Dos valores medidos en este proyecto:

      CAP_PROP_BUFFERSIZE = 2   Con 1, el driver descarta uno de cada dos
                                cuadros y se obtienen exactamente la mitad
                                de los fps (15 en vez de 30).
      sin grab() de descarte    Con el bucle de captura en su propio hilo la
                                cola no crece, y el grab() solo tiraba
                                cuadros buenos.
    """

    def __init__(self, device=0, width=640, height=480, fps=30,
                 jpeg_quality=70, **_ignored):
        self.device = int(device)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.jpeg_quality = int(jpeg_quality)
        self.cap = None

    def open(self):
        import cv2
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"no se pudo abrir /dev/video{self.device}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.width
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.height

    def read(self):
        import cv2
        ok, img = self.cap.read()
        stamp = realtime_ns()
        if not ok or img is None:
            return None
        ok, buf = cv2.imencode(".jpg", img,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return None
        return Frame(stamp, buf.tobytes(), self.width, self.height)

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def describe(self):
        return (f"USB /dev/video{self.device}  {self.width}x{self.height}@{self.fps}  "
                f"q={self.jpeg_quality}")


# ==========================================================================
def create_source(spec: str, **kwargs) -> CameraSource:
    """'csi:0' -> RpicamSource,  'usb:0' o '0' -> V4L2Source."""
    spec = str(spec).strip().lower()
    kind, _, index = spec.partition(":") if ":" in spec else ("usb", "", spec)
    if kind in ("csi", "libcamera", "rpicam"):
        return RpicamSource(camera_num=int(index or 0), **kwargs)
    if kind in ("usb", "v4l2"):
        return V4L2Source(device=int(index or 0), **kwargs)
    raise ValueError(f"fuente desconocida: '{spec}' (usa csi:0 o usb:0)")
