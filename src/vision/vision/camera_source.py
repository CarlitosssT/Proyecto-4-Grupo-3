#!/usr/bin/env python3
"""
Abstracción de fuente de cámara.

Va en:  src/vision/vision/camera_source.py

Motivo
------
`cv2.VideoCapture` NO sirve para una Camera Module 3. Desde Bullseye el stack
legacy de cámara de Raspberry Pi está eliminado: el nodo `/dev/video0` de una
CSI expone el Bayer crudo sin procesar por el ISP. Si abres eso con OpenCV
obtienes basura verde, no una imagen. Todo el procesado (debayer, balance de
blancos, control de exposición, corrección de viñeteo y, en la Module 3, el
autofoco por detección de fase) vive en libcamera.

Así que hay dos caminos de captura irreconciliables. La solución no es un `if`
dentro del nodo, sino una interfaz común con dos implementaciones. El nodo
publica frames y no sabe de dónde salen.

Interfaz
--------
Toda fuente devuelve un `Frame` con:
    stamp_ns  nanosegundos en el dominio de CLOCK_REALTIME (= reloj de ROS)
    jpeg      bytes JPEG listos para publicar, o None
    bgr       numpy array BGR, o None

Que `jpeg` pueda venir ya codificado es la decisión de diseño importante.
Permite rutas donde el frame NUNCA se descomprime:

  · USB en modo passthrough: la webcam ya entrega MJPEG, lo publicamos tal cual
  · CSI en modo hardware: el encoder JPEG del SoC produce los bytes directamente

En ambos casos la CPU de la Pi no toca un solo píxel. Si además necesitas el
array para procesar (ArUco, detección de obstáculos), pides `want_bgr=True` y
la fuente te da las dos cosas.

Especificación de fuente
------------------------
    "usb:0"   ->  V4L2Source(device=0)
    "csi:0"   ->  LibcameraSource(camera_num=0)
    "0"       ->  se asume usb (compatibilidad con el código viejo)
"""
from dataclasses import dataclass
from typing import Optional
import time

import numpy as np


# --------------------------------------------------------------------------
@dataclass
class Frame:
    stamp_ns: int
    jpeg: Optional[bytes] = None
    bgr: Optional[np.ndarray] = None
    width: int = 0
    height: int = 0


def realtime_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_REALTIME)


def boottime_to_realtime_offset_ns() -> int:
    """
    libcamera sella los frames con CLOCK_BOOTTIME. El reloj de ROS usa
    CLOCK_REALTIME. Sumar este offset convierte uno en otro.

    Sin esta conversión el `header.stamp` de la CSI queda desfasado por el
    uptime de la máquina — típicamente horas o días. Es un error silencioso:
    todo publica bien y tu sonda de latencia reporta números absurdos.

    Se calcula una vez al arrancar. Los dos relojes avanzan al mismo ritmo, el
    offset solo salta si alguien ajusta la hora del sistema (por ejemplo NTP
    corrigiendo al arrancar). Por eso: sincroniza el reloj ANTES de lanzar los
    nodos, no durante.
    """
    return time.clock_gettime_ns(time.CLOCK_REALTIME) - time.clock_gettime_ns(
        time.CLOCK_BOOTTIME
    )


# --------------------------------------------------------------------------
class CameraSource:
    """Interfaz común. No instanciar directamente."""

    def open(self) -> None:
        raise NotImplementedError

    def read(self, want_bgr: bool = False) -> Optional[Frame]:
        """Devuelve el frame más reciente, o None si falló la captura."""
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def describe(self) -> str:
        return self.__class__.__name__


# --------------------------------------------------------------------------
class V4L2Source(CameraSource):
    """
    Webcam USB vía OpenCV.

    Modos de codificación
    ---------------------
    "cpu"          read() -> BGR, luego cv2.imencode. Universal, cuesta CPU.
                   Si la cámara entrega MJPEG, OpenCV lo DECODIFICA y nosotros
                   lo volvemos a comprimir: se paga dos veces por nada.

    "passthrough"  Con CAP_PROP_CONVERT_RGB=0 y FOURCC=MJPG, OpenCV devuelve el
                   buffer MJPEG crudo del driver sin decodificar. Cero CPU.
                   Contras: la calidad la fija la webcam, `jpeg_quality` deja de
                   tener efecto, y no todos los drivers lo respetan. Por eso hay
                   verificación al abrir y caída automática a "cpu".
    """

    def __init__(self, device=0, width=640, height=480, fps=30,
                 jpeg_quality=80, encode_mode="cpu", flush_buffer=False):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.encode_mode = encode_mode
        self.flush_buffer = flush_buffer
        self.cap = None
        self._note = ""

    def open(self):
        import cv2

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"no se pudo abrir /dev/video{self.device}")

        # FOURCC antes que resolución: algunos drivers resetean el tamaño al
        # cambiar de formato.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self.encode_mode == "passthrough":
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            ok, probe = self.cap.read()
            # Un buffer MJPEG crudo llega como array 1xN de uint8 que empieza
            # con el marcador SOI de JPEG (0xFF 0xD8). Si en su lugar llega un
            # array HxWx3, el driver ignoró la petición.
            looks_raw = (
                ok and probe is not None and probe.ndim <= 2
                and probe.size > 2 and probe.flat[0] == 0xFF and probe.flat[1] == 0xD8
            )
            if not looks_raw:
                self._note = "passthrough no soportado por el driver, usando cpu"
                self.encode_mode = "cpu"
                self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.width
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.height

    def read(self, want_bgr=False):
        import cv2

        # Descartar el frame encolado sin decodificarlo. Barato, y evita que la
        # cola del driver crezca si el consumidor se atrasa.
        if self.flush_buffer:
            self.cap.grab()

        ok, data = self.cap.read()
        stamp = realtime_ns()
        if not ok or data is None:
            return None

        if self.encode_mode == "passthrough" and data.ndim <= 2:
            jpeg = data.tobytes()
            bgr = None
            if want_bgr:
                bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            return Frame(stamp, jpeg, bgr, self.width, self.height)

        bgr = data
        if (bgr.shape[1], bgr.shape[0]) != (self.width, self.height):
            bgr = cv2.resize(bgr, (self.width, self.height))
        ok_enc, buf = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok_enc:
            return None
        return Frame(stamp, buf.tobytes(), bgr if want_bgr else None,
                     self.width, self.height)

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def describe(self):
        s = (f"V4L2 /dev/video{self.device}  {self.width}x{self.height}@{self.fps} "
             f"modo={self.encode_mode} q={self.jpeg_quality}")
        return s + (f"  [{self._note}]" if self._note else "")


# --------------------------------------------------------------------------
class LibcameraSource(CameraSource):
    """
    Cámara CSI (Camera Module 3 / Module 3 Wide, sensor IMX708) vía Picamera2.

    Ajustes específicos de la Module 3 que NO existen en una webcam y que
    importan mucho en un rover:

    af_mode / lens_position
        La Module 3 tiene autofoco PDAF. En un rover en movimiento el autofoco
        continuo es activamente dañino: la cámara "caza" el foco cada vez que
        cambia la escena y pierdes medio segundo de imagen útil en cada
        búsqueda. Default aquí: MANUAL con el foco fijado.
        LensPosition se expresa en dioptrías (1/distancia_en_metros):
            0.0  = infinito
            0.5  = 2 m
            1.0  = 1 m
            3.0  = 33 cm
        Para un rover de exterior, 0.0 o 0.2 suele ser lo correcto.

    exposure_max_us
        Con poca luz el control automático alarga la exposición y aparece
        motion blur, que arruina cualquier detección posterior. Este límite
        obliga al ISP a subir ganancia en vez de tiempo. 4000 us (1/250 s) es
        un punto de partida razonable en movimiento.

    Modos de codificación
    ---------------------
    "cpu"       capture_array -> cv2.imencode. Funciona en cualquier Pi.
    "hardware"  MJPEGEncoder de Picamera2. En Pi 4 usa el encoder JPEG del SoC
                y la CPU queda casi libre. En Pi 5 no hay bloque de encode por
                hardware, así que Picamera2 cae a software; sigue siendo más
                eficiente que el camino cpu porque evita la conversión a BGR.
    """

    def __init__(self, camera_num=0, width=640, height=480, fps=30,
                 jpeg_quality=80, encode_mode="cpu",
                 af_mode="manual", lens_position=0.0, exposure_max_us=4000,
                 hflip=False, vflip=False):
        self.camera_num = camera_num
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.encode_mode = encode_mode
        self.af_mode = af_mode
        self.lens_position = lens_position
        self.exposure_max_us = exposure_max_us
        self.hflip = hflip
        self.vflip = vflip

        self.picam = None
        self._offset_ns = 0
        self._queue = None
        self._model = "?"

    def open(self):
        from picamera2 import Picamera2

        self._offset_ns = boottime_to_realtime_offset_ns()
        self.picam = Picamera2(self.camera_num)
        self._model = self.picam.camera_properties.get("Model", "?")

        frame_us = int(1_000_000 / self.fps)
        controls = {"FrameDurationLimits": (frame_us, frame_us)}

        # Techo de tiempo de exposición: menos motion blur en movimiento.
        if self.exposure_max_us:
            controls["ExposureTime"] = min(self.exposure_max_us, frame_us)

        # El autofoco solo existe si el sensor lo tiene. Un IMX219 (Camera
        # Module 2) o una HQ Camera no lo tienen y pedirlo lanza excepción.
        if "AfMode" in self.picam.camera_controls:
            try:
                from libcamera import controls as lc
                if self.af_mode == "manual":
                    controls["AfMode"] = lc.AfModeEnum.Manual
                    controls["LensPosition"] = float(self.lens_position)
                elif self.af_mode == "continuous":
                    controls["AfMode"] = lc.AfModeEnum.Continuous
                else:
                    controls["AfMode"] = lc.AfModeEnum.Auto
            except ImportError:
                pass

        if self.encode_mode == "hardware":
            # El encoder JPEG quiere YUV420. Pedirle RGB lo obliga a convertir
            # y se pierde la ventaja.
            cfg = self.picam.create_video_configuration(
                main={"size": (self.width, self.height), "format": "YUV420"},
                controls=controls,
                transform=self._transform(),
            )
        else:
            # RGB888 en Picamera2 entrega los bytes en orden BGR, que es
            # justo lo que OpenCV espera. El nombre es engañoso y es una
            # fuente clásica de imágenes con los colores invertidos.
            cfg = self.picam.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                controls=controls,
                transform=self._transform(),
            )

        self.picam.configure(cfg)

        if self.encode_mode == "hardware":
            self._start_hardware_encoder()
        else:
            self.picam.start()

    def _transform(self):
        try:
            from libcamera import Transform
            return Transform(hflip=int(self.hflip), vflip=int(self.vflip))
        except ImportError:
            return None

    def _start_hardware_encoder(self):
        import queue
        from picamera2.encoders import MJPEGEncoder
        from picamera2.outputs import Output

        self._queue = queue.Queue(maxsize=2)
        offset = self._offset_ns
        q = self._queue

        class QueueOutput(Output):
            # La firma de outputframe ha cambiado entre versiones de
            # Picamera2. El *args/**kwargs evita romper al actualizar.
            def outputframe(self, frame, keyframe=True, timestamp=None,
                            *args, **kwargs):
                stamp = (timestamp * 1000 + offset) if timestamp else realtime_ns()
                if q.full():
                    # Descartar el viejo, quedarse con el nuevo. Esta cola
                    # NUNCA debe crecer: es la misma lógica de KEEP_LAST(1)
                    # del QoS, aplicada dentro del proceso.
                    try:
                        q.get_nowait()
                    except Exception:
                        pass
                try:
                    q.put_nowait((stamp, bytes(frame)))
                except Exception:
                    pass

        self.picam.start_recording(
            MJPEGEncoder(q=self.jpeg_quality), QueueOutput()
        )

    def read(self, want_bgr=False):
        if self.encode_mode == "hardware":
            try:
                stamp, jpeg = self._queue.get(timeout=2.0)
            except Exception:
                return None
            bgr = None
            if want_bgr:
                import cv2
                bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            return Frame(stamp, jpeg, bgr, self.width, self.height)

        import cv2

        request = self.picam.capture_request()
        try:
            bgr = request.make_array("main")
            md = request.get_metadata()
        finally:
            # Si no liberas la request, el pool de buffers se agota y la
            # captura se cuelga en silencio a los pocos segundos.
            request.release()

        # SensorTimestamp es el instante REAL de exposición del sensor, no el
        # momento en que tu código recibió el buffer. Es el mejor sello de
        # tiempo disponible en todo el sistema, y la webcam no tiene nada
        # equivalente.
        ts = md.get("SensorTimestamp")
        stamp = (ts + self._offset_ns) if ts else realtime_ns()

        ok, buf = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return None
        return Frame(stamp, buf.tobytes(), bgr if want_bgr else None,
                     self.width, self.height)

    def close(self):
        if self.picam is not None:
            try:
                if self.encode_mode == "hardware":
                    self.picam.stop_recording()
                else:
                    self.picam.stop()
            finally:
                self.picam.close()
                self.picam = None

    def describe(self):
        return (f"libcamera cam{self.camera_num} ({self._model})  "
                f"{self.width}x{self.height}@{self.fps}  modo={self.encode_mode}  "
                f"af={self.af_mode} lens={self.lens_position}")


# --------------------------------------------------------------------------
def create_source(spec: str, **kwargs) -> CameraSource:
    """
    'usb:0' / 'csi:0' / '0' -> instancia de CameraSource.

    Los kwargs que no aplican a un tipo se descartan, para poder pasar el
    mismo diccionario de parámetros a cualquier fuente.
    """
    spec = str(spec).strip()
    if ":" in spec:
        kind, _, index = spec.partition(":")
    else:
        kind, index = "usb", spec
    kind = kind.lower()
    index = int(index)

    common = ("width", "height", "fps", "jpeg_quality", "encode_mode")

    if kind in ("usb", "v4l2"):
        allowed = common + ("flush_buffer",)
        kw = {k: v for k, v in kwargs.items() if k in allowed}
        return V4L2Source(device=index, **kw)

    if kind in ("csi", "libcamera", "picam"):
        allowed = common + ("af_mode", "lens_position", "exposure_max_us",
                            "hflip", "vflip")
        kw = {k: v for k, v in kwargs.items() if k in allowed}
        return LibcameraSource(camera_num=index, **kw)

    raise ValueError(f"tipo de fuente desconocido: '{kind}' (usa usb: o csi:)")
