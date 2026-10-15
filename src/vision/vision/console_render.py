#!/usr/bin/env python3
"""
Dibujo de la consola del operador. Sin ROS y sin ventana: recibe el último
cuadro y el estado, y devuelve la imagen a mostrar. Así se puede probar sin
pantalla.

Va en:  src/vision/vision/console_render.py

Criterios de diseño
-------------------
- El video ocupa toda la ventana y conserva su proporción (bandas negras si
  la ventana no es 16:9). Nada tapa el centro de la imagen.
- El estado se lee de un vistazo por COLOR, no leyendo números:
      verde   = todo bien
      ámbar   = acercándose al límite (señal débil, se pierden cuadros)
      rojo    = sin video; la imagen en pantalla es vieja
- Cuando el video se congela, la imagen se oscurece y aparece un aviso grande
  con cuántos segundos lleva congelada. Sin esto, un cuadro congelado es
  indistinguible de un rover detenido, y el operador sigue manejando a ciegas.
- Los umbrales de señal salen de lo medido: −70 dBm funcionaba (1 % de
  pérdida), −75 dBm perdía ~60 % de los cuadros.
"""
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

# Colores BGR
GREEN = (90, 200, 90)
AMBER = (0, 175, 255)
RED = (70, 70, 235)
WHITE = (240, 240, 240)
GREY = (170, 170, 170)
DARK = (25, 25, 25)

SIGNAL_OK_DBM = -65      # por encima: verde
SIGNAL_WARN_DBM = -72    # entre -65 y -72: ámbar; por debajo: rojo


@dataclass
class ConsoleState:
    topic: str = ""
    frame_age_s: Optional[float] = None       # None = nunca llegó un cuadro
    fps: Optional[float] = None
    expected_fps: float = 30.0
    loss_pct: Optional[float] = None
    mbps: Optional[float] = None
    kb_per_frame: Optional[float] = None
    link: dict = field(default_factory=dict)  # lo que publica el rover
    link_age_s: Optional[float] = None
    show_hud: bool = True
    show_guides: bool = False
    recording: bool = False
    rec_seconds: float = 0.0
    message: str = ""                          # aviso temporal (foto guardada...)
    stale_after_s: float = 0.5


def evaluate(st: ConsoleState):
    """Devuelve (texto, color) del estado general."""
    if st.frame_age_s is None:
        return "ESPERANDO VIDEO", GREY
    if st.frame_age_s > st.stale_after_s:
        return "SIN VIDEO", RED
    sig = st.link.get("signal_dbm") if st.link else None
    unstable = (
        (st.fps is not None and st.fps < 0.7 * st.expected_fps)
        or (st.loss_pct is not None and st.loss_pct == st.loss_pct and st.loss_pct > 5)
    )
    if unstable:
        return "INESTABLE", AMBER
    if sig is not None and sig < SIGNAL_WARN_DBM:
        return "SENAL DEBIL", AMBER
    return "OK", GREEN


def signal_level(dbm):
    if dbm is None:
        return 0, GREY
    if dbm > -55:
        return 4, GREEN
    if dbm > SIGNAL_OK_DBM:
        return 3, GREEN
    if dbm > SIGNAL_WARN_DBM:
        return 2, AMBER
    if dbm > -80:
        return 1, RED
    return 0, RED


# ---------------------------------------------------------------------------
def _text(img, s, org, scale, color=WHITE, thick=1, anchor="left"):
    """Texto con sombra para que se lea sobre cualquier fondo."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (w, h), _ = cv2.getTextSize(s, font, scale, thick)
    x, y = org
    if anchor == "center":
        x -= w // 2
    elif anchor == "right":
        x -= w
    cv2.putText(img, s, (x + 1, y + 1), font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), font, scale, color, thick, cv2.LINE_AA)
    return w, h


def _panel(img, x0, y0, x1, y1, alpha=0.55):
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(img.shape[1], x1), min(img.shape[0], y1)
    if x1 <= x0 or y1 <= y0:
        return
    roi = img[y0:y1, x0:x1]
    roi[:] = (roi * (1 - alpha)).astype(np.uint8)


def _pill(img, s, org, scale, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    thick = max(1, int(round(scale * 2)))
    (w, h), base = cv2.getTextSize(s, font, scale, thick)
    x, y = org
    pad = int(8 * scale / 0.6)
    cv2.rectangle(img, (x, y - h - pad), (x + w + 2 * pad, y + pad), color, -1)
    cv2.putText(img, s, (x + pad, y), font, scale, DARK, thick, cv2.LINE_AA)
    return w + 2 * pad


def _bars(img, x, y, level, color, u):
    for i in range(4):
        bh = int((i + 1) * 5 * u)
        bw = int(5 * u)
        x0 = x + i * int(8 * u)
        c = color if i < level else (90, 90, 90)
        cv2.rectangle(img, (x0, y - bh), (x0 + bw, y), c, -1)
    return 4 * int(8 * u)


def letterbox(frame, w, h):
    """Escala el cuadro para que quepa en w x h conservando la proporción."""
    canvas = np.zeros((h, w, 3), np.uint8)
    if frame is None:
        return canvas, (0, 0, w, h)
    fh, fw = frame.shape[:2]
    s = min(w / fw, h / fh)
    nw, nh = max(1, int(fw * s)), max(1, int(fh * s))
    x, y = (w - nw) // 2, (h - nh) // 2
    interp = cv2.INTER_LINEAR if s > 1 else cv2.INTER_AREA
    canvas[y:y + nh, x:x + nw] = cv2.resize(frame, (nw, nh), interpolation=interp)
    return canvas, (x, y, nw, nh)


# ---------------------------------------------------------------------------
def render(frame, st: ConsoleState, w: int, h: int):
    img, (vx, vy, vw, vh) = letterbox(frame, w, h)
    u = max(0.6, h / 720.0)            # unidad de escala: todo crece con la ventana
    status, color = evaluate(st)
    stale = st.frame_age_s is not None and st.frame_age_s > st.stale_after_s

    if stale:
        img = (img * 0.35).astype(np.uint8)

    # Guías: centro y tercios, para alinear la pala y juzgar distancias.
    if st.show_guides and frame is not None:
        g = (200, 200, 200)
        for k in (1, 2):
            cv2.line(img, (vx + vw * k // 3, vy), (vx + vw * k // 3, vy + vh), g, 1, cv2.LINE_AA)
            cv2.line(img, (vx, vy + vh * k // 3), (vx + vw, vy + vh * k // 3), g, 1, cv2.LINE_AA)
        cx, cy, r = vx + vw // 2, vy + vh // 2, int(18 * u)
        cv2.line(img, (cx - r, cy), (cx + r, cy), WHITE, 2, cv2.LINE_AA)
        cv2.line(img, (cx, cy - r), (cx, cy + r), WHITE, 2, cv2.LINE_AA)

    if st.show_hud:
        bar_h = int(46 * u)
        _panel(img, 0, 0, w, bar_h)
        base_y = int(31 * u)
        x = int(14 * u)
        x += _pill(img, status, (x, base_y), 0.62 * u, color) + int(18 * u)

        def field_(label, value, col=WHITE):
            nonlocal x
            lw, _ = _text(img, label, (x, base_y), 0.48 * u, GREY)
            x += lw + int(6 * u)
            vw_, _ = _text(img, value, (x, base_y), 0.6 * u, col, 2)
            x += vw_ + int(22 * u)

        if st.fps is not None and st.fps == st.fps:
            col = GREEN if st.fps >= 0.9 * st.expected_fps else AMBER if st.fps >= 0.7 * st.expected_fps else RED
            field_("FPS", f"{st.fps:4.1f}", col)
        if st.loss_pct is not None and st.loss_pct == st.loss_pct:
            col = GREEN if st.loss_pct < 1 else AMBER if st.loss_pct < 5 else RED
            field_("PERDIDA", f"{st.loss_pct:.0f}%", col)
        if st.mbps is not None and st.mbps == st.mbps:
            field_("VIDEO", f"{st.mbps:.1f} Mbps")

        # Lado derecho: el Wi-Fi del rover
        link = st.link or {}
        rx = w - int(14 * u)
        if link.get("connected"):
            dbm = link.get("signal_dbm")
            lvl, lcol = signal_level(dbm)
            band = "5 GHz" if (link.get("freq_mhz") or 0) >= 5000 else "2.4 GHz"
            tx = link.get("tx_mbps")
            info = f"{dbm:.0f} dBm  {band}" + (f"  subida {tx:.0f} Mbps" if tx else "")
            tw, _ = _text(img, info, (rx, base_y), 0.5 * u, lcol, 1, anchor="right")
            _bars(img, rx - tw - int(44 * u), base_y + int(2 * u), lvl, lcol, u)
        elif link and not link.get("connected"):
            _text(img, "rover sin Wi-Fi", (rx, base_y), 0.5 * u, RED, 1, anchor="right")
        elif st.link_age_s is None:
            _text(img, "sin datos del Wi-Fi", (rx, base_y), 0.5 * u, GREY, 1, anchor="right")

        # Pie: atajos de teclado
        foot = "F pantalla completa   H datos   G guias   S foto   R grabar   Q salir"
        _panel(img, 0, h - int(30 * u), w, h, 0.45)
        _text(img, foot, (int(14 * u), h - int(10 * u)), 0.45 * u, GREY)

    if st.recording:
        cx, cy = w - int(30 * u), int(78 * u) if st.show_hud else int(30 * u)
        cv2.circle(img, (cx, cy), int(9 * u), RED, -1)
        m, s = divmod(int(st.rec_seconds), 60)
        _text(img, f"REC {m:02d}:{s:02d}", (cx - int(18 * u), cy + int(7 * u)), 0.55 * u, RED, 2, anchor="right")

    # Avisos grandes en el centro
    if st.frame_age_s is None:
        _text(img, "Esperando video...", (w // 2, h // 2), 1.0 * u, WHITE, 2, anchor="center")
        _text(img, st.topic, (w // 2, h // 2 + int(40 * u)), 0.55 * u, GREY, 1, anchor="center")
    elif stale:
        cv2.rectangle(img, (0, 0), (w - 1, h - 1), RED, max(4, int(8 * u)))
        _text(img, "SIN VIDEO", (w // 2, h // 2), 1.6 * u, RED, 3, anchor="center")
        _text(img, f"imagen congelada hace {st.frame_age_s:.1f} s",
              (w // 2, h // 2 + int(46 * u)), 0.7 * u, WHITE, 2, anchor="center")
        _text(img, "detener el rover", (w // 2, h // 2 + int(82 * u)), 0.6 * u, AMBER, 2, anchor="center")

    if st.message:
        _text(img, st.message, (w // 2, h - int(48 * u)), 0.6 * u, WHITE, 2, anchor="center")

    return img
