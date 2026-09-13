#!/usr/bin/env python3
"""
Utilidades compartidas para instrumentar el enlace de video.

Va en:  src/vision/vision/link_metrics.py

No es un nodo. Lo importan camera_node, latency_probe_node, link_echo_node
y rtt_probe_node.

Truco de la secuencia
---------------------
En ROS 2 el `std_msgs/Header` YA NO tiene campo `seq` (existía en ROS 1 y se
eliminó). Sin número de secuencia es imposible medir pérdida de frames: solo
ves los que llegan, nunca sabes cuántos faltaron.

Para no tener que definir un mensaje custom (lo que obligaría a un paquete
ament_cmake aparte), el contador viaja incrustado en `header.frame_id`:

    frame_id = "camera_link|1523"

`pack_frame_id` / `parse_frame_id` hacen el empaquetado y el desempaquetado.
Es un hack consciente: si más adelante necesitas TF real sobre ese frame_id,
migra a un mensaje propio.
"""
from collections import deque


SEQ_SEP = "|"


# --------------------------------------------------------------------------
# Secuencia incrustada en frame_id
# --------------------------------------------------------------------------
def pack_frame_id(frame_id: str, seq: int) -> str:
    """'camera_link', 42  ->  'camera_link|42'"""
    return f"{frame_id}{SEQ_SEP}{seq}"


def parse_frame_id(raw: str):
    """
    'camera_link|42'  ->  ('camera_link', 42)
    'camera_link'     ->  ('camera_link', None)   # publisher sin instrumentar

    Usa rpartition para que un frame_id que ya contenga '|' no rompa nada.
    """
    if SEQ_SEP not in raw:
        return raw, None
    base, _, tail = raw.rpartition(SEQ_SEP)
    try:
        return base, int(tail)
    except ValueError:
        return raw, None


# --------------------------------------------------------------------------
# Conversión de tiempo
# --------------------------------------------------------------------------
def stamp_to_ns(stamp) -> int:
    """builtin_interfaces/Time -> nanosegundos (int)."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_ms(ns: int) -> float:
    return ns / 1_000_000.0


def percentile(sorted_values, q: float):
    """
    Percentil por interpolación lineal. Sin numpy, para que esto corra en
    una Raspberry Pi pelada sin instalar nada.

    `sorted_values` debe venir YA ordenado. q en [0, 100].
    """
    if not sorted_values:
        return float("nan")
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    pos = (len(sorted_values) - 1) * (q / 100.0)
    low = int(pos)
    high = min(low + 1, len(sorted_values) - 1)
    frac = pos - low
    return sorted_values[low] * (1.0 - frac) + sorted_values[high] * frac


# --------------------------------------------------------------------------
# Ventana deslizante de métricas
# --------------------------------------------------------------------------
class RollingStats:
    """
    Acumula muestras de los últimos `window_sec` segundos y calcula el
    resumen que de verdad describe un enlace de video inalámbrico:

      fps_rx    frames que LLEGAN por segundo (no los que se publican)
      p50 / p95 / max de latencia, en ms
      jitter    desviación media del intervalo entre llegadas, en ms
      loss_pct  % de frames publicados que nunca llegaron
      kbps      throughput útil a nivel de aplicación

    El p95 importa más que el promedio: teleoperar un rover es aceptable si
    la latencia TÍPICA es baja, pero se vuelve peligroso si uno de cada
    veinte frames llega 800 ms tarde. El promedio esconde eso.
    """

    def __init__(self, window_sec: float = 5.0):
        self.window_ns = int(window_sec * 1e9)
        self.samples = deque()      # (t_rx_ns, latency_ms, size_bytes, seq)
        self.total_rx = 0
        self.total_lost = 0
        self._last_seq = None

    def add(self, t_rx_ns: int, latency_ms: float, size_bytes: int, seq):
        self.total_rx += 1

        # Pérdida acumulada global (independiente de la ventana)
        if seq is not None and self._last_seq is not None:
            gap = seq - self._last_seq
            if gap > 1:
                self.total_lost += gap - 1
        if seq is not None:
            self._last_seq = seq

        self.samples.append((t_rx_ns, latency_ms, size_bytes, seq))
        cutoff = t_rx_ns - self.window_ns
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def summary(self) -> dict:
        n = len(self.samples)
        if n == 0:
            return {"n": 0}

        t_first = self.samples[0][0]
        t_last = self.samples[-1][0]
        span_s = max((t_last - t_first) / 1e9, 1e-9)

        lat = sorted(s[1] for s in self.samples)
        total_bytes = sum(s[2] for s in self.samples)

        # Jitter: variación del intervalo entre llegadas consecutivas
        gaps = [
            (self.samples[i][0] - self.samples[i - 1][0]) / 1e6
            for i in range(1, n)
        ]
        if gaps:
            mean_gap = sum(gaps) / len(gaps)
            jitter = sum(abs(g - mean_gap) for g in gaps) / len(gaps)
        else:
            jitter = float("nan")

        # Pérdida dentro de la ventana
        seqs = [s[3] for s in self.samples if s[3] is not None]
        if len(seqs) >= 2:
            expected = max(seqs) - min(seqs) + 1
            loss_pct = 100.0 * (expected - len(seqs)) / expected
        else:
            loss_pct = float("nan")

        return {
            "n": n,
            "fps_rx": (n - 1) / span_s if n > 1 else float("nan"),
            "p50_ms": percentile(lat, 50),
            "p95_ms": percentile(lat, 95),
            "max_ms": lat[-1],
            "jitter_ms": jitter,
            "loss_pct": loss_pct,
            "kbps": (total_bytes * 8 / 1000.0) / span_s,
            "avg_kb": (total_bytes / n) / 1024.0,
        }

    @staticmethod
    def format(s: dict) -> str:
        if s.get("n", 0) == 0:
            return "sin datos en la ventana (¿el publisher está vivo?)"
        return (
            f"fps_rx={s['fps_rx']:5.1f}  "
            f"lat p50={s['p50_ms']:6.1f} p95={s['p95_ms']:6.1f} max={s['max_ms']:6.1f} ms  "
            f"jitter={s['jitter_ms']:5.1f} ms  "
            f"loss={s['loss_pct']:5.1f}%  "
            f"{s['kbps']:7.0f} kbps  ({s['avg_kb']:.1f} KB/frame)"
        )


# --------------------------------------------------------------------------
# CSV
# --------------------------------------------------------------------------
class CsvLogger:
    """
    Escritor CSV con flush periódico.

    El flush importa: si matas el nodo con Ctrl-C en pleno campo, sin flush
    pierdes hasta 8 KB de buffer, que a 24 fps son varios segundos de datos.
    """

    def __init__(self, path: str, fieldnames, flush_every: int = 30):
        import csv
        import os

        self.path = path
        if path:
            os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
            self._fh = open(path, "w", newline="")
            self._w = csv.DictWriter(self._fh, fieldnames=fieldnames)
            self._w.writeheader()
        else:
            self._fh = None
            self._w = None
        self._n = 0
        self._flush_every = flush_every

    @property
    def enabled(self) -> bool:
        return self._w is not None

    def write(self, row: dict):
        if self._w is None:
            return
        self._w.writerow(row)
        self._n += 1
        if self._n % self._flush_every == 0:
            self._fh.flush()

    def close(self):
        if self._fh is not None:
            self._fh.flush()
            self._fh.close()
            self._fh = None
            self._w = None
