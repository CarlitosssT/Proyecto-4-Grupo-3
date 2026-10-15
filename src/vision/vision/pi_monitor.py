#!/usr/bin/env python3
"""
Registro del estado de la Raspberry mientras transmite video.

Va en:  src/vision/vision/pi_monitor.py

Lo lanza solo `rover.launch.py probe:=true`; no hace falta correrlo a mano.
Cada `period` segundos escribe una fila con:

    temperatura, CPU total, CPU de rpicam-vid, CPU del camera_node,
    frecuencia de la CPU, bajo voltaje, memoria, señal Wi-Fi,
    velocidades de subida/bajada y reintentos de transmisión.

Responde a "¿la Pi aguanta?": si la temperatura sube hasta que baja la
frecuencia, o si hay bajo voltaje, los cuadros se pierden por la Pi y no
por el Wi-Fi, y una antena no lo arreglaría.

A mano (por ejemplo, para dejarlo corriendo mientras se usa la cámara):

    ros2 run vision pi_monitor --csv ~/medidas/pi_prueba.csv

Solo usa la librería estándar de Python.
"""
import argparse
import csv
import glob
import os
import re
import subprocess
import time

from vision.wifi_status import detect_wifi_iface, read_wifi

CLK_TCK = os.sysconf("SC_CLK_TCK")
FIELDS = [
    "label", "t_unix", "temp_c", "cpu_total_pct", "cpu_rpicam_pct",
    "cpu_camera_node_pct", "freq_mhz", "undervoltage", "mem_used_mb",
    "iface", "signal_dbm", "tx_mbps", "rx_mbps", "tx_retries", "tx_failed",
]


def _read(path):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return None


def temp_c():
    v = _read("/sys/class/thermal/thermal_zone0/temp")
    return round(int(v) / 1000.0, 1) if v else None


def freq_mhz():
    v = _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    return int(v) // 1000 if v else None


def undervoltage():
    """1 si el sensor de voltaje de la Pi marca bajo voltaje ahora, 0 si no, None si no hay sensor."""
    for d in glob.glob("/sys/class/hwmon/hwmon*"):
        if _read(f"{d}/name") == "rpi_volt":
            v = _read(f"{d}/in0_lcrit_alarm")
            return int(v) if v is not None else None
    return None


def mem_used_mb():
    info = {}
    for line in (_read("/proc/meminfo") or "").splitlines():
        k, _, rest = line.partition(":")
        info[k] = int(rest.split()[0]) if rest.split() else 0
    if "MemTotal" in info and "MemAvailable" in info:
        return (info["MemTotal"] - info["MemAvailable"]) // 1024
    return None


def cpu_times():
    """(ocupado, total) en ticks, sumando todos los núcleos."""
    parts = (_read("/proc/stat") or "cpu 0 0 0 0").splitlines()[0].split()[1:]
    vals = [int(x) for x in parts]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals) - idle, sum(vals)


def find_pids():
    """{'rpicam': [pids], 'camera_node': [pids]}"""
    out = {"rpicam": [], "camera_node": []}
    for p in os.listdir("/proc"):
        if not p.isdigit():
            continue
        try:
            with open(f"/proc/{p}/cmdline", "rb") as fh:
                argv = [a.decode(errors="replace") for a in fh.read().split(b"\0") if a]
        except OSError:
            continue
        if not argv:
            continue
        # argv[0] es el programa; argv[1] cuenta cuando es un script de Python.
        names = [os.path.basename(a) for a in argv[:2]]
        if "rpicam-vid" in names:
            out["rpicam"].append(int(p))
        elif "camera_node" in names:
            out["camera_node"].append(int(p))
    return out


def proc_ticks(pid):
    s = _read(f"/proc/{pid}/stat")
    if not s:
        return None
    # El nombre del proceso va entre paréntesis y puede tener espacios.
    fields = s[s.rfind(")") + 2:].split()
    return int(fields[11]) + int(fields[12])  # utime + stime


def tx_counters(iface):
    try:
        out = subprocess.run(["iw", "dev", iface, "station", "dump"],
                             capture_output=True, text=True, timeout=1).stdout
    except Exception:
        return None, None
    r = re.search(r"tx retries:\s*(\d+)", out)
    f = re.search(r"tx failed:\s*(\d+)", out)
    return (int(r.group(1)) if r else None, int(f.group(1)) if f else None)


class Monitor:
    def __init__(self, label="", iface="auto"):
        self.label = label
        self.iface = detect_wifi_iface() if iface == "auto" else iface
        self.prev_t = time.monotonic()
        self.prev_cpu = cpu_times()
        self.prev_proc = {}
        self.prev_tx = tx_counters(self.iface) if self.iface else (None, None)

    def _proc_pct(self, pids, dt):
        total = 0.0
        seen = False
        for pid in pids:
            t = proc_ticks(pid)
            if t is None:
                continue
            seen = True
            prev = self.prev_proc.get(pid)
            self.prev_proc[pid] = t
            if prev is not None:
                total += 100.0 * (t - prev) / CLK_TCK / dt
        return round(total, 1) if seen else None

    def sample(self):
        now = time.monotonic()
        dt = max(1e-3, now - self.prev_t)
        self.prev_t = now

        busy, total = cpu_times()
        pb, pt = self.prev_cpu
        self.prev_cpu = (busy, total)
        cpu_total = round(100.0 * (busy - pb) / max(1, total - pt), 1)

        pids = find_pids()
        wifi = read_wifi(self.iface or "auto")
        retries, failed = tx_counters(self.iface) if self.iface else (None, None)
        d_retries = d_failed = None
        if retries is not None and self.prev_tx[0] is not None:
            d_retries = retries - self.prev_tx[0]
        if failed is not None and self.prev_tx[1] is not None:
            d_failed = failed - self.prev_tx[1]
        self.prev_tx = (retries, failed)

        return {
            "label": self.label,
            "t_unix": round(time.time(), 3),
            "temp_c": temp_c(),
            "cpu_total_pct": cpu_total,
            "cpu_rpicam_pct": self._proc_pct(pids["rpicam"], dt),
            "cpu_camera_node_pct": self._proc_pct(pids["camera_node"], dt),
            "freq_mhz": freq_mhz(),
            "undervoltage": undervoltage(),
            "mem_used_mb": mem_used_mb(),
            "iface": wifi.get("iface") or "",
            "signal_dbm": wifi.get("signal_dbm"),
            "tx_mbps": wifi.get("tx_mbps"),
            "rx_mbps": wifi.get("rx_mbps"),
            "tx_retries": d_retries,
            "tx_failed": d_failed,
        }


def _fmt(v, unit="", nd=0):
    if v is None or v == "":
        return "?"
    return f"{v:.{nd}f}{unit}" if isinstance(v, (int, float)) else f"{v}{unit}"


def main():
    ap = argparse.ArgumentParser(description="Registra temperatura, CPU y Wi-Fi de la Pi.")
    ap.add_argument("--csv", default="", help="archivo de salida (vacío = solo pantalla)")
    ap.add_argument("--label", default="")
    ap.add_argument("--period", type=float, default=2.0, help="segundos entre filas")
    ap.add_argument("--print-every", type=float, default=10.0, help="segundos entre líneas en pantalla")
    ap.add_argument("--iface", default="auto")
    args, _ = ap.parse_known_args()   # ignora --ros-args si lo lanza ROS

    mon = Monitor(args.label, args.iface)
    fh = writer = None
    if args.csv:
        path = os.path.expanduser(args.csv)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        fh = open(path, "w", newline="")
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        print(f"[pi_monitor] registrando en {path} cada {args.period:g} s", flush=True)

    last_print = 0.0
    time.sleep(args.period)          # la primera muestra necesita un intervalo previo
    try:
        while True:
            row = mon.sample()
            if writer:
                writer.writerow(row)
                fh.flush()
            if time.monotonic() - last_print >= args.print_every:
                last_print = time.monotonic()
                uv = row["undervoltage"]
                print(
                    f"[pi_monitor] {_fmt(row['temp_c'], ' C', 1)}  "
                    f"CPU {_fmt(row['cpu_total_pct'], '%')} "
                    f"(rpicam {_fmt(row['cpu_rpicam_pct'], '%')}, "
                    f"nodo {_fmt(row['cpu_camera_node_pct'], '%')})  "
                    f"{_fmt(row['freq_mhz'], ' MHz')}  "
                    f"señal {_fmt(row['signal_dbm'], ' dBm')}  "
                    f"subida {_fmt(row['tx_mbps'], ' Mbps', 1)}"
                    + ("  ¡BAJO VOLTAJE!" if uv == 1 else ""),
                    flush=True,
                )
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()


if __name__ == "__main__":
    main()
