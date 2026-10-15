#!/usr/bin/env python3
"""
Lee el estado del Wi-Fi del rover para mostrárselo al operador.

Va en:  src/vision/vision/wifi_status.py

El operador necesita saber cuándo se está acercando al límite de alcance
ANTES de que se corte el video. En las mediciones de este proyecto el enlace
pasó de funcionar a perder el 60 % de los cuadros con solo 5 dB de diferencia
(−70 a −75 dBm), así que la señal es el aviso temprano.

No necesita permisos de administrador: `iw dev <iface> link` es lectura.
"""
import os
import re
import subprocess


def detect_wifi_iface():
    """
    Devuelve la interfaz inalámbrica por la que sale el tráfico, o None.

    Si se conecta un adaptador USB (wlan1) y la ruta pasa por él, se elige
    ese solo, sin cambiar configuración.
    """
    try:
        out = subprocess.run(["ip", "-o", "route", "get", "1.1.1.1"],
                             capture_output=True, text=True, timeout=1).stdout
        m = re.search(r"\bdev (\S+)", out)
        if m and os.path.isdir(f"/sys/class/net/{m.group(1)}/wireless"):
            return m.group(1)
    except Exception:
        pass
    # Sin ruta por defecto: la primera interfaz inalámbrica que esté arriba.
    try:
        for name in sorted(os.listdir("/sys/class/net")):
            if os.path.isdir(f"/sys/class/net/{name}/wireless"):
                state = open(f"/sys/class/net/{name}/operstate").read().strip()
                if state == "up":
                    return name
    except Exception:
        pass
    return None


def parse_iw_link(text: str) -> dict:
    """Convierte la salida de `iw dev X link` en un diccionario."""
    if not text or "Not connected" in text:
        return {"connected": False}

    def num(pattern):
        m = re.search(pattern, text)
        return float(m.group(1)) if m else None

    ssid = re.search(r"SSID:\s*(.+)", text)
    return {
        "connected": True,
        "ssid": ssid.group(1).strip() if ssid else "",
        "freq_mhz": num(r"freq:\s*([\d.]+)"),
        "signal_dbm": num(r"signal:\s*(-?\d+)"),
        "tx_mbps": num(r"tx bitrate:\s*([\d.]+)"),
        "rx_mbps": num(r"rx bitrate:\s*([\d.]+)"),
    }


def read_wifi(iface: str = "auto") -> dict:
    if not iface or iface == "auto":
        iface = detect_wifi_iface()
    if not iface:
        return {"connected": False, "iface": None}
    try:
        out = subprocess.run(["iw", "dev", iface, "link"],
                             capture_output=True, text=True, timeout=1).stdout
    except Exception:
        out = ""
    info = parse_iw_link(out)
    info["iface"] = iface
    return info
