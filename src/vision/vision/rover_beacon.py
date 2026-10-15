#!/usr/bin/env python3
"""
Encontrar la Raspberry del rover en la red, sin saber su IP.

Va en:  src/vision/vision/rover_beacon.py

El problema
-----------
La IP de la Pi la asigna el router y cambia (terminó en .9, .10, .13...).
Todo lo que tenía la IP escrita a mano se rompía. Y el descubrimiento
automático de ROS 2 no funciona en esta red porque el router no deja pasar
multicast (se probó con `ros2 multicast send/receive`).

La solución, en tres capas
--------------------------
Se prueban en orden y basta con que una funcione:

1. Baliza. El nodo de la cámara escucha en el puerto UDP 47800. Este módulo
   pregunta "¿quién es el rover?" por broadcast y además, por si el router
   bloquea el broadcast, a cada dirección de la subred (.1 a .254). La Pi
   contesta con su nombre. Es la más confiable, pero requiere que la cámara
   esté corriendo.

2. Nombre. Prueba grupo3p4.local (mDNS) y grupo3p4.lan (el DNS de routers
   tipo GL.iNet/OpenWrt). Funciona si la red lo permite.

3. Dirección física (MAC). Toda Raspberry Pi tiene una MAC que empieza por
   un prefijo asignado a Raspberry Pi Ltd (dc:a6:32 en la Pi 4 de este
   proyecto). Tras el barrido de la capa 1, la tabla ARP del sistema ya
   conoce las MAC de la subred. Funciona aunque la cámara esté apagada,
   así que sirve también para entrar por SSH.

Uso
---
No necesita ROS ni nada instalado: solo Python 3. Funciona en Linux y Mac.

    python3 rover_beacon.py                 # dice dónde está el rover
    python3 rover_beacon.py --ip            # solo la IP (para scripts)
    python3 rover_beacon.py --export        # línea export ROS_STATIC_PEERS=...

    ssh ubuntu@$(python3 rover_beacon.py --ip)

Con el paquete instalado:  ros2 run vision find_rover
"""
import argparse
import json
import os
import platform
import re
import socket
import subprocess
import sys
import threading
import time

PORT = 47800
PROBE = b"ROVER?"
DEFAULT_NAMES = ("grupo3p4.local", "grupo3p4.lan", "grupo3p4")

# Prefijos de MAC (OUI) registrados a Raspberry Pi Foundation / Ltd.
RPI_OUIS = ("b8:27:eb", "dc:a6:32", "e4:5f:01", "d8:3a:dd", "2c:cf:67", "28:cd:c1")


# ===========================================================================
# Lado ROVER: responder a las preguntas
# ===========================================================================
class BeaconResponder:
    """Hilo que contesta a quien pregunte por el rover. Se usa desde camera_node."""

    def __init__(self, info_fn, port=PORT):
        self.info_fn = info_fn
        self.port = port
        self.sock = None
        self.last_client = None
        self._running = False

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data.startswith(PROBE):
                continue
            self.last_client = addr[0]
            try:
                info = dict(self.info_fn())
            except Exception:
                info = {}
            info.setdefault("rover", socket.gethostname())
            try:
                self.sock.sendto(json.dumps(info).encode(), addr)
            except OSError:
                pass

    def stop(self):
        self._running = False
        if self.sock:
            self.sock.close()


# ===========================================================================
# Lado PC: buscar
# ===========================================================================
def primary_ipv4():
    """IP local por la que se sale a la red. No envía ningún paquete."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def _subnet_hosts(ip):
    base = ".".join(ip.split(".")[:3])
    return [f"{base}.{i}" for i in range(1, 255) if f"{base}.{i}" != ip], f"{base}.255"


def resolve_names(names=DEFAULT_NAMES, timeout=1.5):
    """Resuelve nombres en paralelo, sin esperar más de `timeout`."""
    found = {}

    def work(n):
        try:
            found[n] = socket.gethostbyname(n)
        except OSError:
            pass

    threads = [threading.Thread(target=work, args=(n,), daemon=True) for n in names]
    for t in threads:
        t.start()
    deadline = time.time() + timeout
    for t in threads:
        t.join(max(0.0, deadline - time.time()))
    return {n: ip for n, ip in found.items() if not ip.startswith("127.")}


def probe(targets, timeout=1.2, port=PORT):
    """Pregunta a cada destino; devuelve {ip: info} de los que contestan."""
    import select

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # Sin bloqueo: enviar a una IP que no existe no debe hacer esperar. Con
    # un socket bloqueante, cada dirección vacía de la subred costaba un
    # tiempo de espera y el barrido tardaba segundos.
    s.setblocking(False)
    answers = {}

    def send_all():
        for t in targets:
            try:
                s.sendto(PROBE, (t, port))
            except OSError:
                pass  # destino inexistente o cola llena: normal en un barrido

    def collect(until):
        while True:
            left = until - time.time()
            if left <= 0:
                return
            ready, _, _ = select.select([s], [], [], left)
            if not ready:
                return
            try:
                data, addr = s.recvfrom(2048)
            except OSError:
                continue
            try:
                answers[addr[0]] = json.loads(data.decode())
            except ValueError:
                pass
            # Con la primera respuesta basta esperar un instante más por si
            # contesta por otra interfaz, no el tiempo completo.
            until = min(until, time.time() + 0.15)

    # Dos rondas: la primera también sirve para que el sistema resuelva las
    # direcciones físicas (ARP); algunos paquetes de la primera se pierden.
    send_all()
    collect(time.time() + timeout / 2)
    if not answers:
        send_all()
        collect(time.time() + timeout / 2)
    s.close()
    return answers


def _norm_mac(mac):
    return ":".join(p.zfill(2) for p in mac.lower().split(":"))


def arp_raspberries():
    """IPs de la tabla ARP cuya MAC es de Raspberry Pi."""
    try:
        if platform.system() == "Darwin":
            out = subprocess.run(["arp", "-an"], capture_output=True, text=True, timeout=2).stdout
            pairs = re.findall(r"\((\d+\.\d+\.\d+\.\d+)\) at ([0-9a-fA-F:]+)", out)
        else:
            out = subprocess.run(["ip", "neigh"], capture_output=True, text=True, timeout=2).stdout
            pairs = re.findall(r"^(\d+\.\d+\.\d+\.\d+) .*?lladdr ([0-9a-fA-F:]+)", out, re.M)
    except Exception:
        return []
    return [ip for ip, mac in pairs if _norm_mac(mac)[:8] in RPI_OUIS]


def find_rover(names=DEFAULT_NAMES, extra_targets=(), timeout=1.2, verbose=False):
    """
    Devuelve (ip, método, info) o (None, None, None).

    método: "baliza", "nombre" o "mac".
    """
    log = (lambda *a: print(*a, file=sys.stderr)) if verbose else (lambda *a: None)

    by_name = resolve_names(names)
    if by_name:
        log(f"  por nombre: {by_name}")

    local = primary_ipv4()
    targets = list(extra_targets) + list(by_name.values()) + ["255.255.255.255"]
    if local:
        hosts, bcast = _subnet_hosts(local)
        targets += [bcast] + hosts
        log(f"  este equipo: {local}  -> buscando en {bcast[:-4]}.0/24")

    answers = probe(targets, timeout=timeout)
    if answers:
        ip, info = next(iter(answers.items()))
        if len(answers) > 1:
            log(f"  varias respuestas: {list(answers)}")
        return ip, "baliza", info

    if by_name:
        return next(iter(by_name.values())), "nombre", {}

    pis = arp_raspberries()
    if pis:
        if len(pis) > 1:
            log(f"  varias Raspberry Pi en la red: {pis}")
        return pis[0], "mac", {}

    return None, None, None


# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="Encuentra la Raspberry del rover en la red.")
    ap.add_argument("--ip", action="store_true", help="imprimir solo la IP")
    ap.add_argument("--export", action="store_true", help="imprimir 'export ROS_STATIC_PEERS=IP'")
    ap.add_argument("--name", action="append", help="nombre adicional a probar")
    ap.add_argument("--respond", action="store_true", help="(pruebas) actuar como el rover")
    args = ap.parse_args()

    if args.respond:
        r = BeaconResponder(lambda: {"rover": socket.gethostname(), "http_port": 8080})
        r.start()
        print(f"respondiendo en UDP {PORT}... Ctrl+C para salir")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            return

    names = tuple(args.name or ()) + DEFAULT_NAMES
    quiet = args.ip or args.export
    if not quiet:
        print("Buscando el rover...", file=sys.stderr)
    ip, how, info = find_rover(names=names, verbose=not quiet)

    if not ip:
        if not quiet:
            print("\nNo encontré el rover. Revisa que:\n"
                  "  - la Pi esté encendida y en la MISMA red que este equipo\n"
                  "  - si la cámara no está corriendo, solo funciona la búsqueda por MAC\n"
                  "  - la red no aísle a los clientes entre sí (redes de universidad)",
                  file=sys.stderr)
        sys.exit(1)

    if args.ip:
        print(ip)
    elif args.export:
        print(f"export ROS_STATIC_PEERS={ip}")
    else:
        port = info.get("http_port") if info else None
        print(f"\nRover encontrado: {ip}   (método: {how}"
              + (f", nombre: {info.get('rover')}" if info.get("rover") else "") + ")")
        if port:
            print(f"Video en el navegador:  http://{ip}:{port}")
        print(f"SSH:                    ssh ubuntu@{ip}")
        if how == "mac":
            print("(la cámara no parece estar corriendo: no contestó la baliza)")


if __name__ == "__main__":
    main()
