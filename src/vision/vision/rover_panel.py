#!/usr/bin/env python3
"""
Panel de control: abrir y cerrar los nodos del rover desde el PC.

Va en:  src/vision/vision/rover_panel.py

Solo se corre en el PC. Los nodos de la Pi se manejan por SSH (el multicast no
pasa en esta red, así que los servicios ROS entre máquinas no son fiables):

    ros2 run vision rover_panel
    python3 rover_panel.py            # también funciona sin ROS, solo Python 3

Requisitos
----------
* PC:  python3-tk  (sudo apt install python3-tk) y el cliente ssh.
* SSH sin contraseña hacia la Pi (se usa BatchMode, nunca pide clave):
      ssh-keygen -t ed25519            # si aún no tienes llave
      ssh-copy-id ubuntu@<ip de la Pi>
* En la Pi: el workspace ya compilado (colcon build).

Cómo funciona
-------------
Cada nodo de la Pi se lanza con setsid dentro de ~/.rover_panel/ (un .pid y un
.log por nodo), así que sigue vivo aunque se cierre la ventana o se caiga el
SSH. Detener = SIGINT al grupo de procesos (cierre limpio de ros2 launch) y,
si no termina en 5 s, SIGKILL. El estado se lee de esos .pid cada 3 s, por lo
que el panel refleja la realidad aunque un nodo haya muerto solo.
"""
import glob
import os
import shlex
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

try:
    from vision.rover_beacon import find_rover
except ImportError:  # corrido como script suelto
    from rover_beacon import find_rover

# Video dentro del panel: necesitan ROS 2 + OpenCV. Sin ellos el panel
# sigue funcionando (solo pierde el video embebido).
try:
    import time
    import json
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                           QoSDurabilityPolicy)
    from sensor_msgs.msg import CompressedImage, Joy
    from std_msgs.msg import Bool, String
    import numpy as np
    try:
        from vision.console_render import ConsoleState, render
        from vision.link_metrics import parse_frame_id, RollingStats
    except ImportError:
        from console_render import ConsoleState, render
        from link_metrics import parse_frame_id, RollingStats
    HAVE_FEED = True
except ImportError:
    HAVE_FEED = False

REMOTE_USER = "ubuntu"
REMOTE_WS = "~/proyecto-4-grupo-3"
ROS_DOMAIN_ID = "42"
POLL_S = 3.0
# Logs de los nodos locales. Si el panel se abre con doble clic, su stdout es
# un pipe sin lector y un nodo que escriba ahí muere por SIGPIPE (código 120).
LOCAL_LOG_DIR = os.path.expanduser("~/.rover_panel")


def _find_ws():
    """Raíz del workspace local (carpeta con src/ y build/ o install/)."""
    d = os.path.dirname(os.path.realpath(__file__))
    while d != os.path.dirname(d):
        if os.path.isdir(os.path.join(d, "src")) and (
                os.path.isdir(os.path.join(d, "install"))
                or os.path.isdir(os.path.join(d, "build"))):
            return d
        d = os.path.dirname(d)
    return os.getcwd()


LOCAL_WS = _find_ws()

# La terminal de VS Code (snap) exporta rutas a librerías de core20 que
# rompen las apps Qt/GTK con "undefined symbol: __libc_pthread_init".
SNAP_ENV_VARS = (
    "GTK_PATH", "GTK_EXE_PREFIX", "GDK_PIXBUF_MODULE_FILE",
    "GDK_PIXBUF_MODULEDIR", "GIO_MODULE_DIR", "GSETTINGS_SCHEMA_DIR",
    "LOCPATH", "GTK_IM_MODULE_FILE",
)

# nombre -> (título, comando en la Pi). {source} se rellena desde el panel.
REMOTE_NODES = {
    "camara": ("Cámara + monitor (rover.launch)",
               "ros2 launch vision rover.launch.py source:={source}"),
    "motores": ("Motores (motor_command)",
                "ros2 run locomotion motor_command"),
    "herramienta": ("Stepper (tool_command)",
                    "ros2 run locomotion tool_command"),
}

# nombre -> (título, comando local). El mando DualSense se lee en el PC y
# rc_control publica /motor_cmd hacia la Pi por la red (ROS_STATIC_PEERS).
LOCAL_NODES = {
    "visor": ("Video (en el panel)", None),   # embebido, sin proceso
    "joy": ("Mando: joy_node", ["ros2", "run", "joy", "joy_node"]),
    "rc": ("Mando: rc_control", ["ros2", "run", "locomotion", "rc_control"]),
}

SSH_OPTS = [
    "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "ControlMaster=auto", "-o", "ControlPersist=60",
    "-o", "ControlPath=~/.ssh/rover-panel-%C",
]

# Prólogo común en la Pi: ROS + workspace + los `export ROS_*` de ~/.bashrc
# (un bash no interactivo no lee ~/.bashrc, ahí vive el DOMAIN_ID/PEERS).
PROLOGUE = f"""
export ROS_DOMAIN_ID={ROS_DOMAIN_ID}
eval "$(grep -E '^export ROS_' ~/.bashrc 2>/dev/null)"
# Se suma la IP de este PC (la que abrió el SSH) a los peers: la del ~/.bashrc
# de la Pi queda vieja cada vez que el PC cambia de red y entonces la Pi no
# encuentra al PC por su cuenta.
PC_IP="${{SSH_CLIENT%% *}}"
[ -n "$PC_IP" ] && export ROS_STATIC_PEERS="${{ROS_STATIC_PEERS:+$ROS_STATIC_PEERS;}}$PC_IP"
for f in /opt/ros/*/setup.bash; do [ -f "$f" ] && . "$f"; done
[ -f {REMOTE_WS}/install/setup.bash ] && . {REMOTE_WS}/install/setup.bash
D=~/.rover_panel; mkdir -p "$D"
"""


def _start_script(name, cmd):
    inner = f"{PROLOGUE}\nexec {cmd}"
    return f"""{PROLOGUE}
if [ -f "$D/{name}.pid" ] && kill -0 "$(cat "$D/{name}.pid")" 2>/dev/null; then
  echo "{name} ya estaba corriendo"; exit 0
fi
setsid bash -c {shlex.quote(inner)} > "$D/{name}.log" 2>&1 < /dev/null &
echo $! > "$D/{name}.pid"
sleep 1
kill -0 "$(cat "$D/{name}.pid")" 2>/dev/null && echo "{name} iniciado" \\
  || {{ echo "{name} murió al arrancar:"; tail -n 15 "$D/{name}.log"; exit 1; }}
"""


def _tail(path, n=15):
    try:
        with open(path, errors="replace") as f:
            return "".join(f.readlines()[-n:])
    except OSError:
        return ""


def _stop_script(name):
    return f"""D=~/.rover_panel; P=$(cat "$D/{name}.pid" 2>/dev/null)
if [ -z "$P" ] || ! kill -0 "$P" 2>/dev/null; then echo "{name} no estaba corriendo"; exit 0; fi
kill -INT -- "-$P" 2>/dev/null
for i in $(seq 50); do kill -0 "$P" 2>/dev/null || break; sleep 0.1; done
kill -0 "$P" 2>/dev/null && kill -KILL -- "-$P" 2>/dev/null
rm -f "$D/{name}.pid"; echo "{name} detenido"
"""


STATUS_SCRIPT = """D=~/.rover_panel
for p in "$D"/*.pid; do [ -f "$p" ] || continue
  kill -0 "$(cat "$p")" 2>/dev/null && basename "$p" .pid
done; true
"""


if HAVE_FEED:
    class RosFeed:
        """Nodo ROS dentro del panel: video, estado del enlace, /joy y /armed.

        Corre en su propio hilo; Tk solo lee los atributos (asignaciones
        simples, seguras con el GIL).
        """

        def __init__(self, topic, host):
            if host:   # sin multicast, ROS necesita saber dónde está la Pi
                os.environ["ROS_STATIC_PEERS"] = host
            os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
            self.topic = topic
            self.link_topic = topic.rsplit("/image", 1)[0] + "/link"
            self.stats = RollingStats(3.0)
            self.state = ConsoleState(topic=topic)
            self.frame = None
            self.last_rx = None
            self.link_rx = None
            self.joy_rx = None
            self.joy_axes = self.joy_nbtn = 0
            self.joy_pressed = []
            self.armed = None            # None = aún no llega /armed
            self.own_init = not rclpy.ok()
            if self.own_init:
                rclpy.init()
            self.node = Node("rover_panel_feed")
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.VOLATILE,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.node.create_subscription(CompressedImage, topic, self._on_image, qos)
            self.node.create_subscription(String, self.link_topic, self._on_link, 1)
            self.node.create_subscription(Joy, "/joy", self._on_joy, 10)
            latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.node.create_subscription(Bool, "/armed", self._on_armed, latched)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()

        def _on_image(self, msg):
            now = time.monotonic_ns()
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            _, seq = parse_frame_id(msg.header.frame_id)
            self.stats.add(now, 0.0, len(msg.data), seq)
            self.frame = img
            self.last_rx = now

        def _on_link(self, msg):
            try:
                self.state.link = json.loads(msg.data)
                self.link_rx = time.monotonic_ns()
                fps = self.state.link.get("fps")
                if fps:
                    self.state.expected_fps = float(fps)
            except Exception:
                pass

        def _on_joy(self, msg):
            self.joy_rx = time.monotonic()
            self.joy_axes, self.joy_nbtn = len(msg.axes), len(msg.buttons)
            self.joy_pressed = [i for i, b in enumerate(msg.buttons) if b]

        def _on_armed(self, msg):
            self.armed = bool(msg.data)

        def joy_age(self):
            return None if self.joy_rx is None else time.monotonic() - self.joy_rx

        def image(self, w, h, hud, flip_h=False, flip_v=False):
            """Cuadro listo para mostrar (BGR, w x h) con el HUD de estado.

            flip_h / flip_v voltean solo lo que se ve en el panel (no la cámara).
            """
            now = time.monotonic_ns()
            st = self.state
            self.stats.prune(now)
            s = self.stats.summary()
            st.show_hud = hud
            st.frame_age_s = None if self.last_rx is None else (now - self.last_rx) / 1e9
            st.link_age_s = None if self.link_rx is None else (now - self.link_rx) / 1e9
            if st.link_age_s is not None and st.link_age_s > 3.0:
                st.link = {}
            if s.get("n", 0) > 1:
                st.fps, st.loss_pct = s["fps_rx"], s["loss_pct"]
                st.mbps, st.kb_per_frame = s["kbps"] / 1000.0, s["avg_kb"]
            else:
                st.fps = st.loss_pct = st.mbps = st.kb_per_frame = None
            frame = self.frame
            if frame is not None and (flip_h or flip_v):
                # cv2.flip: 1 = horizontal, 0 = vertical, -1 = ambos (180°)
                frame = cv2.flip(frame, -1 if flip_h and flip_v else (1 if flip_h else 0))
            return render(frame, st, w, h)

        def close(self):
            try:
                self.executor.shutdown()
                self.node.destroy_node()
                if self.own_init and rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


class Panel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Panel del rover")
        self.minsize(900, 560)
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{int(sw * 0.92)}x{int(sh * 0.88)}+10+10")
        self.host = tk.StringVar()
        self.source = tk.StringVar(value="csi:0")
        self.remote_state = {}      # nombre -> bool
        self.reachable = False
        self.local_procs = {}
        self.mode_proc = None
        self.busy = set()
        self.pkg_vars = {}
        self.building = False
        self.feed = None
        self.photo = None
        self._build()
        self.after(100, self.find)
        self.after(500, self._tick)
        self.after(100, self._draw)
        self.protocol("WM_DELETE_WINDOW", self._close)

    # ---- interfaz -----------------------------------------------------
    def _build(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Pi:").pack(side="left")
        ttk.Entry(top, textvariable=self.host, width=22).pack(side="left", padx=4)
        ttk.Button(top, text="Buscar", command=self.find).pack(side="left")
        self.link = ttk.Label(top, text="● sin conexión", foreground="#b00020")
        self.link.pack(side="left", padx=10)

        main = ttk.Frame(self)
        main.pack(fill="both", expand=True)
        left = ttk.Frame(main)
        left.pack(side="left", fill="y")
        right = ttk.Frame(main)
        right.pack(side="left", fill="both", expand=True, padx=(0, 8))
        self._build_view(right)

        box = ttk.LabelFrame(left, text="En la Raspberry", padding=8)
        box.pack(fill="x", padx=8, pady=4)
        self.rows = {}
        for name, (title, _cmd) in REMOTE_NODES.items():
            r = ttk.Frame(box)
            r.pack(fill="x", pady=2)
            dot = ttk.Label(r, text="●", foreground="#888", width=2)
            dot.pack(side="left")
            ttk.Label(r, text=title, width=30).pack(side="left")
            b = ttk.Button(r, text="Iniciar", width=9,
                           command=lambda n=name: self.toggle(n))
            b.pack(side="left", padx=2)
            ttk.Button(r, text="Log", width=5,
                       command=lambda n=name: self.show_log(n)).pack(side="left")
            self.rows[name] = (dot, b)
        s = ttk.Frame(box)
        s.pack(fill="x", pady=(6, 0))
        ttk.Label(s, text="Fuente de cámara:").pack(side="left")
        ttk.Combobox(s, textvariable=self.source, width=8,
                     values=["csi:0", "usb:0"]).pack(side="left", padx=4)
        # El tópico del video depende de la fuente (/csi0 o /usb0): si se
        # cambia con el video abierto, el panel se vuelve a suscribir.
        self.source.trace_add("write", lambda *_: self.after(300, self._follow_source))

        loc = ttk.LabelFrame(left, text="En este PC", padding=8)
        loc.pack(fill="x", padx=8, pady=4)
        self.lrows = {}
        for name, (title, _cmd) in LOCAL_NODES.items():
            r = ttk.Frame(loc)
            r.pack(fill="x", pady=2)
            dot = ttk.Label(r, text="●", foreground="#888", width=2)
            dot.pack(side="left")
            ttk.Label(r, text=title, width=30).pack(side="left")
            b = ttk.Button(r, text="Iniciar", width=9,
                           command=lambda n=name: self.toggle_local(n))
            b.pack(side="left", padx=2)
            self.lrows[name] = (dot, b)

        self.mode_lbl = tk.Label(loc, text="Herramienta: —", font=("TkDefaultFont", 13, "bold"),
                                 fg="white", bg="#888", pady=4)
        self.mode_lbl.pack(fill="x", pady=(6, 0))

        ttk.Button(left, text="Detener todos los nodos",
                   command=self.stop_all).pack(fill="x", padx=8, pady=4)

        bld = ttk.LabelFrame(left, text="Compilar (colcon build)", padding=8)
        bld.pack(fill="x", padx=8, pady=4)
        pk = ttk.Frame(bld)
        pk.pack(fill="x")
        ttk.Label(pk, text="Paquetes:").pack(side="left")
        for pkg in self._packages():
            v = tk.BooleanVar(value=True)
            self.pkg_vars[pkg] = v
            ttk.Checkbutton(pk, text=pkg, variable=v).pack(side="left", padx=6)
        bb = ttk.Frame(bld)
        bb.pack(fill="x", pady=(6, 0))
        self.build_pc = ttk.Button(bb, text="Compilar en este PC",
                                   command=lambda: self.build("pc"))
        self.build_pc.pack(side="left", padx=2)
        self.build_pi = ttk.Button(bb, text="Compilar en la Pi",
                                   command=lambda: self.build("pi"))
        self.build_pi.pack(side="left", padx=2)

        self.out = tk.Text(left, height=8, width=46, state="disabled", wrap="word")
        self.out.pack(fill="both", expand=True, padx=8, pady=8)

    # ---- video dentro del panel ----------------------------------
    def _build_view(self, parent):
        self.video = tk.Label(parent, bg="black", fg="#aaa", text=(
            "Sin video. Inicia «Video» en «En este PC»."
            if HAVE_FEED else
            "Video no disponible: faltan ROS 2 / OpenCV.\n"
            "Abre el panel con ./lanzar_panel.sh"),
            width=1, height=1)
        self.video.pack(fill="both", expand=True, pady=(0, 6))
        opts = ttk.Frame(parent)
        opts.pack(fill="x")
        self.hud = tk.BooleanVar(value=True)
        ttk.Checkbutton(opts, text="Datos sobre el video",
                        variable=self.hud).pack(side="left")
        self.flip_h = tk.BooleanVar(value=False)
        self.flip_v = tk.BooleanVar(value=False)
        ttk.Checkbutton(opts, text="Voltear \u2194",
                        variable=self.flip_h).pack(side="left", padx=(16, 0))
        ttk.Checkbutton(opts, text="Voltear \u2195",
                        variable=self.flip_v).pack(side="left", padx=(8, 0))

        self.ctl = tk.Label(parent, text="Mando: —", anchor="w", fg="white",
                            bg="#888", font=("TkDefaultFont", 11, "bold"), padx=8, pady=3)
        self.ctl.pack(fill="x", pady=(6, 0))

    def _feed_topic(self):
        return f"/{self.source.get().replace(':', '')}/image/compressed"

    def _follow_source(self):
        if self.feed and self.feed.topic != self._feed_topic():
            self._stop_feed()
            self._start_feed()

    def _start_feed(self):
        if not HAVE_FEED:
            self.say("Falta ROS 2 / OpenCV: abre el panel con ./lanzar_panel.sh")
            return False
        host = self.host.get().strip()
        if not host:
            self.say("Aviso: sin IP de la Pi, puede que no llegue el video (usa Buscar).")
        try:
            self.feed = RosFeed(self._feed_topic(), host)
        except Exception as e:  # rclpy sin workspace cargado, etc.
            self.feed = None
            self.say(f"No pude iniciar el video: {e}")
            return False
        self.say(f"Video escuchando {self.feed.topic}.")
        return True

    def _stop_feed(self):
        f, self.feed = self.feed, None
        if f:
            f.close()
        self.photo = None
        self.ctl.configure(text="Mando: —", bg="#888")
        self.video.configure(image="", text="Sin video. Inicia «Video» en «En este PC».")

    def _draw(self):
        f = self.feed
        if f is not None:
            w, h = self.video.winfo_width(), self.video.winfo_height()
            if w > 64 and h > 36:
                try:
                    ok, ppm = cv2.imencode(".ppm", f.image(w, h, self.hud.get(),
                                                           self.flip_h.get(), self.flip_v.get()))
                    if ok:
                        self.photo = tk.PhotoImage(data=ppm.tobytes(), format="PPM")
                        self.video.configure(image=self.photo, text="")
                except Exception as e:
                    self.say(f"Error dibujando video: {e}")
                    self._stop_feed()
            self._show_ctl(f)
        self.after(40, self._draw)

    def _show_ctl(self, f):
        age = f.joy_age()
        if age is None or age > 1.0:
            text, color = "SIN /joy: joy_node no ve el mando (o no llega a este panel)", "#b00020"
        else:
            btn = ",".join(map(str, f.joy_pressed)) or "ninguno"
            arm = {True: "ARMADO", False: "DESARMADO", None: "sin /armed (¿rc_control?)"}[f.armed]
            text = (f"/joy ok ({f.joy_axes} ejes, {f.joy_nbtn} botones)  |  "
                    f"botones pulsados: {btn}  (Options=9, Create=8)  |  {arm}")
            color = "#1b7f3b" if f.armed else "#e65100"
        self.ctl.configure(text=text, bg=color)

    def _packages(self):
        src = os.path.join(LOCAL_WS, "src")
        found = sorted(
            os.path.basename(os.path.dirname(f))
            for f in glob.glob(os.path.join(src, "*", "package.xml")))
        return found or ["vision", "locomotion"]

    # ---- detener todo / compilar --------------------------------------
    def stop_all(self):
        if self.feed:
            self._stop_feed()
        for name in list(self.local_procs):
            self.toggle_local(name)
        for name, on in list(self.remote_state.items()):
            if on and name not in self.busy:
                self.toggle(name)

    def build(self, where):
        pkgs = [p for p, v in self.pkg_vars.items() if v.get()]
        if not pkgs:
            self.say("Selecciona al menos un paquete.")
            return
        if self.building:
            self.say("Ya hay una compilación en curso.")
            return
        self.building = True
        for b in (self.build_pc, self.build_pi):
            b.state(["disabled"])
        args = "--symlink-install --packages-select " + " ".join(pkgs)
        self.say(f"Compilando [{' '.join(pkgs)}] en {'la Pi' if where == 'pi' else 'este PC'}...")

        def work():
            if where == "pi":
                script = (f"{PROLOGUE}\ncd {REMOTE_WS} && colcon build {args} 2>&1 | tail -n 25"
                          "; exit ${PIPESTATUS[0]}")
                rc, out = self._ssh(script, timeout=900)
            else:
                cmd = ('for f in /opt/ros/*/setup.bash; do . "$f"; done; '
                       f'cd {shlex.quote(LOCAL_WS)} && colcon build {args} 2>&1 | tail -n 25'
                       '; exit ${PIPESTATUS[0]}')
                try:
                    p = subprocess.run(["bash", "-c", cmd], capture_output=True,
                                       text=True, timeout=900)
                    rc, out = p.returncode, p.stdout.strip()
                except subprocess.TimeoutExpired:
                    rc, out = 1, "colcon build: tiempo agotado."
            self.after(0, lambda: self._build_done(rc, out))
        self._bg(work)

    def _build_done(self, rc, out):
        self.building = False
        for b in (self.build_pc, self.build_pi):
            b.state(["!disabled"])
        self.say(out)
        self.say("Compilación terminada." if rc == 0
                 else f"La compilación falló (código {rc}).")

    def say(self, text):
        self.out.configure(state="normal")
        self.out.insert("end", text.rstrip() + "\n")
        self.out.see("end")
        self.out.configure(state="disabled")

    # ---- SSH ----------------------------------------------------------
    def _ssh(self, script, timeout=30):
        host = self.host.get().strip()
        if not host:
            return 255, "No hay IP de la Pi (usa Buscar)."
        try:
            p = subprocess.run(
                ["ssh", *SSH_OPTS, f"{REMOTE_USER}@{host}", "bash -s"],
                input=script, capture_output=True, text=True, timeout=timeout)
            return p.returncode, (p.stdout + p.stderr).strip()
        except subprocess.TimeoutExpired:
            return 255, "SSH: tiempo agotado."
        except FileNotFoundError:
            return 255, "No se encontró el comando ssh."

    def _bg(self, fn):
        threading.Thread(target=fn, daemon=True).start()

    def find(self):
        self.say("Buscando la Pi...")

        def work():
            ip, how, _ = find_rover()
            self.after(0, lambda: self._found(ip, how))
        self._bg(work)

    def _found(self, ip, how):
        if ip:
            self.host.set(ip)
            self.say(f"Pi en {ip} ({how}).")
        else:
            self.say("No encontré la Pi. Escribe su IP a mano.")

    # ---- nodos remotos ------------------------------------------------
    def toggle(self, name):
        if name in self.busy:
            return
        self.busy.add(name)
        running = self.remote_state.get(name, False)
        cmd = REMOTE_NODES[name][1].format(source=self.source.get())
        script = _stop_script(name) if running else _start_script(name, cmd)

        def work():
            rc, out = self._ssh(script, timeout=40)
            if rc == 255 and not out.startswith(("No hay", "SSH")):
                out += "\n(¿SSH sin contraseña configurado? ssh-copy-id ubuntu@<ip>)"
            self.after(0, lambda: self._done(name, out))
        self._bg(work)

    def _done(self, name, out):
        self.busy.discard(name)
        self.say(out or name)
        self._poll()

    def show_log(self, name):
        def work():
            _, out = self._ssh(f'tail -n 40 ~/.rover_panel/{name}.log 2>&1')
            self.after(0, lambda: self.say(f"--- log {name} ---\n{out}"))
        self._bg(work)

    def _poll(self):
        def work():
            rc, out = self._ssh(STATUS_SCRIPT, timeout=10)
            ok = rc == 0
            up = set(out.split()) if ok else set()
            self.after(0, lambda: self._apply(ok, up))
        self._bg(work)

    def _apply(self, ok, up):
        self.reachable = ok
        self.link.configure(text="● conectado" if ok else "● sin conexión",
                            foreground="#1b7f3b" if ok else "#b00020")
        for name, (dot, btn) in self.rows.items():
            on = name in up
            self.remote_state[name] = on
            dot.configure(foreground="#1b7f3b" if on else "#888")
            btn.configure(text="Detener" if on else "Iniciar")

    def _tick(self):
        if self.host.get().strip() and not self.busy:
            self._poll()
        for name, (dot, btn) in self.lrows.items():
            if name == "visor":
                on = self.feed is not None
                dot.configure(foreground="#1b7f3b" if on else "#888")
                btn.configure(text="Detener" if on else "Iniciar")
                continue
            p = self.local_procs.get(name)
            if p and p.poll() is not None:
                self.say(f"{name} terminó (código {p.returncode}).\n"
                         + _tail(self._local_log(name)))
                p = self.local_procs.pop(name)
                p = None
            on = p is not None
            dot.configure(foreground="#1b7f3b" if on else "#888")
            btn.configure(text="Detener" if on else "Iniciar")
        self.after(int(POLL_S * 1000), self._tick)

    # ---- nodo local ---------------------------------------------------
    def toggle_local(self, name):
        if name == "visor":
            if self.feed:
                self._stop_feed()
                self.say("Video detenido.")
            else:
                self._start_feed()
            return
        if name == "rc" and not self.feed:
            self._start_feed()    # para ver /joy y /armed en el panel
        p = self.local_procs.get(name)
        if p:
            os.killpg(p.pid, signal.SIGINT)
            self.say(f"{name} detenido.")
            self.local_procs.pop(name)
            if name == "rc":
                self._stop_mode_watch()
            return
        host = self.host.get().strip()
        cmd = LOCAL_NODES[name][1]
        env = {k: v for k, v in os.environ.items() if k not in SNAP_ENV_VARS}
        env["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
        if host:   # sin multicast, ROS necesita saber dónde está la Pi
            env["ROS_STATIC_PEERS"] = host
        else:
            self.say("Aviso: sin IP de la Pi, /motor_cmd puede no llegar (usa Buscar).")
        try:
            os.makedirs(LOCAL_LOG_DIR, exist_ok=True)
            with open(self._local_log(name), "w") as log:
                self.local_procs[name] = subprocess.Popen(
                    cmd, env=env, start_new_session=True,
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
            self.say(f"{name} iniciado: " + " ".join(cmd))
            if name == "rc":
                self._start_mode_watch(env)
        except FileNotFoundError:
            self.say("No encuentro `ros2`: abre el panel desde una terminal "
                     "con el workspace cargado (source install/setup.bash).")

    @staticmethod
    def _local_log(name):
        return os.path.join(LOCAL_LOG_DIR, f"local_{name}.log")

    # ---- modo de la herramienta (servo / stepper) ----------------------
    def _start_mode_watch(self, env):
        self._stop_mode_watch()
        cmd = ["ros2", "topic", "echo", "/tool_mode", "std_msgs/msg/String", "--field", "data",
               "--qos-durability", "transient_local", "--qos-reliability", "reliable"]
        try:
            proc = subprocess.Popen(
                cmd, env={**env, "PYTHONUNBUFFERED": "1"}, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                start_new_session=True)
        except FileNotFoundError:
            return
        self.mode_proc = proc
        self._set_mode(None)

        def read():
            for line in proc.stdout:
                mode = line.strip()
                if mode in ("stepper", "servo"):
                    self.after(0, lambda m=mode: self._set_mode(m))
        self._bg(read)

    def _stop_mode_watch(self):
        p, self.mode_proc = self.mode_proc, None
        if p:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        self._set_mode(None)

    def _set_mode(self, mode):
        text, color = {
            "stepper": ("Herramienta: STEPPER", "#1565c0"),
            "servo": ("Herramienta: SERVO", "#e65100"),
        }.get(mode, ("Herramienta: —", "#888"))
        self.mode_lbl.configure(text=text, bg=color)

    def _close(self):
        self._stop_mode_watch()
        self._stop_feed()
        for p in self.local_procs.values():
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        self.destroy()   # los nodos de la Pi siguen corriendo a propósito


def main():
    Panel().mainloop()


if __name__ == "__main__":
    main()
