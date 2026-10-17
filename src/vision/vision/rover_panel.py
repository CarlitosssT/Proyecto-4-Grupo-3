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

REMOTE_USER = "ubuntu"
REMOTE_WS = "~/proyecto-4-grupo-3"
ROS_DOMAIN_ID = "42"
POLL_S = 3.0

# nombre -> (título, comando en la Pi). {source} se rellena desde el panel.
REMOTE_NODES = {
    "camara": ("Cámara + monitor (rover.launch)",
               "ros2 launch vision rover.launch.py source:={source}"),
    "motores": ("Motores (motor_command)",
                "ros2 run locomotion motor_command"),
    "rc": ("Control RC (rc_control)",
           "ros2 run locomotion rc_control"),
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


class Panel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Panel del rover")
        self.minsize(560, 420)
        self.host = tk.StringVar()
        self.source = tk.StringVar(value="csi:0")
        self.remote_state = {}      # nombre -> bool
        self.reachable = False
        self.local_proc = None
        self.busy = set()
        self._build()
        self.after(100, self.find)
        self.after(500, self._tick)
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

        box = ttk.LabelFrame(self, text="En la Raspberry", padding=8)
        box.pack(fill="x", padx=8, pady=4)
        self.rows = {}
        for name, (title, _cmd) in REMOTE_NODES.items():
            r = ttk.Frame(box)
            r.pack(fill="x", pady=2)
            dot = ttk.Label(r, text="●", foreground="#888", width=2)
            dot.pack(side="left")
            ttk.Label(r, text=title, width=34).pack(side="left")
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

        loc = ttk.LabelFrame(self, text="En este PC", padding=8)
        loc.pack(fill="x", padx=8, pady=4)
        r = ttk.Frame(loc)
        r.pack(fill="x")
        self.ldot = ttk.Label(r, text="●", foreground="#888", width=2)
        self.ldot.pack(side="left")
        ttk.Label(r, text="Visor de video (base.launch)", width=34).pack(side="left")
        self.lbtn = ttk.Button(r, text="Iniciar", width=9, command=self.toggle_local)
        self.lbtn.pack(side="left", padx=2)

        self.out = tk.Text(self, height=10, state="disabled", wrap="word")
        self.out.pack(fill="both", expand=True, padx=8, pady=8)

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
        if self.local_proc and self.local_proc.poll() is not None:
            self.local_proc = None
        on = self.local_proc is not None
        self.ldot.configure(foreground="#1b7f3b" if on else "#888")
        self.lbtn.configure(text="Detener" if on else "Iniciar")
        self.after(int(POLL_S * 1000), self._tick)

    # ---- nodo local ---------------------------------------------------
    def toggle_local(self):
        if self.local_proc:
            os.killpg(self.local_proc.pid, signal.SIGINT)
            self.say("Visor detenido.")
            return
        host = self.host.get().strip() or "auto"
        cmd = ["ros2", "launch", "vision", "base.launch.py",
               f"rover:={host}", f"source:={self.source.get()}"]
        try:
            self.local_proc = subprocess.Popen(cmd, start_new_session=True)
            self.say("Visor iniciado: " + " ".join(cmd))
        except FileNotFoundError:
            self.say("No encuentro `ros2`: abre el panel desde una terminal "
                     "con el workspace cargado (source install/setup.bash).")

    def _close(self):
        if self.local_proc:
            try:
                os.killpg(self.local_proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        self.destroy()   # los nodos de la Pi siguen corriendo a propósito


def main():
    Panel().mainloop()


if __name__ == "__main__":
    main()
