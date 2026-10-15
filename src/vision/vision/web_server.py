#!/usr/bin/env python3
"""
Visor web servido desde la Raspberry.

Va en:  src/vision/vision/web_server.py

Cualquier equipo en la misma red abre  http://<ip-de-la-pi>:8080  en el
navegador y ve la cámara. No necesita ROS, Python ni nada instalado: sirve
para el Mac, un celular o el computador de un compañero.

Cómo transmite
--------------
La página PIDE cada cuadro ("dame uno más nuevo que el N") y la Pi contesta
apenas tiene uno. Así nunca se acumulan cuadros viejos: si la red va lenta,
el navegador simplemente recibe menos cuadros por segundo, pero siempre el
más reciente. Un flujo continuo (MJPEG) en cambio llenaría el búfer de TCP y
el video se iría retrasando cada vez más.

Diferencia con la consola de ROS
--------------------------------
Esto va por HTTP (TCP), que reenvía lo que se pierde. Con buena señal es
indistinguible. Con señal al límite, en vez de cortes cortos verás pausas un
poco más largas. Para operar en la prueba real conviene la consola de ROS
(base.launch.py); el navegador es para ver la cámara desde cualquier parte.

Rutas
-----
    /               la página
    /frame.jpg      el cuadro siguiente  (?after=N: más nuevo que N)
    /status.json    estado del rover (Wi-Fi, fps, calidad)
    /stream.mjpg    flujo continuo, para VLC u otros programas
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


class FrameHub:
    """Último cuadro publicado, con espera eficiente por uno nuevo."""

    def __init__(self):
        self._cond = threading.Condition()
        self._seq = -1
        self._jpeg = None

    def publish(self, jpeg: bytes, seq: int):
        with self._cond:
            self._jpeg, self._seq = jpeg, seq
            self._cond.notify_all()

    def wait_newer(self, after: int, timeout: float):
        with self._cond:
            self._cond.wait_for(lambda: self._seq > after, timeout=timeout)
            if self._seq > after and self._jpeg is not None:
                return self._seq, self._jpeg
            return None


def make_handler(hub: FrameHub, status_fn):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"     # conexión persistente: menos latencia

        def log_message(self, *args):     # sin una línea por petición
            pass

        def setup(self):
            super().setup()
            # Sin Nagle. La cabecera y la imagen salen en dos envíos; con
            # Nagle activo, el segundo se retiene hasta que el otro lado
            # confirma el primero, y esa confirmación se demora ~40 ms a
            # propósito. Medido: 23 fps de 30 con Nagle.
            import socket
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def _send(self, code, body: bytes, ctype, extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            try:
                if url.path in ("/", "/index.html"):
                    self._send(200, PAGE.encode(), "text/html; charset=utf-8")
                elif url.path == "/frame.jpg":
                    after = int(parse_qs(url.query).get("after", ["-1"])[0])
                    got = hub.wait_newer(after, timeout=1.0)
                    if got is None:
                        self._send(204, b"", "image/jpeg")
                    else:
                        seq, jpeg = got
                        self._send(200, jpeg, "image/jpeg", {"X-Seq": str(seq)})
                elif url.path == "/status.json":
                    try:
                        st = status_fn()
                    except Exception:
                        st = {}
                    self._send(200, json.dumps(st).encode(), "application/json")
                elif url.path == "/stream.mjpg":
                    self._stream()
                else:
                    self._send(404, b"no existe", "text/plain")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            seq = -1
            while True:
                got = hub.wait_newer(seq, timeout=2.0)
                if got is None:
                    continue
                seq, jpeg = got
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                 + jpeg + b"\r\n")

    return Handler


class WebServer:
    def __init__(self, hub: FrameHub, status_fn, port=8080):
        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), make_handler(hub, status_fn))
        self.httpd.daemon_threads = True
        self.port = port

    def start(self):
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()


# ===========================================================================
# La página. Todo en un archivo, sin dependencias externas: funciona sin
# internet. Mismos colores y umbrales que la consola de ROS.
# ===========================================================================
PAGE = r"""<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Rover - camara</title>
<style>
  :root { --ok:#5ac85a; --warn:#ffaf00; --bad:#eb4646; --mute:#9a9a9a; }
  html, body { margin:0; height:100%; background:#000; color:#eee;
    font: 15px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; overflow:hidden; }
  canvas { position:fixed; inset:0; width:100vw; height:100vh; display:block; }
  .bar { position:fixed; left:0; right:0; display:flex; align-items:center; gap:22px;
    padding:10px 16px; background:rgba(0,0,0,.55); z-index:2; }
  .top { top:0; }  .bottom { bottom:0; gap:8px; flex-wrap:wrap; }
  .pill { padding:4px 10px; border-radius:4px; color:#111; font-weight:700; letter-spacing:.5px; }
  .f b { font-size:17px; } .f span { color:var(--mute); font-size:12px; margin-right:6px; }
  .grow { flex:1; }
  .sig { display:flex; align-items:flex-end; gap:3px; height:18px; }
  .sig i { width:5px; background:#555; display:block; }
  button { background:#2a2a2a; color:#ddd; border:1px solid #444; border-radius:4px;
    padding:6px 10px; font:inherit; font-size:13px; cursor:pointer; }
  button:hover { background:#3a3a3a; }
  #alert { position:fixed; inset:0; display:none; flex-direction:column; align-items:center;
    justify-content:center; z-index:1; pointer-events:none; text-align:center; }
  #alert h1 { margin:0; font-size:56px; color:var(--bad); letter-spacing:2px; }
  #alert p { margin:8px 0 0; font-size:20px; }
  #alert .act { color:var(--warn); font-weight:700; }
  body.stale #alert { display:flex; }
  body.stale::after { content:""; position:fixed; inset:0; border:8px solid var(--bad);
    pointer-events:none; z-index:3; }
  body.nohud .bar { display:none; }
  @media (max-width: 700px) { .bar { gap:12px; font-size:13px; } .opt { display:none; } }
</style>
</head>
<body>
<canvas id="c"></canvas>
<div class="bar top">
  <span id="state" class="pill" style="background:var(--mute)">ESPERANDO VIDEO</span>
  <span class="f"><span>FPS</span><b id="fps">--</b></span>
  <span class="f opt"><span>VIDEO</span><b id="mbps">--</b></span>
  <span class="grow"></span>
  <span class="sig" id="bars"><i style="height:5px"></i><i style="height:10px"></i><i style="height:15px"></i><i style="height:20px"></i></span>
  <span id="wifi" style="color:var(--mute); white-space:nowrap">sin datos del Wi-Fi</span>
</div>
<div id="alert"><h1>SIN VIDEO</h1><p id="age"></p><p class="act">detener el rover</p></div>
<div class="bar bottom">
  <button onclick="fs()">F pantalla completa</button>
  <button onclick="toggleHud()">H datos</button>
  <button onclick="guides=!guides;draw()">G guias</button>
  <button onclick="snap()">S foto</button>
  <span class="grow"></span>
  <span id="msg" style="color:var(--mute)"></span>
</div>
<script>
const c = document.getElementById('c'), ctx = c.getContext('2d');
let bmp = null, blob = null, seq = -1, lastRx = 0, guides = false;
let arrivals = [], bytes = [], expected = 30, link = null, linkAt = 0;
const SIG_WARN = -72, STALE_MS = 500;

function resize() {
  const r = window.devicePixelRatio || 1;
  c.width = innerWidth * r; c.height = innerHeight * r; draw();
}
addEventListener('resize', resize);

function draw() {
  ctx.fillStyle = '#000'; ctx.fillRect(0, 0, c.width, c.height);
  if (!bmp) return;
  const s = Math.min(c.width / bmp.width, c.height / bmp.height);
  const w = bmp.width * s, h = bmp.height * s, x = (c.width - w) / 2, y = (c.height - h) / 2;
  ctx.drawImage(bmp, x, y, w, h);
  if (document.body.classList.contains('stale')) {
    ctx.fillStyle = 'rgba(0,0,0,.65)'; ctx.fillRect(x, y, w, h);
  }
  if (guides) {
    ctx.strokeStyle = 'rgba(220,220,220,.8)'; ctx.lineWidth = 1;
    for (const k of [1, 2]) {
      ctx.beginPath(); ctx.moveTo(x + w*k/3, y); ctx.lineTo(x + w*k/3, y + h); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(x, y + h*k/3); ctx.lineTo(x + w, y + h*k/3); ctx.stroke();
    }
    const cx = x + w/2, cy = y + h/2, r = 18 * (devicePixelRatio || 1);
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx - r, cy); ctx.lineTo(cx + r, cy);
    ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r); ctx.stroke();
  }
}

async function getFrame(after) {
  const r = await fetch('/frame.jpg?after=' + after, {cache: 'no-store'});
  if (r.status !== 200) return null;            // 204: no hubo cuadro nuevo en 1 s
  return {seq: parseInt(r.headers.get('X-Seq') || '0'), blob: await r.blob()};
}

async function loop() {
  // Se pide el siguiente cuadro ANTES de decodificar y dibujar el actual,
  // para que el tiempo de dibujo no se reste de la transmisión.
  let pending = getFrame(seq);
  while (true) {
    let res = null;
    try { res = await pending; }
    catch (e) {
      await new Promise(ok => setTimeout(ok, 500));   // sin conexión: reintentar
      pending = getFrame(seq); continue;
    }
    if (!res) { pending = getFrame(seq); continue; }
    seq = res.seq;
    pending = getFrame(seq);
    const now = performance.now();
    lastRx = now; arrivals.push(now); bytes.push([now, res.blob.size]);
    try {
      const next = await createImageBitmap(res.blob);
      if (bmp && bmp.close) bmp.close();
      bmp = next; blob = res.blob; draw();
    } catch (e) {}                                     // JPEG dañado: se ignora
  }
}

async function poll() {
  try {
    const r = await fetch('/status.json', {cache: 'no-store'});
    link = await r.json(); linkAt = performance.now();
    if (link.fps) expected = link.fps;
  } catch (e) {}
}

function level(dbm) {
  if (dbm == null) return [0, 'var(--mute)'];
  if (dbm > -55) return [4, 'var(--ok)'];
  if (dbm > -65) return [3, 'var(--ok)'];
  if (dbm > SIG_WARN) return [2, 'var(--warn)'];
  if (dbm > -80) return [1, 'var(--bad)'];
  return [0, 'var(--bad)'];
}

function tick() {
  const now = performance.now();
  arrivals = arrivals.filter(t => now - t < 3000);
  bytes = bytes.filter(([t]) => now - t < 3000);
  const fps = arrivals.length > 1 ? (arrivals.length - 1) / ((arrivals.at(-1) - arrivals[0]) / 1000) : null;
  const mbps = bytes.length ? bytes.reduce((a, [, n]) => a + n, 0) * 8 / 3e6 : null;
  const stale = lastRx && now - lastRx > STALE_MS;
  document.body.classList.toggle('stale', !!stale);
  if (stale) document.getElementById('age').textContent =
    'imagen congelada hace ' + ((now - lastRx) / 1000).toFixed(1) + ' s';

  const okLink = link && now - linkAt < 3000 ? link : null;
  const dbm = okLink && okLink.connected ? okLink.signal_dbm : null;
  let txt, col;
  if (!lastRx) { txt = 'ESPERANDO VIDEO'; col = 'var(--mute)'; }
  else if (stale) { txt = 'SIN VIDEO'; col = 'var(--bad)'; }
  else if (fps != null && fps < 0.7 * expected) { txt = 'INESTABLE'; col = 'var(--warn)'; }
  else if (dbm != null && dbm < SIG_WARN) { txt = 'SENAL DEBIL'; col = 'var(--warn)'; }
  else { txt = 'OK'; col = 'var(--ok)'; }
  const st = document.getElementById('state'); st.textContent = txt; st.style.background = col;

  // Con el video congelado no se muestra el promedio de los últimos
  // segundos (que todavía parece bueno): se muestra 0 en rojo.
  const f = document.getElementById('fps');
  const shown = stale ? 0 : fps;
  f.textContent = shown == null ? '--' : shown.toFixed(1);
  f.style.color = shown == null ? '' : shown >= 0.9*expected ? 'var(--ok)' : shown >= 0.7*expected ? 'var(--warn)' : 'var(--bad)';
  document.getElementById('mbps').textContent = mbps == null ? '--' : mbps.toFixed(1) + ' Mbps';

  const [lv, lc] = level(dbm);
  document.querySelectorAll('#bars i').forEach((el, i) => el.style.background = i < lv ? lc : '#555');
  const w = document.getElementById('wifi');
  if (dbm != null) {
    const band = (okLink.freq_mhz || 0) >= 5000 ? '5 GHz' : '2.4 GHz';
    w.innerHTML = `${Math.round(dbm)} dBm ${band}` +
      (okLink.tx_mbps ? `<span class="opt"> &nbsp;subida ${Math.round(okLink.tx_mbps)} Mbps</span>` : '');
    w.style.color = lc;
  } else { w.textContent = okLink ? 'rover sin Wi-Fi' : 'sin datos del Wi-Fi'; w.style.color = 'var(--mute)'; }
  if (stale) draw();
}

function fs() {
  const el = document.documentElement;
  if (document.fullscreenElement || document.webkitFullscreenElement)
    (document.exitFullscreen || document.webkitExitFullscreen).call(document);
  else (el.requestFullscreen || el.webkitRequestFullscreen).call(el);
}
function toggleHud() { document.body.classList.toggle('nohud'); }
function flash(t) {
  const m = document.getElementById('msg'); m.textContent = t;
  clearTimeout(flash.t); flash.t = setTimeout(() => m.textContent = '', 2500);
}
function snap() {
  if (!blob) return;
  const a = document.createElement('a');
  const d = new Date(), p = n => String(n).padStart(2, '0');
  a.download = `foto_${d.getFullYear()}${p(d.getMonth()+1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.jpg`;
  a.href = URL.createObjectURL(blob); a.click(); flash('foto descargada');
}
addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (k === 'f') fs(); else if (k === 'h') toggleHud();
  else if (k === 'g') { guides = !guides; draw(); } else if (k === 's') snap();
});

resize(); loop(); poll();
setInterval(poll, 1000); setInterval(tick, 100);
</script>
</body>
</html>
"""
