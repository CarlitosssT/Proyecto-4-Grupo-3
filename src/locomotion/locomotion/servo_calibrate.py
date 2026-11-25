#!/usr/bin/env python3
"""
Calibra el punto muerto ("quieto") de los servos de rueda con el teclado.

Necesita motor_command corriendo. Con el rover desarmado (o con los gatillos
sueltos), motor_command manda a cada servo su pulso de centro. Si una rueda
se mueve así, su centro está corrido: ajusta hasta que quede quieta.
Pon las ruedas en el aire.

Cambia los parámetros de motor_command en caliente (servo_center_us para los 4
a la vez, servo_trim_us para uno solo). motor_command los guarda en su archivo
de calibración, así que siguen ahí la próxima vez que arranque.

Orden recomendado:
  1. Con "todos" seleccionado, mueve el centro hasta que la mayoría quede quieta.
  2. Selecciona cada servo que todavía se mueva y ajusta su trim.

El PCA9685 a 50 Hz tiene una resolución de ~5 us por paso: cambios de menos
de 5 us pueden no notarse.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

# Mismo orden que SERVO_MOTORS en motor_command.py (no se importa de ahí para
# no depender de smbus2: esta herramienta también puede correr en el PC).
SERVO_MOTORS = ("front_right", "mid_right", "front_left", "mid_left")

TARGET = "/motor_command"
FINE_US = 5
COARSE_US = 25

HELP = f"""
─────────────────────────────────────────────────────
  Calibración del centro de los servos de rueda
─────────────────────────────────────────────────────
  0       : todos (servo_center_us)
  {'  '.join(f'{i + 1}' for i in range(len(SERVO_MOTORS)))} : {', '.join(SERVO_MOTORS)} (servo_trim_us)
  a / d   : -{FINE_US} / +{FINE_US} us
  A / D   : -{COARSE_US} / +{COARSE_US} us
  r       : volver a 0 el trim del servo elegido (o el centro a 1500)
  q       : salir (lo guardado se queda)
─────────────────────────────────────────────────────
  Si una rueda izquierda va hacia atrás: subir (d).
  Si una rueda izquierda va hacia adelante: bajar (a).
  En las derechas es al revés (van invertidas).
─────────────────────────────────────────────────────
"""


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ServoCalibrate(Node):
    def __init__(self):
        super().__init__("servo_calibrate")
        self.get_cli_ = self.create_client(GetParameters, f"{TARGET}/get_parameters")
        self.set_cli_ = self.create_client(SetParameters, f"{TARGET}/set_parameters")
        self.center_ = None
        self.trim_ = None
        self.selected_ = None   # None = todos

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{TARGET} no responde: ¿está corriendo motor_command?")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            raise RuntimeError(f"{TARGET} no contestó a tiempo")
        return future.result()

    def load(self):
        res = self.call(self.get_cli_, GetParameters.Request(
            names=["servo_center_us", "servo_trim_us"]))
        if len(res.values) != 2 or res.values[0].type != ParameterType.PARAMETER_INTEGER:
            raise RuntimeError(f"{TARGET} no tiene servo_center_us/servo_trim_us: ¿versión vieja?")
        self.center_ = res.values[0].integer_value
        self.trim_ = list(res.values[1].integer_array_value)

    def push(self, center, trim):
        params = [
            Parameter(name="servo_center_us", value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=center)),
            Parameter(name="servo_trim_us", value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY, integer_array_value=trim)),
        ]
        res = self.call(self.set_cli_, SetParameters.Request(parameters=params))
        bad = [r.reason for r in res.results if not r.successful]
        if bad:
            print(f"Rechazado: {bad[0]}")
            return
        self.center_, self.trim_ = center, trim

    def adjust(self, delta):
        center, trim = self.center_, list(self.trim_)
        if self.selected_ is None:
            center += delta
        else:
            trim[self.selected_] += delta
        self.push(center, trim)

    def reset(self):
        center, trim = self.center_, list(self.trim_)
        if self.selected_ is None:
            center = 1500
        else:
            trim[self.selected_] = 0
        self.push(center, trim)

    def status(self):
        sel = "todos" if self.selected_ is None else SERVO_MOTORS[self.selected_]
        pulses = ", ".join(f"{n} {self.center_ + t} us"
                           for n, t in zip(SERVO_MOTORS, self.trim_))
        return f"[{sel}] centro={self.center_} trim={self.trim_} -> {pulses}"


def main(args=None):
    rclpy.init(args=args)
    node = ServoCalibrate()
    settings = termios.tcgetattr(sys.stdin)
    try:
        node.load()
        print(HELP)
        print(node.status())
        while True:
            key = get_key(settings)
            if not key:
                continue
            if key == 'q' or key == '\x03':
                break
            if key == '0':
                node.selected_ = None
            elif key.isdigit() and 1 <= int(key) <= len(SERVO_MOTORS):
                node.selected_ = int(key) - 1
            elif key in "aAdD":
                step = COARSE_US if key.isupper() else FINE_US
                node.adjust(step if key.lower() == 'd' else -step)
            elif key == 'r':
                node.reset()
            else:
                continue
            print(node.status())
    except RuntimeError as e:
        print(e)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():   # Ctrl+C ya cierra el contexto en Jazzy
            rclpy.shutdown()


if __name__ == "__main__":
    main()
