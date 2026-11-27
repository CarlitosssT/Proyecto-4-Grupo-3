#!/usr/bin/env python3
"""
Control remoto con mando DualSense — publica en /motor_cmd.

Requiere el nodo `joy_node` del paquete `joy` corriendo en paralelo,
publicando el tópico /joy (ros2 run joy joy_node).

Arming switch: arranca DESARMADO. Options arma (solo con gatillos y joystick
izquierdo en reposo) y Create desarma. Desarmado publica /motor_cmd en cero y
no manda nada a la herramienta ni a la cámara. Si /joy deja de llegar más de
JOY_TIMEOUT_S, se desarma solo. El estado va en /armed (Bool, latched), que
motor_command usa para frenar los motores.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Int32, String
from rclpy.qos import QoSProfile, DurabilityPolicy

# PS5 DualSense — ejes (verificar con `ros2 topic echo /joy`, puede variar
# según el driver/udev rules del sistema)
AX_STEER = 0   # joystick izquierdo horizontal
AX_RT = 5      # gatillo derecho (avanzar)
AX_LT = 2      # gatillo izquierdo (retroceder)
AX_CAM_TILT = 4  # joystick derecho vertical (arriba = +1): inclina la cámara

# Botones (mismo layout del driver que los ejes de arriba; verificar igual)
BTN_CROSS = 0  # X: alterna el modo de la herramienta
BTN_L1 = 4     # negativo
BTN_R1 = 5     # positivo
BTN_CREATE = 8   # desarmar
BTN_OPTIONS = 9  # armar
BTN_R3 = 12    # clic del joystick derecho: centra la cámara

# Escala de /motor_cmd. motor_command satura cada lado a ±1, así que con
# SPEED_MAX = 1 la velocidad es proporcional a todo el recorrido del gatillo
# (antes era 5: con ~20 % de gatillo las ruedas ya iban a tope).
SPEED_MAX = 1.0             # linear.x con el gatillo a fondo
STEER_MAX = 1.0             # angular.z con el joystick a fondo (motor_command lo multiplica por 0.7)

# Arming switch
JOY_TIMEOUT_S = 0.5         # sin /joy por más de esto -> se desarma solo
                            # (joy_node repite el último estado a 20 Hz: autorepeat_rate)
ARM_TRIGGER_REST = 0.9      # gatillo en reposo = eje cerca de +1
ARM_STEER_REST = 0.15       # joystick izquierdo en reposo

# Modos de la herramienta (R1/L1 actúan sobre el que esté activo)
MODE_STEPPER = "stepper"
MODE_SERVO = "servo"

STEPPER_CHUNK = 100          # pasos por comando mientras se mantiene R1/L1
STEPPER_REPEAT_S = 0.15     # ~ lo que tarda el stepper en dar el chunk a 400 pasos/s
SERVO_STEP_DEG = 1          # grados por comando mientras se mantiene R1/L1
SERVO_REPEAT_S = 0.03       # ~33 °/s (la prueba del Arduino iba a ~66 °/s)

# Servo de la cámara (inclinación). Control por velocidad: cuanto más se
# empuja el stick, más rápido se mueve; al soltarlo la cámara se queda donde está.
CAM_DEADZONE = 0.15         # ignora el ruido del stick en reposo
CAM_MAX_DEG_S = 15.0        # velocidad con el stick a fondo
CAM_INVERT = True           # True si "stick arriba" baja la cámara (según montaje)
CAM_CENTER_DEG = 90         # ángulo al que va con R3 (motor_command lo limita a cam_min/cam_max)


class RCControlNode(Node):
    def __init__(self):
        super().__init__("rc_control_node")
        self.create_subscription(Joy, "/joy", self.gamepad_callback, 10)
        self.motor_pub_ = self.create_publisher(Twist, "/motor_cmd", 10)
        self.stepper_pub_ = self.create_publisher(Int32, "/stepper_steps", 10)
        self.servo_pub_ = self.create_publisher(Int32, "/servo_delta", 10)
        self.cam_delta_pub_ = self.create_publisher(Int32, "/camera_tilt_delta", 10)
        self.cam_angle_pub_ = self.create_publisher(Int32, "/camera_tilt_angle", 10)

        # Latched: el panel recibe el modo actual aunque se conecte después
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub_ = self.create_publisher(String, "/tool_mode", latched)
        self.armed_pub_ = self.create_publisher(Bool, "/armed", latched)
        self.stepper_stop_pub_ = self.create_publisher(Empty, "/stepper_stop", 10)

        self.latest_joy_ = None
        self.last_joy_time_ = 0.0
        self.armed_ = False
        self.prev_options_ = False
        self.prev_create_ = False
        self.tool_mode_ = MODE_STEPPER
        self.prev_cross_ = False
        self.last_tool_cmd_ = 0.0
        self.cam_accum_ = 0.0       # fracción de grado aún no enviada
        self.prev_r3_ = False
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz
        self.publish_mode()
        self.armed_pub_.publish(Bool(data=False))

        self.get_logger().info(
            "rc_control_node listo — DESARMADO (Options=armar, Create=desarmar)\n"
            "  Motores : LT=retroceder  RT=avanzar  JoyIzq=girar\n"
            "  Herramienta : X=alternar stepper/servo  R1=positivo  L1=negativo\n"
            "  Cámara : JoyDer arriba/abajo=inclinar  R3=centrar\n"
            f"  Modo inicial: {self.tool_mode_}"
        )

    def publish_mode(self):
        self.mode_pub_.publish(String(data=self.tool_mode_))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def gamepad_callback(self, msg: Joy):
        self.latest_joy_ = msg
        self.last_joy_time_ = self.now_s()

    # ---------------- arming ----------------
    def set_armed(self, armed, reason):
        if armed == self.armed_:
            return
        self.armed_ = armed
        self.armed_pub_.publish(Bool(data=armed))
        if armed:
            self.get_logger().info(f"ARMADO ({reason})")
        else:
            self.motor_pub_.publish(Twist())
            self.stepper_stop_pub_.publish(Empty())
            self.cam_accum_ = 0.0
            self.get_logger().warn(f"DESARMADO ({reason})")

    def sticks_at_rest(self, joy: Joy):
        """None si se puede armar; si no, qué falta soltar."""
        busy = []
        if joy.axes[AX_RT] < ARM_TRIGGER_REST:
            busy.append("RT")
        if joy.axes[AX_LT] < ARM_TRIGGER_REST:
            busy.append("LT")
        if abs(joy.axes[AX_STEER]) > ARM_STEER_REST:
            busy.append("joystick izquierdo")
        return ", ".join(busy) or None

    def arming_loop(self, joy: Joy):
        def pressed(i):
            return len(joy.buttons) > i and bool(joy.buttons[i])

        options, create = pressed(BTN_OPTIONS), pressed(BTN_CREATE)
        if create and not self.prev_create_:
            self.set_armed(False, "Create")
        elif options and not self.prev_options_ and not self.armed_:
            busy = self.sticks_at_rest(joy)
            if busy:
                self.get_logger().warn(f"No se arma: suelta {busy}")
            else:
                self.set_armed(True, "Options")
        self.prev_options_, self.prev_create_ = options, create

    def linear_remap(self, value, a=-1.0, b=1.0, c=0.0, d=0.5):
        return (d - c) * (value - a) / (b - a) + c

    def main_loop(self):
        if self.latest_joy_ is None or len(self.latest_joy_.axes) < 6:
            return

        joy = self.latest_joy_
        if self.now_s() - self.last_joy_time_ > JOY_TIMEOUT_S:
            self.set_armed(False, f"sin /joy por más de {JOY_TIMEOUT_S} s")
            self.motor_pub_.publish(Twist())
            return

        self.arming_loop(joy)
        if not self.armed_:
            self.motor_pub_.publish(Twist())
            # Sigue los flancos para que un botón ya apretado no dispare al armar
            self.prev_cross_ = len(joy.buttons) > BTN_CROSS and bool(joy.buttons[BTN_CROSS])
            self.prev_r3_ = len(joy.buttons) > BTN_R3 and bool(joy.buttons[BTN_R3])
            self.last_tool_cmd_ = 0.0
            return

        # Gatillo: +1 en reposo, -1 a fondo
        speed = self.linear_remap(joy.axes[AX_RT], a=1, b=-1, c=0, d=SPEED_MAX)
        speed -= self.linear_remap(joy.axes[AX_LT], a=1, b=-1, c=0, d=SPEED_MAX)
        angle = self.linear_remap(joy.axes[AX_STEER], a=-1, b=1, c=-STEER_MAX, d=STEER_MAX)

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angle
        self.motor_pub_.publish(msg)

        self.tool_loop(joy)
        self.camera_loop(joy)

    def camera_loop(self, joy: Joy):
        if len(joy.axes) <= AX_CAM_TILT:
            return

        # R3 centra la cámara (flanco de subida)
        r3 = len(joy.buttons) > BTN_R3 and bool(joy.buttons[BTN_R3])
        if r3 and not self.prev_r3_:
            self.cam_accum_ = 0.0
            self.cam_angle_pub_.publish(Int32(data=CAM_CENTER_DEG))
        self.prev_r3_ = r3

        value = joy.axes[AX_CAM_TILT]
        if abs(value) < CAM_DEADZONE:
            self.cam_accum_ = 0.0
            return
        if CAM_INVERT:
            value = -value

        # Reescala para que la velocidad arranque en 0 justo al salir de la zona muerta
        sign = 1.0 if value > 0 else -1.0
        mag = (abs(value) - CAM_DEADZONE) / (1.0 - CAM_DEADZONE)
        self.cam_accum_ += sign * mag * CAM_MAX_DEG_S * 0.02  # periodo del timer

        # Solo se envían grados enteros; el resto se acumula para el próximo ciclo
        step = int(self.cam_accum_)
        if step != 0:
            self.cam_accum_ -= step
            self.cam_delta_pub_.publish(Int32(data=step))

    def tool_loop(self, joy: Joy):
        if len(joy.buttons) <= max(BTN_CROSS, BTN_L1, BTN_R1):
            return

        # X alterna el modo (solo en el flanco de subida)
        cross = bool(joy.buttons[BTN_CROSS])
        if cross and not self.prev_cross_:
            self.tool_mode_ = (
                MODE_SERVO if self.tool_mode_ == MODE_STEPPER else MODE_STEPPER
            )
            self.get_logger().info(f"Herramienta: modo {self.tool_mode_}")
            self.publish_mode()
        self.prev_cross_ = cross

        direction = int(bool(joy.buttons[BTN_R1])) - int(bool(joy.buttons[BTN_L1]))
        if direction == 0:
            self.last_tool_cmd_ = 0.0   # al soltar, la siguiente pulsación sale al instante
            return

        # Primer comando inmediato; mientras se mantiene, se repite con periodo fijo
        now = self.get_clock().now().nanoseconds * 1e-9
        period = STEPPER_REPEAT_S if self.tool_mode_ == MODE_STEPPER else SERVO_REPEAT_S
        if now - self.last_tool_cmd_ < period:
            return
        self.last_tool_cmd_ = now

        out = Int32()
        if self.tool_mode_ == MODE_STEPPER:
            out.data = direction * -STEPPER_CHUNK
            self.stepper_pub_.publish(out)
        else:
            out.data = direction * SERVO_STEP_DEG
            self.servo_pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_armed(False, "nodo cerrado")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
