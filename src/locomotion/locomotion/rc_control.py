#!/usr/bin/env python3
"""
Control remoto con mando DualSense — publica en /motor_cmd.

Requiere el nodo `joy_node` del paquete `joy` corriendo en paralelo,
publicando el tópico /joy (ros2 run joy joy_node).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from rclpy.qos import QoSProfile, DurabilityPolicy

# PS5 DualSense — ejes (verificar con `ros2 topic echo /joy`, puede variar
# según el driver/udev rules del sistema)
AX_STEER = 0   # joystick izquierdo horizontal
AX_RT = 5      # gatillo derecho (avanzar)
AX_LT = 2      # gatillo izquierdo (retroceder)

# Botones (mismo layout del driver que los ejes de arriba; verificar igual)
BTN_CROSS = 0  # X: alterna el modo de la herramienta
BTN_L1 = 4     # negativo
BTN_R1 = 5     # positivo

# Modos de la herramienta (R1/L1 actúan sobre el que esté activo)
MODE_STEPPER = "stepper"
MODE_SERVO = "servo"

STEPPER_CHUNK = 50          # pasos por comando mientras se mantiene R1/L1
STEPPER_REPEAT_S = 0.15     # ~ lo que tarda el stepper en dar el chunk a 400 pasos/s
SERVO_STEP_DEG = 1          # grados por comando mientras se mantiene R1/L1
SERVO_REPEAT_S = 0.03       # ~33 °/s (la prueba del Arduino iba a ~66 °/s)


class RCControlNode(Node):
    def __init__(self):
        super().__init__("rc_control_node")
        self.create_subscription(Joy, "/joy", self.gamepad_callback, 10)
        self.motor_pub_ = self.create_publisher(Twist, "/motor_cmd", 10)
        self.stepper_pub_ = self.create_publisher(Int32, "/stepper_steps", 10)
        self.servo_pub_ = self.create_publisher(Int32, "/servo_delta", 10)

        # Latched: el panel recibe el modo actual aunque se conecte después
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub_ = self.create_publisher(String, "/tool_mode", latched)

        self.latest_joy_ = None
        self.tool_mode_ = MODE_STEPPER
        self.prev_cross_ = False
        self.last_tool_cmd_ = 0.0
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz
        self.publish_mode()

        self.get_logger().info(
            "rc_control_node listo\n"
            "  Motores : LT=retroceder  RT=avanzar  JoyIzq=girar\n"
            "  Herramienta : X=alternar stepper/servo  R1=positivo  L1=negativo\n"
            f"  Modo inicial: {self.tool_mode_}"
        )

    def publish_mode(self):
        self.mode_pub_.publish(String(data=self.tool_mode_))

    def gamepad_callback(self, msg: Joy):
        self.latest_joy_ = msg

    def linear_remap(self, value, a=-1.0, b=1.0, c=0.0, d=1.0):
        return (d - c) * (value - a) / (b - a) + c

    def main_loop(self):
        if self.latest_joy_ is None or len(self.latest_joy_.axes) < 6:
            return

        joy = self.latest_joy_
        speed = self.linear_remap(joy.axes[AX_RT], a=1, b=-1, c=0, d=5)
        speed -= self.linear_remap(joy.axes[AX_LT], a=1, b=-1, c=0, d=5)
        angle = self.linear_remap(joy.axes[AX_STEER], a=-1, b=1, c=-3, d=3)

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angle
        self.motor_pub_.publish(msg)

        self.tool_loop(joy)

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
            out.data = direction * STEPPER_CHUNK
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
