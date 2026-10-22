#!/usr/bin/env python3
"""
Herramienta de excavación (servo de pala + stepper) vía serial a una ESP32.

La ESP32 (firmware/tool_esp32/tool_esp32.ino) mueve el servo y el stepper; este
nodo solo traduce los tópicos de ROS 2 a comandos de texto por USB-serial.

Tópicos (los mismos que antes):
  /stepper_steps  (std_msgs/Int32) pasos RELATIVOS a dar (+/-)       -> "<pasos>"
  /servo_delta    (std_msgs/Int32) grados a sumar al ángulo (+/-)    -> "d<grados>"
  /servo_angle    (std_msgs/Int32) ir a un ángulo exacto             -> "g<ángulo>"
  /stepper_stop   (std_msgs/Empty) frena el stepper                  -> "x"

Parámetros: port (/dev/ttyACM0), baudrate (115200).
Si la ESP se desconecta, el nodo reintenta abrir el puerto cada segundo.
"""
import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Empty, Int32


class ToolCommand(Node):
    def __init__(self):
        super().__init__("tool_command")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        self.port_name_ = self.get_parameter("port").value
        self.baud_ = int(self.get_parameter("baudrate").value)
        self.ser_ = None
        self.lock_ = threading.Lock()
        self.running_ = True

        self._connect()
        self.reader_ = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_.start()
        self.create_timer(1.0, self._reconnect)

        self.create_subscription(Int32, "/stepper_steps", self.steps_callback, 10)
        self.create_subscription(Empty, "/stepper_stop", self.stop_callback, 10)
        self.create_subscription(Int32, "/servo_delta", self.delta_callback, 10)
        self.create_subscription(Int32, "/servo_angle", self.angle_callback, 10)

    # ---------------- serial ----------------
    def _connect(self):
        try:
            ser = serial.Serial()
            ser.port = self.port_name_
            ser.baudrate = self.baud_
            ser.timeout = 0.2
            # Evita que abrir el puerto resetee la ESP vía DTR/RTS.
            ser.dtr = False
            ser.rts = False
            ser.open()
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(
                f"No se pudo abrir {self.port_name_}: {e}",
                throttle_duration_sec=5.0,
            )
            return
        with self.lock_:
            self.ser_ = ser
        self.get_logger().info(f"ESP32 conectada en {self.port_name_} @ {self.baud_}.")

    def _reconnect(self):
        with self.lock_:
            connected = self.ser_ is not None
        if not connected:
            self._connect()

    def _drop(self, err):
        with self.lock_:
            ser, self.ser_ = self.ser_, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
            self.get_logger().error(f"Serial perdido ({err}); reintentando...")

    def _send(self, text: str):
        with self.lock_:
            ser = self.ser_
        if ser is None:
            return
        try:
            ser.write((text + "\n").encode())
        except (serial.SerialException, OSError) as e:
            self._drop(e)

    def _read_loop(self):
        """Muestra en el log lo que imprime la ESP."""
        while self.running_:
            with self.lock_:
                ser = self.ser_
            if ser is None:
                threading.Event().wait(0.2)
                continue
            try:
                line = ser.readline().decode(errors="replace").strip()
            except (serial.SerialException, OSError) as e:
                self._drop(e)
                continue
            if line:
                self.get_logger().info(f"[ESP] {line}")

    # ---------------- callbacks ----------------
    def steps_callback(self, msg: Int32):
        if msg.data != 0:
            self._send(str(msg.data))

    def stop_callback(self, _msg: Empty):
        self._send("x")

    def delta_callback(self, msg: Int32):
        if msg.data != 0:
            self._send(f"d{msg.data}")

    def angle_callback(self, msg: Int32):
        self._send(f"g{msg.data}")

    def destroy_node(self):
        self.running_ = False
        self._send("x")
        with self.lock_:
            ser, self.ser_ = self.ser_, None
        if ser is not None:
            ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ToolCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
