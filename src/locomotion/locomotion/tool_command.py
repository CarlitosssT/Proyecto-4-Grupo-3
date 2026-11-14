#!/usr/bin/env python3
"""
Puente serial con la ESP32 DevKit: stepper de la herramienta + encoders.

La ESP32 (firmware/tool_esp32/tool_esp32.ino) mueve el stepper y cuenta los
pulsos de los 6 encoders. Este nodo es el ÚNICO que abre ese puerto serial:
traduce los tópicos de ROS 2 a comandos de texto y publica los conteos que
manda la ESP. Para convertirlos a ángulo y velocidad está encoder_node.

El servo de la pala ya NO pasa por aquí: va en el PCA9685 y lo mueve
motor_command (/servo_delta y /servo_angle, los mismos tópicos de antes).

Se suscribe a:
  /stepper_steps  (std_msgs/Int32) pasos RELATIVOS a dar (+/-)       -> "<pasos>"
  /stepper_stop   (std_msgs/Empty) frena el stepper                  -> "x"

Publica:
  /encoder_ticks  (std_msgs/Int64MultiArray) una por cada línea
                  "E <micros> <c1> ... <c6>" de la ESP (20 por segundo):
                  data[0] = micros() de la ESP, data[1..6] = conteos crudos.

Parámetros: port (/dev/ttyUSB0), baudrate (115200).
Si la ESP se desconecta, el nodo reintenta abrir el puerto cada segundo.
"""
import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, Int64MultiArray

NUM_ENCODERS = 6


class ToolCommand(Node):
    def __init__(self):
        super().__init__("tool_command")

        # La DevKit (CP2102/CH340) suele aparecer como ttyUSB0; la C3 era ttyACM0
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_name_ = self.get_parameter("port").value
        self.baud_ = int(self.get_parameter("baudrate").value)
        self.ser_ = None
        self.lock_ = threading.Lock()
        self.running_ = True

        # Antes de arrancar el hilo lector, que es quien publica
        self.encoder_pub_ = self.create_publisher(Int64MultiArray, "/encoder_ticks", 10)

        self._connect()
        self.reader_ = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_.start()
        self.create_timer(1.0, self._reconnect)

        self.create_subscription(Int32, "/stepper_steps", self.steps_callback, 10)
        self.create_subscription(Empty, "/stepper_stop", self.stop_callback, 10)

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
        """Publica los reportes de encoders y muestra en el log lo demás."""
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
            if line.startswith("E "):
                self._publish_encoders(line)
            elif line:
                self.get_logger().info(f"[ESP] {line}")

    def _publish_encoders(self, line: str):
        """'E <micros> <c1> ... <c6>' -> /encoder_ticks."""
        try:
            values = [int(p) for p in line.split()[1:]]
        except ValueError:
            values = []
        if len(values) != 1 + NUM_ENCODERS:
            self.get_logger().warn(
                f"Reporte de encoders incompleto: {line!r}", throttle_duration_sec=5.0)
            return
        self.encoder_pub_.publish(Int64MultiArray(data=values))

    # ---------------- callbacks ----------------
    def steps_callback(self, msg: Int32):
        if msg.data != 0:
            self._send(str(msg.data))

    def stop_callback(self, _msg: Empty):
        self._send("x")

    def destroy_node(self):
        self.running_ = False
        # Espera a que el hilo lector termine su lectura (timeout del serial:
        # 0.2 s) antes de cerrar el puerto y destruir el publicador que usa.
        self.reader_.join(timeout=1.0)
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
