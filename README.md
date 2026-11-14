# Proyecto 4 — Grupo 3 (ASME IAM3D R.O.V.E.R.)

Workspace de ROS 2 con los paquetes de **locomoción** y **visión** del rover de tracción diferencial (skid-steer) de 6 ruedas / 3 puentes H, construido para la competencia ASME IAM3D R.O.V.E.R.

## Estructura del repositorio

```
src/
├── locomotion/          # Control de los 6 motores DC (teclado o mando DualSense)
│   ├── locomotion/
│   │   ├── motor_command.py     # Nodo del PCA9685 (I2C): 6 motores, servo de cámara y servo de pala
│   │   ├── tool_command.py      # Puente serial a la ESP32 DevKit: stepper (/stepper_steps, /stepper_stop) y conteos de encoders (/encoder_ticks)
│   │   ├── encoder_node.py      # Convierte /encoder_ticks en ángulo y velocidad de cada rueda (/wheel_states)
│   │   ├── keyboard_control.py  # Teleoperación por teclado (w/a/s/d/espacio/q)
│   │   └── rc_control.py        # Teleoperación con mando DualSense (requiere joy_node)
│   ├── package.xml
│   └── setup.py
└── vision/              # Captura y visualización de cámaras
    ├── vision/
    │   ├── camera_node.py        # Captura con OpenCV, publica CompressedImage
    │   └── image_viewer_node.py  # Visor standalone (sin rqt) para una cámara
    ├── launch/
    │   └── cameras.launch.py     # Lanza N instancias de camera_node (una por device)
    ├── package.xml
    └── setup.py
```

## Paquete `locomotion`

Controla 6 motores DC agrupados en 3 puentes H TB6612FNG (2 motores por puente), organizados en tracción tipo skid-steer: todos los motores de un mismo lado (izquierdo/derecho) reciben el mismo comando.

- **`motor_command`**: se suscribe a `/motor_cmd` (`geometry_msgs/Twist`) y traduce `linear.x` (avance) y `angular.z` (giro) a un comando por lado, normalizando ambos lados juntos para preservar la diferencia de velocidad (el giro) cuando algún valor supera el rango. Corre a 50 Hz y genera el PWM de los puentes H con un PCA9685 por I2C (`smbus2`). También mueve el **servo de inclinación de la cámara** en el canal 12 del mismo PCA9685 (tópicos `/camera_tilt_delta` y `/camera_tilt_angle`, `Int32` en grados). Parámetros: `i2c_bus` (1), `i2c_address` (`0x40`), `require_arm` (`true`: frena los 6 motores mientras `/armed` no sea `true`), `cmd_timeout` (0.5 s: frena si deja de llegar `/motor_cmd`; `0` lo desactiva), `pwm_frequency` (50 Hz: el PCA tiene una sola frecuencia para todos los canales y el servo la necesita baja), `cam_channel` (12), `cam_min_deg`/`cam_max_deg` (20/160), `cam_start_deg` (90), `cam_pulse_min_us`/`cam_pulse_max_us` (500/2400). El **servo de la pala** va en el canal 13 (tópicos `/servo_delta` y `/servo_angle`, `Int32` en grados; siempre se mueve con rampa de ~50 °/s): `shovel_channel` (13), `shovel_min_deg`/`shovel_max_deg` (5/130), `shovel_start_deg` (30), `shovel_pulse_min_us`/`shovel_pulse_max_us` (500/2400).
- **`keyboard_control`**: teleoperación por teclado. `w/s` sube/baja velocidad lineal, `a/d` gira, `espacio` detiene, `q` sale. Publica en `/motor_cmd`.
- **`rc_control`**: teleoperación con mando PS5 DualSense a través de `/joy`. Requiere tener corriendo `joy_node` del paquete `joy` en paralelo. Publica en `/motor_cmd`. **Arming switch:** arranca *desarmado*; **Options** arma (solo con gatillos y joystick izquierdo en reposo) y **Create** desarma (además manda `/stepper_stop`). Desarmado publica `/motor_cmd` en cero y no manda nada a la herramienta ni a la cámara. Si `/joy` deja de llegar más de 0.5 s se desarma solo. El estado va en `/armed` (`Bool`, latched). Además controla la herramienta de excavación: **X** alterna entre modo *stepper* (inicial) y modo *servo*; **R1** = positivo y **L1** = negativo sobre el modo activo (se repite mientras se mantiene el botón). Publica en `/stepper_steps` y `/servo_delta`. El **joystick derecho (vertical)** inclina la cámara por velocidad (más empuje = más rápido; al soltar se queda quieta) y **R3** la centra; publica en `/camera_tilt_delta` y `/camera_tilt_angle`. Constantes `AX_CAM_TILT`, `CAM_INVERT`, `CAM_MAX_DEG_S` al inicio del archivo. Los índices de botones (`BTN_*`) están al inicio de `rc_control.py`; verificar con `ros2 topic echo /joy`.
- **`tool_command`**: puente serial hacia la **ESP32 DevKit** (ESP32 clásica), que mueve el stepper NEMA 17 con A4988 y cuenta los pulsos de los 6 encoders (firmware en `firmware/tool_esp32/tool_esp32.ino`; librerías `AccelStepper` y `ESP32Encoder`). Es el único nodo que abre ese puerto. El servo de la pala no pasa por la ESP: va en el PCA9685 (ver `motor_command`). Se suscribe a `/stepper_steps` (`Int32`, pasos relativos → `"<pasos>"`) y `/stepper_stop` (`Empty` → `x`); publica `/encoder_ticks` (`Int64MultiArray`: `data[0]` = `micros()` de la ESP, `data[1..6]` = conteos crudos en el orden front/mid/rear izquierdo y front/mid/rear derecho), 20 veces por segundo. Parámetros: `port` (`/dev/ttyUSB0`), `baudrate` (115200). Reconecta solo si la ESP se desconecta.
- **`encoder_node`**: convierte `/encoder_ticks` en `/wheel_states` (`sensor_msgs/JointState`: ángulo acumulado en rad y velocidad en rad/s de cada rueda). No abre el serial. Parámetros: `counts_per_rev` (cuentas por vuelta de **rueda** = pulsos por vuelta del encoder × 4 × reducción) e `invert` (6 booleanos, uno por rueda, para cambiar el signo). Sin `counts_per_rev` arranca en **modo calibración**: no publica `/wheel_states` y muestra los conteos; dale 10 vueltas a mano a una rueda marcada y divide entre 10 lo que cambió su conteo. Los valores medidos se pueden dejar fijos en `DEFAULT_COUNTS_PER_REV` y `DEFAULT_INVERT` al inicio del archivo.

**Canales del PCA9685** — puentes H TB6612FNG; ver detalle en `motor_command.py`:

| Puente H | Motores | AIN1, AIN2 (izq.) | BIN1, BIN2 (der.) |
|---|---|---|---|
| 1 (front) | front_left, front_right | 0, 1 | 2, 3 |
| 2 (mid) | mid_left, mid_right | 4, 5 | 6, 7 |
| 3 (rear) | rear_left, rear_right | 8, 9 | 10, 11 |

Canal 12 → señal del servo de la cámara. Canal 13 → señal del servo de la pala. Canales 14–15 libres.

**Pines de la ESP32 DevKit** (herramienta + encoders):

| Señal | GPIO |
|---|---|
| Stepper STEP, DIR | 18, 4 |
| Encoder front_left (A, B) | 32, 33 |
| Encoder mid_left (A, B) | 25, 26 |
| Encoder rear_left (A, B) | 27, 13 |
| Encoder front_right (A, B) | 23, 22 |
| Encoder mid_right (A, B) | 21, 19 |
| Encoder rear_right (A, B) | 17, 16 |

Los encoders se alimentan con **3.3 V** de la ESP (con 5 V sus salidas llegarían a 5 V y los pines de la ESP32 solo aguantan 3.3 V) y comparten GND con ella. Los 12 pines llevan pull-up interno. La ESP cuenta por hardware (PCNT), en cuadratura ×4.

STBY de los 3 puentes → 5 V fijo. PWMA y PWMB de cada puente van fijos a 3.3 V (el PWM se aplica sobre los AIN/BIN). VCC de los TB6612 → 3.3 V, VM → batería de motores (máx. 13.5 V).

Conexión PCA9685 ↔ Raspberry Pi: VCC → 3.3 V (pin 1), GND → GND (pin 6), SDA → GPIO 2 (pin 3), SCL → GPIO 3 (pin 5). OE a GND o sin conectar; V+ → fuente de 5–6 V para los servos de la cámara y de la pala (no los 3.3 V de la Pi: un servo puede pedir >1 A y reiniciarla, y el de la pala trabaja cargado: la fuente debe aguantar los dos a la vez). Habilitar I2C (`raspi-config` → Interface → I2C) y comprobar con `i2cdetect -y 1` (debe aparecer `40`). Las GND de la Pi, el PCA, los puentes H y la batería deben estar unidas.

### Uso

```bash
# Nodo de motores (en el Raspberry Pi del rover)
ros2 run locomotion motor_command

# Opción A: teclado (no arma: lanzar motor_command con -p require_arm:=false)
ros2 run locomotion keyboard_control

# Opción B: mando DualSense
ros2 run joy joy_node
ros2 run locomotion rc_control

# Stepper de la herramienta y conteos de encoders, en el Raspberry Pi
# (el servo de la pala lo mueve motor_command)
ros2 run locomotion tool_command

# Velocidad y ángulo de las ruedas (primero sin parámetros, para calibrar)
ros2 run locomotion encoder_node --ros-args -p counts_per_rev:=<cuentas por vuelta de rueda> \
    -p invert:="[false, false, false, true, true, true]"
ros2 topic echo /wheel_states
```

## Paquete `vision`

- **`camera_node`**: captura frames con OpenCV y los publica como `sensor_msgs/CompressedImage` (JPEG). Parámetros: `video_device`, `frame_rate`, `width`, `height`, `topic`, `frame_id`. QoS `BEST_EFFORT`/`VOLATILE`/`KEEP_LAST(1)` para minimizar latencia de streaming en vivo.
- **`image_viewer_node`**: se suscribe a un tópico `CompressedImage` y muestra el video en una ventana OpenCV, sin depender de `rqt`. Fuerza el backend Qt `xcb` para evitar problemas de tamaño de ventana bajo Wayland/GNOME.
- **`cameras.launch.py`**: lanza una instancia de `camera_node` por cada índice de `/dev/video*` indicado.

### Uso

```bash
# Una sola cámara
ros2 run vision camera_node --ros-args -p video_device:=0 -p frame_rate:=24

# Varias cámaras a la vez
ros2 launch vision cameras.launch.py devices:=0,2 frame_rate:=24

# Visualizar un stream
ros2 run vision image_viewer_node --ros-args -p topic:=/camera0/image/compressed
```

## Dependencias

- ROS 2 (`rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `launch`, `launch_ros`)
- `python3-smbus2` (I2C al PCA9685 de los motores en Raspberry Pi)
- `joy` (para `rc_control`)
- `cv_bridge`, `python3-opencv`

## Build

```bash
colcon build --packages-select locomotion vision
source install/setup.bash
```
