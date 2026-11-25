# Proyecto 4 — Grupo 3 (ASME IAM3D R.O.V.E.R.)

Workspace de ROS 2 con los paquetes de **locomoción** y **visión** del rover de tracción diferencial (skid-steer) de 6 ruedas, construido para la competencia ASME IAM3D R.O.V.E.R.

## Estructura del repositorio

```
src/
├── locomotion/          # Control de las 6 ruedas: 4 servos de velocidad + 2 motores DC (teclado o mando DualSense)
│   ├── locomotion/
│   │   ├── motor_command.py     # Nodo del PCA9685 (I2C): 4 servos de rueda, 2 motores DC, servo de cámara y servo de pala
│   │   ├── tool_command.py      # Puente serial a la ESP32 DevKit: stepper (/stepper_steps, /stepper_stop)
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

Controla 6 ruedas en tracción tipo skid-steer: las delanteras y medias llevan **servos de rotación continua** (de velocidad) y las traseras **motorreductores DC** en un puente H TB6612FNG. Todo va al mismo PCA9685 y todos los motores de un mismo lado (izquierdo/derecho) reciben el mismo comando.

- **`motor_command`**: se suscribe a `/motor_cmd` (`geometry_msgs/Twist`) y traduce `linear.x` (avance) y `angular.z` (giro) a un comando por lado, normalizando ambos lados juntos para preservar la diferencia de velocidad (el giro) cuando algún valor supera el rango. Corre a 50 Hz y genera con un PCA9685 por I2C (`smbus2`) los pulsos de los servos de rueda (1500 µs = quieto, 1000/2000 µs = tope en cada sentido; con el comando en ~0 no manda pulsos para que no se arrastren) y el PWM del puente H de los traseros. También mueve el **servo de inclinación de la cámara** en el canal 9 del mismo PCA9685 (tópicos `/camera_tilt_delta` y `/camera_tilt_angle`, `Int32` en grados). Parámetros: `i2c_bus` (1), `i2c_address` (`0x40`), `servo_invert` (4 booleanos en orden front_right, mid_right, front_left, mid_left; por defecto `[true, true, false, false]` porque los derechos van espejados), `servo_trim_us` (4 enteros, corrección del punto muerto de cada servo), `servo_center_us` (1500), `servo_span_us` (500), `servo_deadband` (0.02), `require_arm` (`true`: frena las 6 ruedas mientras `/armed` no sea `true`), `cmd_timeout` (0.5 s: frena si deja de llegar `/motor_cmd`; `0` lo desactiva), `pwm_frequency` (50 Hz: el PCA tiene una sola frecuencia para todos los canales y los servos la necesitan baja), `cam_channel` (9), `cam_min_deg`/`cam_max_deg` (20/160), `cam_start_deg` (90), `cam_pulse_min_us`/`cam_pulse_max_us` (500/2400). El **servo de la pala** va en el canal 8 (tópicos `/servo_delta` y `/servo_angle`, `Int32` en grados; siempre se mueve con rampa de ~50 °/s): `shovel_channel` (8), `shovel_min_deg`/`shovel_max_deg` (0/180), `shovel_start_deg` (30), `shovel_pulse_min_us`/`shovel_pulse_max_us` (500/2400).
- **`keyboard_control`**: teleoperación por teclado. `w/s` sube/baja velocidad lineal, `a/d` gira, `espacio` detiene, `q` sale. Publica en `/motor_cmd`.
- **`rc_control`**: teleoperación con mando PS5 DualSense a través de `/joy`. Requiere tener corriendo `joy_node` del paquete `joy` en paralelo. Publica en `/motor_cmd`. **Arming switch:** arranca *desarmado*; **Options** arma (solo con gatillos y joystick izquierdo en reposo) y **Create** desarma (además manda `/stepper_stop`). Desarmado publica `/motor_cmd` en cero y no manda nada a la herramienta ni a la cámara. Si `/joy` deja de llegar más de 0.5 s se desarma solo. El estado va en `/armed` (`Bool`, latched). Además controla la herramienta de excavación: **X** alterna entre modo *stepper* (inicial) y modo *servo*; **R1** = positivo y **L1** = negativo sobre el modo activo (se repite mientras se mantiene el botón). Publica en `/stepper_steps` y `/servo_delta`. El **joystick derecho (vertical)** inclina la cámara por velocidad (más empuje = más rápido; al soltar se queda quieta) y **R3** la centra; publica en `/camera_tilt_delta` y `/camera_tilt_angle`. Constantes `AX_CAM_TILT`, `CAM_INVERT`, `CAM_MAX_DEG_S` al inicio del archivo. Los índices de botones (`BTN_*`) están al inicio de `rc_control.py`; verificar con `ros2 topic echo /joy`.
- **`tool_command`**: puente serial hacia la **ESP32 DevKit** (ESP32 clásica), que mueve el stepper NEMA 17 con A4988 (firmware en `firmware/tool_esp32/tool_esp32.ino`; librería `AccelStepper`). Es el único nodo que abre ese puerto. El servo de la pala no pasa por la ESP: va en el PCA9685 (ver `motor_command`). Se suscribe a `/stepper_steps` (`Int32`, pasos relativos → `"<pasos>"`) y `/stepper_stop` (`Empty` → `x`); lo que responde la ESP sale en el log. Parámetros: `port` (`/dev/ttyUSB0`), `baudrate` (115200). Reconecta solo si la ESP se desconecta.
**Canales del PCA9685** — ver detalle en `motor_command.py`:

| Canal | Conexión |
|---|---|
| 0 | Servo de velocidad front_right |
| 1 | Servo de velocidad mid_right |
| 2 | Servo de velocidad front_left |
| 3 | Servo de velocidad mid_left |
| 4, 5 | Puente H pines 1, 2 (AIN1, AIN2) → rear_left |
| 6, 7 | Puente H pines 3, 4 (BIN1, BIN2) → rear_right |
| 8 | Servo de la pala |
| 9 | Servo de la cámara |
| 10–15 | Libres |

**Pines de la ESP32 DevKit** (herramienta): stepper STEP → GPIO 13, DIR → GPIO 27.

**Colores de los cables de los motores traseros** — los cables originales del motor se alargaron con extensiones, así que el color cambia a mitad del cable:

| Pin (color original) | Izq. trasera | Der. trasera |
|---|---|---|
| M2 (rojo) | Rojo | Rojo |
| M1 (blanco) | Amarillo + Blanco (bicolor) | Naranja |

STBY del puente H → 5 V fijo. PWMA y PWMB van fijos a 3.3 V (el PWM se aplica sobre los AIN/BIN). VCC del TB6612 → 3.3 V, VM → batería de motores (máx. 13.5 V).

Conexión PCA9685 ↔ Raspberry Pi: VCC → 3.3 V (pin 1), GND → GND (pin 6), SDA → GPIO 2 (pin 3), SCL → GPIO 3 (pin 5). OE a GND o sin conectar; V+ → fuente de 5–6 V para los 6 servos (4 de rueda, cámara y pala; no los 3.3 V de la Pi: un servo puede pedir >1 A y reiniciarla). Los 4 servos de rueda arrancan a la vez y trabajan cargados: la fuente debe aguantar varios amperes, y conviene llevar ese cable directo a la bornera V+ del PCA con calibre grueso. Habilitar I2C (`raspi-config` → Interface → I2C) y comprobar con `i2cdetect -y 1` (debe aparecer `40`). Las GND de la Pi, el PCA, el puente H y la batería deben estar unidas.

### Uso

```bash
# Nodo de motores (en el Raspberry Pi del rover)
ros2 run locomotion motor_command

# Opción A: teclado (no arma: lanzar motor_command con -p require_arm:=false)
ros2 run locomotion keyboard_control

# Opción B: mando DualSense
ros2 run joy joy_node
ros2 run locomotion rc_control

# Stepper de la herramienta, en el Raspberry Pi
# (el servo de la pala lo mueve motor_command)
ros2 run locomotion tool_command

```

## Paquete `vision`

- **`camera_node`**: captura frames con OpenCV y los publica como `sensor_msgs/CompressedImage` (JPEG). Parámetros: `video_device`, `frame_rate`, `width`, `height`, `topic`, `frame_id`. QoS `BEST_EFFORT`/`VOLATILE`/`KEEP_LAST(1)` para minimizar latencia de streaming en vivo.
- **`image_viewer_node`**: se suscribe a un tópico `CompressedImage` y muestra el video en una ventana OpenCV, sin depender de `rqt`. Fuerza el backend Qt `xcb` para evitar problemas de tamaño de ventana bajo Wayland/GNOME.
- **`rover_panel`** (`./lanzar_panel.sh` en el PC): panel gráfico (Tk) para abrir/cerrar los nodos de la Pi por SSH. El video de la cámara va **dentro del mismo panel** (sin ventana emergente): el botón «Video» suscribe el panel al video, al estado del enlace, a `/joy` y a `/armed`. Requiere `python3-tk`, `python3-opencv` y ROS 2 cargado; sin ellos el panel abre pero sin video. `image_viewer_node` sigue disponible como visor aparte.
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
