# Proyecto 4 — Grupo 3 (ASME IAM3D R.O.V.E.R.)

Workspace de ROS 2 con los paquetes de **locomoción** y **visión** del rover de tracción diferencial (skid-steer) de 6 ruedas / 3 puentes H, construido para la competencia ASME IAM3D R.O.V.E.R.

## Estructura del repositorio

```
src/
├── locomotion/          # Control de los 6 motores DC (teclado o mando DualSense)
│   ├── locomotion/
│   │   ├── motor_command.py     # Nodo que mapea /motor_cmd a los 6 motores vía GPIO
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

Controla 6 motores DC agrupados en 3 puentes H (2 motores por puente), organizados en tracción tipo skid-steer: todos los motores de un mismo lado (izquierdo/derecho) reciben el mismo comando.

- **`motor_command`**: se suscribe a `/motor_cmd` (`geometry_msgs/Twist`) y traduce `linear.x` (avance) y `angular.z` (giro) a un comando por lado, normalizando ambos lados juntos para preservar la diferencia de velocidad (el giro) cuando algún valor supera el rango. Corre a 50 Hz y usa `gpiozero` con el backend `lgpio`.
- **`keyboard_control`**: teleoperación por teclado. `w/s` sube/baja velocidad lineal, `a/d` gira, `espacio` detiene, `q` sale. Publica en `/motor_cmd`.
- **`rc_control`**: teleoperación con mando PS5 DualSense a través de `/joy`. Requiere tener corriendo `joy_node` del paquete `joy` en paralelo. Publica en `/motor_cmd`.

**Pines GPIO (BCM)** — ver detalle y advertencia de cableado en `motor_command.py`:

| Puente H | Motores | Pines |
|---|---|---|
| 1 (front) | front_left, front_right | 5, 6, 13, 19 |
| 2 (mid) | mid_left, mid_right | 17, 18, 27, 22 |
| 3 (rear) | rear_left, rear_right | 12, 16, 20, 21 |

Se dejan libres I2C, SPI0 y UART para un IMU.

### Uso

```bash
# Nodo de motores (en el Raspberry Pi del rover)
ros2 run locomotion motor_command

# Opción A: teclado
ros2 run locomotion keyboard_control

# Opción B: mando DualSense
ros2 run joy joy_node
ros2 run locomotion rc_control
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
- `gpiozero` + `lgpio` (control de motores en Raspberry Pi)
- `joy` (para `rc_control`)
- `cv_bridge`, `python3-opencv`

## Build

```bash
colcon build --packages-select locomotion vision
source install/setup.bash
```
