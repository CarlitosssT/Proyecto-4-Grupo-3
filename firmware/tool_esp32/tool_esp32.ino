/*
  Herramienta (stepper) + encoders de las 6 ruedas — ESP32 DevKit (ESP32 clásica)

  Recibe comandos de texto por USB-serial (115200, fin de línea '\n') desde el
  nodo ROS 2 tool_command (Raspberry Pi) y le manda los conteos de los encoders.
  También sirve para probar a mano desde el Monitor Serie.

  El servo de la pala ya NO va aquí: está en el PCA9685 (lo mueve motor_command).

  PLACA: ESP32 Dev Module (WROOM). No sirve tal cual en una C3: estos pines no
  existen ahí y la C3 no tiene contadores de pulsos por hardware (PCNT).

  LIBRERÍAS (Gestor de Librerías):
    - "AccelStepper" de Mike McCauley
    - "ESP32Encoder" de Kevin Harrington

  COMANDOS DEL STEPPER:
    200 / -200 -> pasos RELATIVOS, se suman al movimiento pendiente
    0          -> poner la posición actual como cero
    x          -> parar (frena con la aceleración configurada)

  COMANDOS DE LOS ENCODERS:
    r<hz>      -> reportes por segundo (r0 = apagar; por defecto 20; máximo 100)
    z          -> poner los 6 conteos en cero

  GENERAL:
    s          -> mostrar estado

  REPORTE DE ENCODERS (una línea por reporte; la lee tool_command):
    E <micros> <front_left> <mid_left> <rear_left> <front_right> <mid_right> <rear_right>
  <micros> es el reloj de la ESP al leer los contadores (da la vuelta cada ~71 min).
  Los conteos van en cuadratura x4: 4 cuentas por cada pulso del encoder.
  Ninguna otra línea empieza por "E " (tool_command se fía de eso).

  CABLEADO DE LOS ENCODERS:
    - Alimentarlos con 3.3 V de la ESP. Con 5 V las salidas A/B llegarían a 5 V
      y los pines de la ESP32 solo aguantan 3.3 V (haría falta un divisor).
    - GND del encoder unida a GND de la ESP.
    - Los 12 pines tienen pull-up interno activado.
*/

#include <AccelStepper.h>
#include <ESP32Encoder.h>

// ===================== PINES =====================
const int PIN_STEP = 13;
const int PIN_DIR  = 27;
// EN del A4988 sin conectar: el módulo lo deja activo por defecto

// {canal A, canal B}. El orden de las filas es el orden del reporte.
// Izquierdos en la tira izquierda de la DevKit, derechos en la derecha.
// Si una rueda cuenta al revés: intercambiar A y B aquí, o usar el parámetro
// "invert" del nodo encoder_node.
const int NUM_ENCODERS = 6;
const int PIN_ENC[NUM_ENCODERS][2] = {
  {32, 33},   // front_left
  {25, 26},   // mid_left
  {18, 4},    // rear_left
  {23, 22},   // front_right
  {21, 19},   // mid_right
  {2, 15},    // rear_right (pines de arranque: si falla la carga del firmware, desconectar sus cables)
};
// Libres: 14 y, solo como entradas sin pull-up, 34, 35, 36 y 39.
// No usar 0, 5, 12 (arranque) ni 6-11 (memoria flash). 2 y 15 también son de arranque
// y se usan en rear_right porque no quedaban otros: ver nota arriba.
// =================================================

// ===================== STEPPER ===================
const float VELOCIDAD_MAX = 400;      // pasos por segundo
const float ACELERACION   = 200;      // pasos por segundo²
// =================================================

// ===================== ENCODERS ==================
const int REPORTES_HZ     = 20;       // reportes por segundo al arrancar
const int REPORTES_HZ_MAX = 100;      // más que esto no cabe bien a 115200 baudios
// =================================================

AccelStepper stepper(1, PIN_STEP, PIN_DIR);
ESP32Encoder encoders[NUM_ENCODERS];

bool stepperMoviendo = false;
unsigned long periodoReporteUs = 1000000UL / REPORTES_HZ;   // 0 = apagado
unsigned long ultimoReporteUs = 0;

// ======================= ENCODERS =======================

void reportarEncoders() {
  Serial.print("E ");
  Serial.print(micros());
  for (int i = 0; i < NUM_ENCODERS; i++) {
    Serial.print(' ');
    Serial.print((long)encoders[i].getCount());
  }
  Serial.println();
}

void ponerReportes(int hz) {
  if (hz > REPORTES_HZ_MAX) hz = REPORTES_HZ_MAX;
  if (hz <= 0) {
    periodoReporteUs = 0;
    Serial.println("Reportes de encoders apagados.");
  } else {
    periodoReporteUs = 1000000UL / hz;
    Serial.print("Reportes de encoders: ");
    Serial.print(hz);
    Serial.println(" por segundo.");
  }
}

// ======================= ESTADO =======================

void mostrarEstado() {
  Serial.println("========== ESTADO ==========");
  Serial.print("Stepper: posición "); Serial.print(stepper.currentPosition());
  Serial.print(", faltan "); Serial.println(stepper.distanceToGo());
  Serial.print("Conteos:");
  for (int i = 0; i < NUM_ENCODERS; i++) {
    Serial.print(' ');
    Serial.print((long)encoders[i].getCount());
  }
  Serial.println();
  Serial.print("Reportes por segundo: ");
  Serial.println(periodoReporteUs == 0 ? 0 : (int)(1000000UL / periodoReporteUs));
  Serial.println("============================");
}

void mostrarAyuda() {
  Serial.println("Stepper:  <pasos> (ej: 200 o -200), 0 = cero, x = parar");
  Serial.println("Conteos:  r<hz> = reportes por segundo (r0 apaga), z = poner en cero");
  Serial.println("General:  s = estado");
}

// ======================= COMANDOS =======================

void procesarComando(String texto) {
  texto.trim();
  texto.toLowerCase();
  if (texto.length() == 0) return;

  char letra = texto.charAt(0);

  if (letra == 'x') {
    stepper.stop();
    Serial.println("Stepper: parando...");
  }
  else if (isDigit(letra) || letra == '-') {
    long pasos = texto.toInt();
    if (pasos == 0) {
      stepper.setCurrentPosition(0);
      Serial.println("Stepper: posición actual marcada como 0.");
    } else {
      stepper.move(pasos);
      stepperMoviendo = true;
    }
  }
  else if (letra == 'r') ponerReportes(texto.substring(1).toInt());
  else if (letra == 'z') {
    for (int i = 0; i < NUM_ENCODERS; i++) encoders[i].clearCount();
    Serial.println("Conteos en cero.");
  }
  else if (letra == 's') mostrarEstado();
  else {
    Serial.println("Comando no reconocido.");
    mostrarAyuda();
  }
}

// ======================= SETUP Y LOOP =======================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Herramienta (stepper) + encoders - ESP32 DevKit");

  stepper.setMaxSpeed(VELOCIDAD_MAX);
  stepper.setAcceleration(ACELERACION);

  // Cuenta por hardware (PCNT): no se pierden pulsos aunque el loop esté ocupado
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  for (int i = 0; i < NUM_ENCODERS; i++) {
    encoders[i].attachFullQuad(PIN_ENC[i][0], PIN_ENC[i][1]);
    encoders[i].clearCount();
  }

  mostrarEstado();
  mostrarAyuda();
}

void loop() {
  // Lee sin bloquear: acumula hasta '\n' (readStringUntil bloquearía el stepper)
  static String buffer = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      procesarComando(buffer);
      buffer = "";
    } else if (c != '\r' && buffer.length() < 32) {
      buffer += c;
    }
  }

  stepper.run();

  if (stepperMoviendo && stepper.distanceToGo() == 0) {
    stepperMoviendo = false;
    Serial.print("Stepper listo. Posición actual: ");
    Serial.println(stepper.currentPosition());
  }

  unsigned long ahora = micros();
  if (periodoReporteUs > 0 && ahora - ultimoReporteUs >= periodoReporteUs) {
    ultimoReporteUs = ahora;
    reportarEncoders();
  }
}
