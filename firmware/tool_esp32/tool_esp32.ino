/*
  Herramienta (stepper) — ESP32 DevKit (ESP32 clásica)

  Recibe comandos de texto por USB-serial (115200, fin de línea '\n') desde el
  nodo ROS 2 tool_command (Raspberry Pi). También sirve para probar a mano
  desde el Monitor Serie.

  El servo de la pala ya NO va aquí: está en el PCA9685 (lo mueve motor_command).

  PLACA: ESP32 Dev Module (WROOM).

  LIBRERÍAS (Gestor de Librerías):
    - "AccelStepper" de Mike McCauley

  COMANDOS DEL STEPPER:
    200 / -200 -> pasos RELATIVOS, se suman al movimiento pendiente
    0          -> poner la posición actual como cero
    x          -> parar (frena con la aceleración configurada)

  GENERAL:
    s          -> mostrar estado
*/

#include <AccelStepper.h>

// ===================== PINES =====================
const int PIN_STEP = 13;
const int PIN_DIR  = 27;
// EN del A4988 sin conectar: el módulo lo deja activo por defecto
// =================================================

// ===================== STEPPER ===================
const float VELOCIDAD_MAX = 800;      // pasos por segundo
const float ACELERACION   = 800;      // pasos por segundo²
// =================================================

AccelStepper stepper(1, PIN_STEP, PIN_DIR);

bool stepperMoviendo = false;

// ======================= ESTADO =======================

void mostrarEstado() {
  Serial.println("========== ESTADO ==========");
  Serial.print("Stepper: posición "); Serial.print(stepper.currentPosition());
  Serial.print(", faltan "); Serial.println(stepper.distanceToGo());
  Serial.println("============================");
}

void mostrarAyuda() {
  Serial.println("Stepper:  <pasos> (ej: 200 o -200), 0 = cero, x = parar");
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
  Serial.println("Herramienta (stepper) - ESP32 DevKit");

  stepper.setMaxSpeed(VELOCIDAD_MAX);
  stepper.setAcceleration(ACELERACION);

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
}
