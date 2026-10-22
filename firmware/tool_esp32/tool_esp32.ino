/*
  Herramienta de excavación (servo de pala + stepper) — ESP32 mini

  Recibe comandos de texto por USB-serial (115200, fin de línea '\n') desde el
  nodo ROS 2 tool_command (Raspberry Pi). También sirve para probar a mano
  desde el Monitor Serie.

  LIBRERÍAS (Gestor de Librerías):
    - "ESP32Servo"   de Kevin Harrington
    - "AccelStepper" de Mike McCauley

  COMANDOS DEL SERVO:
    e / c / v  -> ir a EXCAVAR / SUBIR (carga) / DESCARGAR (volteo)
    p          -> ciclo: excavar -> subir -> descargar -> subir
    + / -      -> mover 1 grado
    d<n>       -> sumar n grados al ángulo actual (d5, d-3)  [lo usa ROS]
    g<ángulo>  -> ir a un ángulo exacto (g45)                [lo usa ROS]
    l / a      -> liberar / volver a enganchar el servo

  COMANDOS DEL STEPPER:
    200 / -200 -> pasos RELATIVOS, se suman al movimiento pendiente
    0          -> poner la posición actual como cero
    x          -> parar (frena con la aceleración configurada)

  GENERAL:
    s          -> mostrar estado
*/

#include <ESP32Servo.h>
#include <AccelStepper.h>

// ===================== PINES =====================
const int PIN_SERVO = 4;
const int PIN_STEP  = 7;
const int PIN_DIR   = 8;
// EN del A4988 sin conectar: el módulo lo deja activo por defecto
// =================================================

// ===================== SERVO =====================
const int LIMITE_MIN = 5;
const int LIMITE_MAX = 130;

int anguloExcavar = 45;
int anguloCarga   = 30;
int anguloVolteo  = 90;

const int MS_POR_GRADO = 15;          // velocidad del servo

const int CICLOS_PRUEBA = 3;
const int ESPERA_EXCAVAR_MS = 2000;
const int ESPERA_SUBIR_MS = 1500;
const int ESPERA_VOLTEO_MS = 1500;

const int PULSO_MIN_US = 500;
const int PULSO_MAX_US = 2400;
// =================================================

// ===================== STEPPER ===================
const float VELOCIDAD_MAX = 400;      // pasos por segundo
const float ACELERACION   = 200;      // pasos por segundo²
// =================================================

Servo servoPala;
AccelStepper stepper(1, PIN_STEP, PIN_DIR);

int anguloActual = 30;
bool enganchado = false;
unsigned long ciclosHechos = 0;
bool stepperMoviendo = false;

// Espera que no detiene al stepper
void esperar(unsigned long ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    stepper.run();
  }
}

// ======================= SERVO =======================

int limitar(int angulo) {
  if (angulo < LIMITE_MIN) return LIMITE_MIN;
  if (angulo > LIMITE_MAX) return LIMITE_MAX;
  return angulo;
}

// 'verbose' = false para los comandos continuos (d, +, -) y no inundar el serial
void moverSuave(int destino, bool verbose = true) {
  destino = limitar(destino);
  if (!enganchado) {
    Serial.println("El servo está suelto. Escribe 'a' para engancharlo.");
    return;
  }
  if (verbose) {
    Serial.print("Servo: moviendo de ");
    Serial.print(anguloActual);
    Serial.print(" a ");
    Serial.println(destino);
  }
  while (anguloActual != destino) {
    if (anguloActual < destino) anguloActual++;
    else anguloActual--;
    servoPala.write(anguloActual);
    esperar(MS_POR_GRADO);
  }
  if (verbose) {
    Serial.print("Servo listo. Ángulo actual: ");
    Serial.println(anguloActual);
  }
}

void engancharServo() {
  servoPala.setPeriodHertz(50);
  servoPala.attach(PIN_SERVO, PULSO_MIN_US, PULSO_MAX_US);
  servoPala.write(anguloActual);
  enganchado = true;
  Serial.println("Servo enganchado.");
}

void soltarServo() {
  servoPala.detach();
  enganchado = false;
  Serial.println("Servo suelto (sin fuerza).");
}

void cicloDePrueba() {
  for (int i = 1; i <= CICLOS_PRUEBA; i++) {
    Serial.print("--- Ciclo ");
    Serial.print(i);
    Serial.print(" de ");
    Serial.println(CICLOS_PRUEBA);

    moverSuave(anguloExcavar);
    esperar(ESPERA_EXCAVAR_MS);
    moverSuave(anguloCarga);
    esperar(ESPERA_SUBIR_MS);
    moverSuave(anguloVolteo);
    esperar(ESPERA_VOLTEO_MS);
    moverSuave(anguloCarga);
    esperar(500);

    ciclosHechos++;
  }
}

// ======================= ESTADO =======================

void mostrarEstado() {
  Serial.println("========== ESTADO ==========");
  Serial.print("Servo: ángulo "); Serial.print(anguloActual);
  Serial.print(" (límites "); Serial.print(LIMITE_MIN); Serial.print("-"); Serial.print(LIMITE_MAX);
  Serial.print("), enganchado: "); Serial.println(enganchado ? "sí" : "no");
  Serial.print("Stepper: posición "); Serial.print(stepper.currentPosition());
  Serial.print(", faltan "); Serial.println(stepper.distanceToGo());
  Serial.println("============================");
}

void mostrarAyuda() {
  Serial.println("Servo:   e, c, v, p, +, -, d<n>, g<ángulo>, l, a");
  Serial.println("Stepper: <pasos> (ej: 200 o -200), 0 = cero, x = parar");
  Serial.println("General: s = estado");
}

// ======================= COMANDOS =======================

void procesarComando(String texto) {
  texto.trim();
  texto.toLowerCase();
  if (texto.length() == 0) return;

  char letra = texto.charAt(0);

  if (letra == 'e')      { Serial.println(">> EXCAVAR");      moverSuave(anguloExcavar); }
  else if (letra == 'c') { Serial.println(">> SUBIR ARENA");  moverSuave(anguloCarga); }
  else if (letra == 'v') { Serial.println(">> DESCARGAR");    moverSuave(anguloVolteo); }
  else if (letra == 'p') cicloDePrueba();
  else if (texto == "+") moverSuave(anguloActual + 1, false);
  else if (texto == "-") moverSuave(anguloActual - 1, false);
  else if (letra == 'd') moverSuave(anguloActual + texto.substring(1).toInt(), false);
  else if (letra == 'g') moverSuave(texto.substring(1).toInt());
  else if (letra == 'l') soltarServo();
  else if (letra == 'a') engancharServo();
  else if (letra == 'x') {
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
  delay(1500);

  Serial.println();
  Serial.println("Herramienta servo + stepper - ESP32");

  stepper.setMaxSpeed(VELOCIDAD_MAX);
  stepper.setAcceleration(ACELERACION);

  anguloActual = limitar(anguloCarga);
  engancharServo();

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
