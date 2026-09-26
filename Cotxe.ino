/*
 * Talos Electronics - Control via ESP32 Sensor (cable directo)
 * ── Pin map ──────────────────────────────────────────────
 * ECO            → pin 10
 * TRIG           → pin 9
 * Sensor IR der  → pin 2
 * Servo          → pin 3
 * ENB            → pin 5  (PWM - motor derecho)
 * IN4            → pin 6
 * IN3            → pin 7
 * IN2            → pin 8
 * IN1            → pin 12
 * ENA            → pin 11 (PWM - motor izquierdo)
 * Sensor IR izq  → pin 13
 * ESP32 TX (18)  → pin 0  (RX Arduino)
 * ESP32 RX (19)  → pin 1  (TX Arduino)
 * ─────────────────────────────────────────────────────────
 *
 * NOTA: El HC-06 ha sido sustituido por un cable directo entre la
 * ESP32 sensor (SoftwareSerial en sus pines 18/19) y el Serial por
 * hardware del Arduino (pines 0/1). Es más fiable que el Bluetooth
 * y reutiliza el mismo Serial que ya usaba el HC-06, así que esta
 * parte del código no cambia: solo cambia qué hay físicamente
 * conectado al otro lado del cable.
 *
 * PROTOCOLO (recibido desde la ESP32 sensor):
 *   'F' → Adelante
 *   'B' → Atrás
 *   'R' → Derecha
 *   'L' → Izquierda
 *   'S' → Parar
 *   'Q' → Servo un paso hacia la izquierda
 *   'E' → Servo un paso hacia la derecha
 *
 * SEGURIDAD: si no llega ningún comando de movimiento (F/B/L/R)
 * durante WATCHDOG_TIMEOUT_MS, el coche se para solo. Esto protege
 * frente a una caída de WiFi/TCP entre el PC y la ESP32 sensor
 * mientras el coche está en marcha (si se corta la conexión, el
 * botón de soltar tecla nunca llegaría a enviarse).
 *
 * NOTA: Desconecta el cable ESP32↔Arduino (pines 0/1) antes de
 * subir el sketch por USB, igual que antes había que hacer con el
 * HC-06 (el bootloader usa esos mismos pines).
 * ─────────────────────────────────────────────────────────
 */

#include <Servo.h>

// ── Pines ─────────────────────────────────────────────────
#define TRIG      9
#define ECO      10
#define ENA      11
#define IN1      12
#define IN2       8
#define IN3       7
#define IN4       6
#define ENB       5
#define SERVO_PIN 3

Servo servo;

const int velocidad = 255;

// ── Servo: control manual por pasos (Q = izquierda, E = derecha) ──
// Variables fácilmente modificables sin tocar el resto del código.
int       servoPos      = 90;   // posición inicial (centro)
const int SERVO_MIN_DEG = 0;   // límite izquierdo
const int SERVO_MAX_DEG = 180;  // límite derecho
const int SERVO_STEP_DEG = 10;  // grados que se mueve por cada pulsación de Q/E

// ── Watchdog de seguridad: auto-stop si se pierde la conexión ────
const unsigned long WATCHDOG_TIMEOUT_MS = 300; // ms sin comandos de movimiento → parar
unsigned long        lastMotorCmdMillis  = 0;
bool                 motorsActivos       = false;

// ═════════════════════════════════════════════════════════
// ULTRASONIDOS
// ═════════════════════════════════════════════════════════
float leerDistancia() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECO, HIGH, 30000);
  if (t == 0) return 999.0;
  return t * 0.01715;
}

bool hayObstaculo() {
  return leerDistancia() <= 25.0;
}

// ═════════════════════════════════════════════════════════
// SERVO — movimiento manual por pasos (Q = izquierda, E = derecha)
// ═════════════════════════════════════════════════════════
void moverServoPaso(int delta) {
  servoPos += delta;
  if (servoPos > SERVO_MAX_DEG) servoPos = SERVO_MAX_DEG;
  if (servoPos < SERVO_MIN_DEG) servoPos = SERVO_MIN_DEG;
  servo.write(servoPos);
}

// ═════════════════════════════════════════════════════════
// MOTORES
// ═════════════════════════════════════════════════════════
void adelante() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, velocidad);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, velocidad);
}

void atras() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, velocidad);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, velocidad);
}

void derecha() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, velocidad);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, velocidad);
}

void izquierda() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, velocidad);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, velocidad);
}

void parar() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ═════════════════════════════════════════════════════════
void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECO,  INPUT);
  pinMode(ENA,  OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1,  OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3,  OUTPUT); pinMode(IN4, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(90);

  Serial.begin(9600);
}

// ═════════════════════════════════════════════════════════
void loop() {

  // ── Comandos desde la ESP32 sensor (cable directo, ex-HC-06) ──
  if (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case 'F': adelante();  lastMotorCmdMillis = millis(); motorsActivos = true;  break;
      case 'B': atras();     lastMotorCmdMillis = millis(); motorsActivos = true;  break;
      case 'R': derecha();   lastMotorCmdMillis = millis(); motorsActivos = true;  break;
      case 'L': izquierda(); lastMotorCmdMillis = millis(); motorsActivos = true;  break;
      case 'S': parar();     lastMotorCmdMillis = millis(); motorsActivos = false; break;
      case 'Q': moverServoPaso(-SERVO_STEP_DEG); break; // izquierda
      case 'E': moverServoPaso(+SERVO_STEP_DEG); break; // derecha
    }
  }

  // ── Watchdog: si el coche está en marcha y no llega ningún
  // comando de movimiento durante WATCHDOG_TIMEOUT_MS, paramos
  // solos. Protege ante una caída de WiFi/TCP entre el PC y la
  // ESP32 sensor mientras el coche estaba avanzando/girando.
  if (motorsActivos && (millis() - lastMotorCmdMillis > WATCHDOG_TIMEOUT_MS)) {
    parar();
    motorsActivos = false;
    Serial.println("WATCHDOG: sin comandos, coche parado.");
  }
}
