#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include "HUSKYLENS.h"
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>   // NOU: librería "EspSoftwareSerial" (instálala desde el Library Manager,
                              // buscando "EspSoftwareSerial" de Peter Lerup — NO la SoftwareSerial de AVR)

// =====================================================================
// INSTRUCCIONES DEL TECLADO 4x4 + PANTALLA LCD
// =====================================================================
//
//  Esta interfaz (teclado 4x4 + LCD) es INDEPENDIENTE del WiFi: funciona
//  desde el primer segundo, tanto si hay WiFi como si no, ya que las
//  tareas se crean antes de intentar conectar. Lo único que necesita
//  para poder enviar la IP a la ESP32-CAM es que la UART hacia la CAM
//  (SerialCAM) se haya inicializado correctamente en el setup().
//
//  MODO MENÚ (UI_MENU):
//    A   -> Subir una opción en el menú
//    B   -> Bajar una opción en el menú
//    #   -> Confirmar / activar la opción resaltada (con ">")
//    1,2,3... -> Acceso directo opcional a la opción N
//
//  MODO INTRODUCIR IP (UI_INPUT_IP):
//    0-9 -> Escribir los dígitos de la IP
//    D   -> Insertar un punto "." (ej: 192 D 168 D 1 D 42)
//    *   -> Borrar el último carácter escrito.
//           Si el campo ya está vacío, vuelve al menú principal.
//    #   -> Confirmar y aplicar la IP introducida (si es válida)
//
//  MODO VER ESTADO (UI_INFO):
//    *   -> Volver al menú principal
//
//  La tecla 'C' queda libre, sin usar, para futuras ampliaciones.
//
//  Todas estas teclas se pueden cambiar fácilmente más abajo, en la
//  sección "CONFIGURACIÓN DE TECLAS DE NAVEGACIÓN (editable)"
//  (variables KEY_MENU_UP, KEY_MENU_DOWN, KEY_CONFIRM, KEY_BACKSPACE,
//  KEY_IP_DOT) sin tener que tocar el resto del código.
//
// =====================================================================
// CONEXIÓN CON EL ARDUINO DEL COCHE (cable directo, ex-HC-06)
// =====================================================================
//
//  Esta ESP32 sensor reenvía al Arduino, por una UART software propia
//  (pines ARDUINO_RX_PIN/ARDUINO_TX_PIN, ver más abajo), cualquier
//  comando de coche/servo que llegue por TCP desde test.py:
//    F/B/L/R/S → movimiento del coche      Q/E → servo del ultrasonido
//
//  Mientras el coche está en marcha (F/B/L/R activos) y la conexión
//  TCP con el PC sigue viva, esta ESP32 reenvía el último comando al
//  Arduino cada CAR_RESEND_INTERVAL_MS, para alimentar el watchdog de
//  seguridad del propio Arduino (que se para solo si deja de recibir
//  comandos). Si el TCP se cae, esta ESP32 deja de reenviar y el
//  Arduino se parará solo en cuanto venza su watchdog.
// =====================================================================

// =====================================================
// CONFIGURACIÓ DE RED
// =====================================================
const char*    WIFI_SSID          = "Kai";
const char*    WIFI_PASSWORD      = "12345678";
char           SERVER_IP[20]      = "192.168.1.42";
const uint16_t SERVER_PORT        = 5000;
const uint32_t PUSH_INTERVAL_MS   = 500;
const uint32_t RECONN_INTERVAL_MS = 3000;

// Intentos iniciales (de 500ms) para conectar al WiFi en el setup().
// Si se agotan sin éxito, el ESP32 YA NO se reinicia: continúa
// funcionando (teclado, LCD, sensores, UART hacia la CAM) y el loop()
// principal seguirá reintentando la conexión en segundo plano.
const uint8_t  WIFI_CONNECT_TRIES = 10;

// ── NOU: IP que s'enviarà a la ESP32-CAM per UART ──────────────
// De moment és la mateixa que SERVER_IP.
// En el futur, aquest buffer es pot omplir des d'un teclat 4x4 + LCD
// seguint l'esquema al final del fitxer.
char cam_target_ip[20];          // s'inicialitza al setup() des de SERVER_IP
// ──────────────────────────────────────────────────────────────

// =====================================================
// HUSKYLENS — Configuració
// =====================================================
#define HUSKY_RX          4
#define HUSKY_TX          5
#define HUSKY_ALGO        ALGORITHM_OBJECT_RECOGNITION
#define HUSKY_INTERVAL_MS 200

const char* huskyAlgoName(int algo) {
    switch (algo) {
        case ALGORITHM_FACE_RECOGNITION:      return "FACE_RECOGNITION";
        case ALGORITHM_OBJECT_TRACKING:       return "OBJECT_TRACKING";
        case ALGORITHM_OBJECT_RECOGNITION:    return "OBJECT_RECOGNITION";
        case ALGORITHM_LINE_TRACKING:         return "LINE_TRACKING";
        case ALGORITHM_COLOR_RECOGNITION:     return "COLOR_RECOGNITION";
        case ALGORITHM_TAG_RECOGNITION:       return "TAG_RECOGNITION";
        case ALGORITHM_OBJECT_CLASSIFICATION: return "OBJECT_CLASSIFICATION";
        default:                              return "UNKNOWN";
    }
}

// =====================================================
// CAM UART — Configuració
// =====================================================
#define CAM_RX_PIN 34
#define CAM_TX_PIN 27 
char cam_server_ip[20] = "0.0.0.0";

HardwareSerial SerialCAM(1);

volatile int   cam_rssi = -100;
volatile float cam_temp = 0.0f;
char           cam_ip[20] = "0.0.0.0";
SemaphoreHandle_t camMutex;

// ── NOU: interval d'enviament de la IP cap a la CAM ────────────
#define CAM_IP_SEND_INTERVAL_MS 5000
// ──────────────────────────────────────────────────────────────

// =====================================================
// ARDUINO COCHE — UART software (cable directo, ex-HC-06)
// =====================================================
// Pines 18/19 elegits perquè els 3 UART de hardware ja estan ocupats:
//   UART0 (Serial)    → USB / debug
//   UART1 (SerialCAM) → ESP32-CAM (pins 34/12)
//   UART2 (SerialHUSKY) → HuskyLens (pins 4/5)
#define ARDUINO_RX_PIN 18   // entra: TX de l'Arduino (pin 1) → aquí
#define ARDUINO_TX_PIN 19   // surt:  aquí → RX de l'Arduino (pin 0)
#define ARDUINO_BAUD   9600

SoftwareSerial SerialARDUINO(ARDUINO_RX_PIN, ARDUINO_TX_PIN); // (rx, tx)

// Mentre el cotxe estigui en marxa, reenviem l'última comanda cada
// CAR_RESEND_INTERVAL_MS per alimentar el watchdog de seguretat de
// l'Arduino (WATCHDOG_TIMEOUT_MS allà ha de ser sempre més gran que
// aquest interval, amb marge).
#define CAR_RESEND_INTERVAL_MS 100
char          lastCarCmd       = 'S';
bool          carMoviment      = false; // true mentre F/B/L/R estiguin actius
unsigned long lastCarSendMs    = 0;
// ──────────────────────────────────────────────────────────────
// =====================================================
// TECLAT 4x4 + LCD I2C — Configuració
// =====================================================
#define LCD_I2C_ADDR 0x27   // canvia a 0x3F si el teu mòdul fa servir aquesta adreça
// OJO: aquí había 18 columnas x 19 filas, que no existen en ningún LCD
// real y rompían el direccionamiento interno de la librería. Lo normal
// es 16x2 (el más común) o 20x4. Ajusta estos dos valores a tu pantalla.
#define LCD_COLS     16
#define LCD_ROWS     2
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

const byte KEYPAD_ROWS = 4;
const byte KEYPAD_COLS = 4;
char keypadKeys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// =====================================================
// TECLAT 4x4 + LCD I2C — Configuración Definitiva (Sin tocar la CAM)
// =====================================================
byte keypadRowPins[KEYPAD_ROWS] = {26, 25, 33, 32}; // Filas (Pines 1 al 4 del teclado)
byte keypadColPins[KEYPAD_COLS] = {13, 14, 15, 23};
Keypad keypad = Keypad(makeKeymap(keypadKeys), keypadRowPins, keypadColPins, KEYPAD_ROWS, KEYPAD_COLS);

// ── CONFIGURACIÓN DE TECLAS DE NAVEGACIÓN (editable) ────────────
// Cambia aquí qué tecla física hace cada función, sin tocar el resto
// del código. Deben coincidir con caracteres presentes en keypadKeys[].
char KEY_MENU_UP   = 'A';  // sube una opción en el menú
char KEY_MENU_DOWN = 'B';  // baja una opción en el menú
char KEY_CONFIRM   = '#';  // confirma selección de menú / confirma IP
char KEY_BACKSPACE = '*';  // borra último carácter / vuelve atrás
char KEY_IP_DOT    = 'D';  // inserta el punto "." al escribir la IP
// ──────────────────────────────────────────────────────────────

enum UIState { UI_MENU, UI_INPUT_IP, UI_INFO, UI_CAM_INFO };
UIState uiState = UI_MENU;

const char* MENU_ITEMS[] = {
  "1.Canviar IP",
  "2.Reset IMU",
  "3.Veure estat",
  "4.IP Camara"
};
const uint8_t MENU_COUNT = 4;
int8_t menuIndex = 0;

String ipInputBuffer = "";

SemaphoreHandle_t ipMutex;       // protegeix SERVER_IP i cam_target_ip
volatile bool sendIPNow = false; // força enviament immediat de la IP a la CAM

// =====================================================
// PROTOTIPOS
// =====================================================
void iniciarWiFi();
bool connectarServidor();
void taskIMU(void* pvParameters);
void taskHuskyLens(void* pvParameters);
void taskCamSerial(void* pvParameters);
void recalibrarIMU();
void enviarDades();
void enviarHusky();
void processarComanda(const String& cmd);
bool mpuInit();
void mpuRead(float&, float&, float&, float&, float&, float&, float&, float&, float&);
void taskUI(void* pvParameters);
void activateMenuItem(uint8_t idx);
void lcdShowMenu();
void lcdShowInputIP();
void lcdShowInfo();
bool isValidIP(const String& ip);
void applyNewIP(const String& newIP);

// =====================================================
// HARDWARE
// =====================================================
#define MPU_ADDR 0x68
#define AK_ADDR  0x0C

TwoWire I2CBME = TwoWire(0);
TwoWire I2CMPU = TwoWire(1);

HardwareSerial SerialHUSKY(2);

Adafruit_BME280 bme;
Madgwick filter;
HUSKYLENS huskyLens;

bool bme_ok   = false;
bool mpu_ok   = false;
bool husky_ok = false;

float gx_offset = 0, gy_offset = 0, gz_offset = 0;

float bme_t = 0.0f, bme_h = 0.0f, bme_p = 0.0f;
unsigned long lastBMEread = 0;

volatile float imu_roll  = 0.0f;
volatile float imu_pitch = 0.0f;
volatile float imu_yaw   = 0.0f;
volatile float mag_x_raw = 0.0f;
volatile float mag_y_raw = 0.0f;
volatile float mag_z_raw = 0.0f;

SemaphoreHandle_t imuMutex;

#define HUSKY_MAX_OBJS 10

struct HuskyObj {
    int16_t id;
    int16_t x, y, w, h;
    bool    learned;
};

volatile HuskyObj husky_objects[HUSKY_MAX_OBJS];
volatile uint8_t  husky_obj_count = 0;
volatile int      husky_current_algo = HUSKY_ALGO;
SemaphoreHandle_t huskyMutex;

WiFiClient    tcpClient;
bool          serverConnected = false;
unsigned long lastPushMs      = 0;
unsigned long lastReconnMs    = 0;
unsigned long lastHuskyMs     = 0;

// =====================================================
// MPU9250 — (sense canvis)
// =====================================================
bool mpuInit() {
  I2CMPU.beginTransmission(MPU_ADDR);
  I2CMPU.write(0x6B);
  I2CMPU.write(0x00);
  if (I2CMPU.endTransmission() != 0) return false;
  delay(100);
  I2CMPU.beginTransmission(MPU_ADDR); I2CMPU.write(0x1B); I2CMPU.write(0x00); I2CMPU.endTransmission();
  I2CMPU.beginTransmission(MPU_ADDR); I2CMPU.write(0x1C); I2CMPU.write(0x00); I2CMPU.endTransmission();
  I2CMPU.beginTransmission(MPU_ADDR); I2CMPU.write(0x37); I2CMPU.write(0x02); I2CMPU.endTransmission();
  delay(100);
  I2CMPU.beginTransmission(AK_ADDR);  I2CMPU.write(0x0A); I2CMPU.write(0x16);
  if (I2CMPU.endTransmission() != 0) Serial.println("  WARN: AK8963 no respon");
  delay(100);
  Serial.println("  Calibrant giroscopi...");
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 100; i++) {
    I2CMPU.beginTransmission(MPU_ADDR); I2CMPU.write(0x43); I2CMPU.endTransmission(false);
    I2CMPU.requestFrom(MPU_ADDR, 6);
    sx += (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f;
    sy += (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f;
    sz += (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f;
    delay(5);
  }
  gx_offset = sx / 100.0f; gy_offset = sy / 100.0f; gz_offset = sz / 100.0f;
  Serial.printf("  Offsets gyro: %.3f, %.3f, %.3f\n", gx_offset, gy_offset, gz_offset);
  return true;
}

void mpuRead(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz,
             float &mx, float &my, float &mz) {
  I2CMPU.beginTransmission(MPU_ADDR); I2CMPU.write(0x3B); I2CMPU.endTransmission(false);
  I2CMPU.requestFrom(MPU_ADDR, 14);
  ax = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 16384.0f;
  ay = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 16384.0f;
  az = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 16384.0f;
  I2CMPU.read(); I2CMPU.read();
  gx = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f - gx_offset;
  gy = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f - gy_offset;
  gz = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f - gz_offset;
  I2CMPU.beginTransmission(AK_ADDR); I2CMPU.write(0x03); I2CMPU.endTransmission(false);
  I2CMPU.requestFrom(AK_ADDR, 7);
  if (I2CMPU.available() >= 7) {
    mx = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) * 0.15f;
    my = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) * 0.15f;
    mz = (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) * 0.15f;
    I2CMPU.read();
  } else { mx = my = mz = 0.0f; }
}

// =====================================================
// WIFI / TCP — (sense canvis)
// =====================================================
void iniciarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connectant al WiFi");
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < WIFI_CONNECT_TRIES) { delay(500); Serial.printf(". status=%d\n", WiFi.status()); tries++; }
  if (WiFi.status() != WL_CONNECTED) {
    // IMPORTANT: ja NO fem ESP.restart() aquí. Sense WiFi, el teclat 4x4,
    // la LCD i la UART cap a la CAM (taskUI / taskCamSerial) ja estan
    // funcionant (es creen abans de cridar aquesta funció). El loop()
    // principal seguirà intentant connectar-se en segon pla.
    Serial.println("\nWiFi no disponible de moment. Continuo sense WiFi (teclat/LCD/UART CAM ja actius).");
    return;
  }
  Serial.println("\nWiFi OK!"); Serial.print("  IP: "); Serial.println(WiFi.localIP());
}

bool connectarServidor() {
  char srv[20];
  if (xSemaphoreTake(ipMutex, portMAX_DELAY)) { memcpy(srv, SERVER_IP, sizeof(srv)); xSemaphoreGive(ipMutex); }
  Serial.printf("Connectant a %s:%d...\n", srv, SERVER_PORT);
  if (!tcpClient.connect(srv, SERVER_PORT)) { Serial.println("  → Fallo connexió"); return false; }
  tcpClient.setNoDelay(true);
  tcpClient.println("HELLO:sensor");
  Serial.println("  → Connectat!");
  return true;
}

// =====================================================
// TASK IMU — (sense canvis)
// =====================================================
void taskIMU(void* pvParameters) {
  filter.begin(100);
  float yaw_offset = 0.0f;
  bool  yaw_calibrated = false;
  int   warmup = 300;
  for (;;) {
    if (mpu_ok) {
      float ax, ay, az, gx, gy, gz, mx, my, mz;
      mpuRead(ax, ay, az, gx, gy, gz, mx, my, mz);
      filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
      if (!yaw_calibrated) { if (warmup > 0) warmup--; else { yaw_offset = filter.getYaw(); yaw_calibrated = true; } }
      float raw_yaw = filter.getYaw() - yaw_offset;
      if (raw_yaw >  180.0f) raw_yaw -= 360.0f;
      if (raw_yaw < -180.0f) raw_yaw += 360.0f;
      if (xSemaphoreTake(imuMutex, portMAX_DELAY)) {
        imu_roll  = 0.9f * imu_roll  + 0.1f * filter.getRoll();
        imu_pitch = 0.9f * imu_pitch + 0.1f * filter.getPitch();
        imu_yaw   = 0.9f * imu_yaw   + 0.1f * raw_yaw;
        mag_x_raw = mx; mag_y_raw = my; mag_z_raw = mz;
        xSemaphoreGive(imuMutex);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// =====================================================
// TASK HUSKYLENS — (sense canvis)
// =====================================================
void taskHuskyLens(void* pvParameters) {
  for (;;) {
    if (husky_ok) {
      bool ok = huskyLens.request();
      if (ok && huskyLens.available()) {
        if (xSemaphoreTake(huskyMutex, pdMS_TO_TICKS(50))) {
          uint8_t count = 0;
          while (huskyLens.available() && count < HUSKY_MAX_OBJS) {
              HUSKYLENSResult result = huskyLens.read();
              husky_objects[count].id      = result.ID;
              husky_objects[count].x       = result.xCenter;
              husky_objects[count].y       = result.yCenter;
              husky_objects[count].w       = result.width;
              husky_objects[count].h       = result.height;
              husky_objects[count].learned = (result.ID > 0);
              count++;
          }
          husky_obj_count = count;
          xSemaphoreGive(huskyMutex);
        }
      } else {
        if (xSemaphoreTake(huskyMutex, pdMS_TO_TICKS(10))) { husky_obj_count = 0; xSemaphoreGive(huskyMutex); }
      }
    }
    vTaskDelay(HUSKY_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

// =====================================================
// TASK CAM SERIAL
// Rep:  "CAM_STAT:<rssi>,<ip>,<temp>"  (sense canvis)
// ── NOU: Envia "SET_IP:<ip>\n" cap a la CAM cada 5 s
//
// Futura extensió (LCD + teclat 4x4):
//   taskUI() modifica cam_target_ip[] via un formulari
//   i crida  xSemaphoreGive(ipChangedSem)  per forçar
//   un enviament immediat sense esperar els 5 s.
//   Esquema de pins suggerit:
//     LCD I2C  → SDA=GPIO21, SCL=GPIO22  (bus lliure)
//     Teclat   → 8 GPIOs (ex: 13,12,14,27,26,25,33,32)
// =====================================================
void taskCamSerial(void* pvParameters) {
  String line = "";
  line.reserve(64);
  uint32_t lastIpSendMs = 0;
  uint32_t lastHeartbeat = 0;   // ── DEBUG

  for (;;) {
    uint32_t nowDbg = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (nowDbg - lastHeartbeat >= 1000) {          // ── DEBUG: cada 1s, sin condiciones
      lastHeartbeat = nowDbg;
      Serial.printf("[CAM_TASK] viva | bytes pendientes: %d\n", SerialCAM.available());
    }

    while (SerialCAM.available()) {
      char c = (char)SerialCAM.read();
      Serial.write(c);
      if (c == '\n') {
        line.trim();
        if (line.startsWith("CAM_STAT:")) {
          String payload = line.substring(9);
          int c1 = payload.indexOf(',');
          int c2 = payload.indexOf(',', c1 + 1);
          if (c1 > 0 && c2 > c1) {
            int    r       = payload.substring(0, c1).toInt();
            String ip_str  = payload.substring(c1 + 1, c2);
            int    c3      = payload.indexOf(',', c2 + 1);
            float  t       = payload.substring(c2 + 1, c3 > c2 ? c3 : payload.length()).toFloat();
            String srv_str = (c3 > c2) ? payload.substring(c3 + 1) : String("");
            srv_str.trim();
            if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(10))) {
              cam_rssi = r; 
              cam_temp = t;
              ip_str.toCharArray(cam_ip, sizeof(cam_ip));
              if (srv_str.length() >= 7)
                srv_str.toCharArray(cam_server_ip, sizeof(cam_server_ip));
              xSemaphoreGive(camMutex);
  }
}
        }
        line = "";
      } else if (c != '\r') {
        if (line.length() < 63) line += c;
      }
    }

    uint32_t now = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - lastIpSendMs >= CAM_IP_SEND_INTERVAL_MS || sendIPNow) {
      lastIpSendMs = now;
      sendIPNow = false;
      char ipToSend[20];
      if (xSemaphoreTake(ipMutex, pdMS_TO_TICKS(10))) {
        memcpy(ipToSend, cam_target_ip, sizeof(ipToSend));
        xSemaphoreGive(ipMutex);
      }
      SerialCAM.print("SET_IP:");
      SerialCAM.println(ipToSend);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// =====================================================
// ENVIAR DADES / HUSKY / RECALIBRAR — (sense canvis)
// =====================================================
void enviarDades() {
  float roll, pitch, yaw, mx, my, mz;
  if (xSemaphoreTake(imuMutex, portMAX_DELAY)) {
    roll = imu_roll; pitch = imu_pitch; yaw = imu_yaw;
    mx = mag_x_raw; my = mag_y_raw; mz = mag_z_raw;
    xSemaphoreGive(imuMutex);
  }
  int c_rssi; float c_temp; char c_ip[20];
  if (xSemaphoreTake(camMutex, portMAX_DELAY)) {
    c_rssi = cam_rssi; c_temp = cam_temp; memcpy(c_ip, cam_ip, sizeof(c_ip));
    xSemaphoreGive(camMutex);
  }
  int s_rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100;
  float s_temp = temperatureRead();
  String s_ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
  char buf[240];
  snprintf(buf, sizeof(buf),
           "DATA:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%s,%.1f,%d,%s,%.1f",
           bme_t, bme_h, bme_p, roll, pitch, yaw, mx, my, mz,
           c_rssi, c_ip, c_temp, s_rssi, s_ip.c_str(), s_temp);
  tcpClient.println(buf);
}

void enviarHusky() {
  if (!husky_ok) return;
  uint8_t count = 0; HuskyObj objs[HUSKY_MAX_OBJS]; int algo = HUSKY_ALGO;
  if (xSemaphoreTake(huskyMutex, pdMS_TO_TICKS(20))) {
    count = husky_obj_count; algo = husky_current_algo;
    for (uint8_t i = 0; i < count; i++)
      objs[i] = { husky_objects[i].id, husky_objects[i].x, husky_objects[i].y, husky_objects[i].w, husky_objects[i].h, husky_objects[i].learned };
    xSemaphoreGive(huskyMutex);
  }
  char buf[400]; int pos = 0;
  pos += snprintf(buf + pos, sizeof(buf) - pos, "HUSKY:%s,%d", huskyAlgoName(algo), count);
  for (uint8_t i = 0; i < count && pos < (int)sizeof(buf) - 40; i++)
    pos += snprintf(buf + pos, sizeof(buf) - pos, ",%d,%d,%d,%d,%d,%d",
                    (int)objs[i].id, (int)objs[i].x, (int)objs[i].y,
                    (int)objs[i].w, (int)objs[i].h, objs[i].learned ? 1 : 0);
  tcpClient.println(buf);
}

void recalibrarIMU() {
  if (!mpu_ok) return;
  Serial.println("Recalibrant giroscopi...");
  if (xSemaphoreTake(imuMutex, portMAX_DELAY)) { imu_roll = imu_pitch = imu_yaw = 0.0f; xSemaphoreGive(imuMutex); }
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 100; i++) {
    I2CMPU.beginTransmission(MPU_ADDR); I2CMPU.write(0x43); I2CMPU.endTransmission(false);
    I2CMPU.requestFrom(MPU_ADDR, 6);
    sx += (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f;
    sy += (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f;
    sz += (int16_t)((I2CMPU.read() << 8) | I2CMPU.read()) / 131.0f;
    delay(5);
  }
  gx_offset = sx / 100.0f; gy_offset = sy / 100.0f; gz_offset = sz / 100.0f;
  Serial.println("Recalibració completada.");
}

// =====================================================
// VALIDACIÓ I APLICACIÓ DE LA NOVA IP
// =====================================================
bool isValidIP(const String& ip) {
  int len = ip.length();
  if (len < 7 || len > 15) return false; // "0.0.0.0" .. "255.255.255.255"
  int dots = 0;
  int segStart = 0;
  for (int i = 0; i <= len; i++) {
    if (i == len || ip[i] == '.') {
      int segLen = i - segStart;
      if (segLen < 1 || segLen > 3) return false;
      int val = ip.substring(segStart, i).toInt();
      if (val < 0 || val > 255) return false;
      if (i < len) dots++;
      segStart = i + 1;
    } else if (!isDigit(ip[i])) {
      return false;
    }
  }
  return dots == 3;
}

void applyNewIP(const String& newIP) {
  if (xSemaphoreTake(ipMutex, portMAX_DELAY)) {
    newIP.toCharArray(SERVER_IP, sizeof(SERVER_IP));
    newIP.toCharArray(cam_target_ip, sizeof(cam_target_ip));
    xSemaphoreGive(ipMutex);
  }
  sendIPNow = true; // que taskCamSerial l'enviï a la CAM al següent cicle, sense esperar 5s
  if (serverConnected) { tcpClient.stop(); serverConnected = false; } // força reconnexió amb la IP nova
  Serial.printf("Nova IP de servidor aplicada: %s\n", newIP.c_str());
}

// =====================================================
// PANTALLES LCD
// =====================================================
void lcdShowMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">");
  lcd.print(MENU_ITEMS[menuIndex]);

  // Antes esto hacía (menuIndex + 1) % MENU_COUNT, así que al llegar a
  // la última opción la línea de abajo volvía a mostrar la opción 1
  // (le daba la vuelta a la lista). Ahora solo mostramos una "siguiente
  // opción" si realmente existe una opción después de la actual.
  if (menuIndex + 1 < MENU_COUNT) {
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.print(MENU_ITEMS[menuIndex + 1]);
  }
}

void lcdShowInputIP() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Introdueix IP:");
  lcd.setCursor(0, 1);
  lcd.print(ipInputBuffer);
}

void lcdShowInfo() {
  lcd.clear();
  char srvIP[20];
  if (xSemaphoreTake(ipMutex, portMAX_DELAY)) { memcpy(srvIP, SERVER_IP, sizeof(srvIP)); xSemaphoreGive(ipMutex); }
  String ipLocal = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(ipLocal);
  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(srvIP);
}

void lcdShowCamInfo() {
  char ip[20], srv[20];
  int  rssi;
  float temp;
  if (xSemaphoreTake(camMutex, portMAX_DELAY)) {
    memcpy(ip,  cam_ip,        sizeof(ip));
    memcpy(srv, cam_server_ip, sizeof(srv));
    rssi = cam_rssi;
    temp = cam_temp;
    xSemaphoreGive(camMutex);
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(ip);       // arriba: IP física de la CAM
  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(srv);      // abajo: IP que busca la CAM
}

// =====================================================
// TASCA UI — Teclat 4x4 + LCD I2C
// =====================================================
void activateMenuItem(uint8_t idx) {
  switch (idx) {
    case 0: // Canviar IP
      ipInputBuffer = "";
      uiState = UI_INPUT_IP;
      lcdShowInputIP();
      break;
    case 1: // Reset IMU
      recalibrarIMU();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("IMU recalibrada");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      lcdShowMenu();
      break;
    case 2: // Veure estat
      uiState = UI_INFO;
      lcdShowInfo();
      break;
    case 3: // IP Camara
      uiState = UI_CAM_INFO;
      lcdShowCamInfo();
      break;
  }
}

void taskUI(void* pvParameters) {
  lcd.clear();
  lcdShowMenu();

  for (;;) {
    char key = keypad.getKey();
    if (key) {
      // ── DEBUG TEMPORAL ────────────────────────────────────────
      // Imprime por el Monitor Serie (115200 baudios) CADA tecla que
      // el código detecta, junto con el estado actual de la pantalla.
      // Pulsa cada botón físico uno a uno y compara: si lo que sale
      // aquí no coincide con el botón que has apretado (p.ej. pulsas
      // el "2" físico y aquí sale 'A' o '5'), el problema es de
      // CABLEADO (las filas/columnas no están en los pines que el
      // código espera, keypadRowPins/keypadColPins), no de lógica.
      // Cuando ya esté todo verificado, puedes borrar esta línea.
      Serial.printf("[TECLAT] tecla='%c' (ASCII %d) | uiState=%d | menuIndex=%d | buffer=\"%s\"\n",
                    key, (int)key, (int)uiState, menuIndex, ipInputBuffer.c_str());
      // ─────────────────────────────────────────────────────────

      switch (uiState) {

        case UI_MENU:
          if (key == KEY_MENU_UP) {                 // sube
            menuIndex = (menuIndex - 1 + MENU_COUNT) % MENU_COUNT;
            lcdShowMenu();
          } else if (key == KEY_MENU_DOWN) {        // baja
            menuIndex = (menuIndex + 1) % MENU_COUNT;
            lcdShowMenu();
          } else if (key >= '1' && key <= ('0' + MENU_COUNT)) { // accés directe 1/2/3
            menuIndex = key - '1';
            activateMenuItem(menuIndex);
          } else if (key == KEY_CONFIRM) {          // confirma l'opció ressaltada
            activateMenuItem(menuIndex);
          }
          break;

        case UI_INPUT_IP:
          if (key >= '0' && key <= '9') {
            if (ipInputBuffer.length() < 15) {
              ipInputBuffer += key;
              lcdShowInputIP();
            }
          } else if (key == KEY_IP_DOT) {           // 'D' fa de punt (.) en aquest mode
            if (ipInputBuffer.length() > 0 &&
                ipInputBuffer.length() < 15 &&
                ipInputBuffer.charAt(ipInputBuffer.length() - 1) != '.') {
              ipInputBuffer += '.';
              lcdShowInputIP();
            }
          } else if (key == KEY_BACKSPACE) {        // esborra últim caràcter / torna al menú
            if (ipInputBuffer.length() > 0) {
              ipInputBuffer.remove(ipInputBuffer.length() - 1);
              lcdShowInputIP();
            } else {
              uiState = UI_MENU;
              lcdShowMenu();
            }
          } else if (key == KEY_CONFIRM) {          // confirma i envia
            if (isValidIP(ipInputBuffer)) {
              applyNewIP(ipInputBuffer);
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("IP actualitzada!");
              lcd.setCursor(0, 1);
              lcd.print(ipInputBuffer);
              vTaskDelay(1500 / portTICK_PERIOD_MS);
              uiState = UI_MENU;
              lcdShowMenu();
            } else {
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("IP no valida!");
              vTaskDelay(1200 / portTICK_PERIOD_MS);
              lcdShowInputIP();
            }
          }
          break;

        case UI_INFO:
          if (key == KEY_BACKSPACE) {               // torna al menú
            uiState = UI_MENU;
            lcdShowMenu();
          }
          break;
        case UI_CAM_INFO:
          if (key == KEY_BACKSPACE) {               // * torna al menú
            uiState = UI_MENU;
            lcdShowMenu();
          } else if (key == KEY_CONFIRM) {          // # refresca la pantalla
            lcdShowCamInfo();
          }
          break;
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void processarComanda(const String& cmd) {
  Serial.println("CMD: " + cmd);

  // ── Comandos del coche/servo (Arduino) ────────────────────────
  // Llegan tal cual desde test.py (un solo carácter), se reenvían
  // directamente a la UART software hacia el Arduino.
  if (cmd.length() == 1 && strchr("FBLRSQE", cmd.charAt(0))) {
  char c = cmd.charAt(0);
  if (c == 'S') {
    for (int i = 0; i < 3; i++) { SerialARDUINO.print(c); delay(3); }
  } else {
    SerialARDUINO.print(c);
  }
  lastCarCmd    = c;
  lastCarSendMs = millis();
  carMoviment   = (c == 'F' || c == 'B' || c == 'L' || c == 'R');
  return;
}

  if (cmd == "REQUEST_BME") {
    char buf[64]; snprintf(buf, sizeof(buf), "BME:%.2f,%.2f,%.2f", bme_t, bme_h, bme_p);
    tcpClient.println(buf); tcpClient.println("ACK:BME_SENT");
  } else if (cmd == "REQUEST_IMU") {
    float roll, pitch, yaw, mx, my, mz;
    if (xSemaphoreTake(imuMutex, portMAX_DELAY)) { roll=imu_roll; pitch=imu_pitch; yaw=imu_yaw; mx=mag_x_raw; my=mag_y_raw; mz=mag_z_raw; xSemaphoreGive(imuMutex); }
    char buf[128]; snprintf(buf, sizeof(buf), "IMU:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", roll, pitch, yaw, mx, my, mz);
    tcpClient.println(buf); tcpClient.println("ACK:IMU_SENT");
  } else if (cmd == "RESET_IMU") {
    recalibrarIMU(); tcpClient.println("ACK:IMU_RESET");
  } else if (cmd == "REQUEST_HUSKY") {
    enviarHusky(); tcpClient.println("ACK:HUSKY_SENT");
  } else if (cmd.startsWith("HUSKY_ALGO:")) {
    String algoStr = cmd.substring(11); int newAlgo = -1;
    if      (algoStr == "FACE_RECOGNITION")      newAlgo = ALGORITHM_FACE_RECOGNITION;
    else if (algoStr == "OBJECT_TRACKING")        newAlgo = ALGORITHM_OBJECT_TRACKING;
    else if (algoStr == "OBJECT_RECOGNITION")     newAlgo = ALGORITHM_OBJECT_RECOGNITION;
    else if (algoStr == "LINE_TRACKING")          newAlgo = ALGORITHM_LINE_TRACKING;
    else if (algoStr == "COLOR_RECOGNITION")      newAlgo = ALGORITHM_COLOR_RECOGNITION;
    else if (algoStr == "TAG_RECOGNITION")        newAlgo = ALGORITHM_TAG_RECOGNITION;
    else if (algoStr == "OBJECT_CLASSIFICATION")  newAlgo = ALGORITHM_OBJECT_CLASSIFICATION;
    if (newAlgo >= 0 && husky_ok) {
      if (huskyLens.writeAlgorithm((protocolAlgorithm)newAlgo)) {
        if (xSemaphoreTake(huskyMutex, portMAX_DELAY)) { husky_current_algo = newAlgo; xSemaphoreGive(huskyMutex); }
        tcpClient.println("ACK:HUSKY_ALGO_OK");
        Serial.printf("HuskyLens → algoritme canviat a %s\n", algoStr.c_str());
      } else tcpClient.println("ERR:HUSKY_ALGO_FAIL");
    } else tcpClient.println("ERR:HUSKY_ALGO_UNKNOWN");
  } else if (cmd == "PING") {
    tcpClient.println("PONG");
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== ESP32 GY-91 + HuskyLens — Control Remot ===");

  // 1. Inicializamos el Wire nativo en los pines 21 y 22 (Aquí conectarás LCD y BME280 en paralelo)
  Wire.begin(21, 22, 100000); 

  // 2. Inicializamos el segundo bus de hardware para la IMU en los pines 16 y 17
  I2CMPU.begin(16, 17, 400000); 
  delay(300);

  // La LCD funcionará automáticamente en los pines 21 y 22 bajo el objeto Wire nativo
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");

  strncpy(cam_target_ip, SERVER_IP, sizeof(cam_target_ip) - 1);
  cam_target_ip[sizeof(cam_target_ip) - 1] = '\0';

  Serial.println("Iniciant BME280...");
  // 3. CAMBIO CLAVE: Cambiamos &I2CBME por &Wire
  bme_ok = bme.begin(0x76, &Wire); 
  Serial.println(bme_ok ? "  BME280 OK" : "  ERROR BME280");

  Serial.println("Iniciant MPU9250...");
  mpu_ok = mpuInit();
  Serial.println(mpu_ok ? "  MPU9250 OK." : "  ERROR MPU9250.");

  Serial.println("Iniciant HuskyLens...");
  delay(2000);
  SerialHUSKY.begin(9600, SERIAL_8N1, HUSKY_RX, HUSKY_TX);
  huskyLens.begin(SerialHUSKY);
  for (int intent = 1; intent <= 5; intent++) {
    Serial.printf("  Intent %d/5...\n", intent);
    if (huskyLens.request()) {
      husky_ok = true;
      huskyLens.writeAlgorithm(HUSKY_ALGO);
      Serial.printf("  HuskyLens OK — algoritme: %s\n", huskyAlgoName(HUSKY_ALGO));
      break;
    }
    delay(1000);
  }
  if (!husky_ok) {
    Serial.println("  ERROR HuskyLens.");
  }

  Serial.println("Iniciant UART CAM bidireccional...");
  SerialCAM.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  Serial.println("  UART CAM OK (RX+TX)");

  Serial.println("Iniciant UART Arduino (cotxe)...");
  SerialARDUINO.begin(ARDUINO_BAUD);
  Serial.println("  UART Arduino OK");

  imuMutex   = xSemaphoreCreateMutex();
  huskyMutex = xSemaphoreCreateMutex();
  camMutex   = xSemaphoreCreateMutex();
  ipMutex    = xSemaphoreCreateMutex();   // NOU

  xTaskCreatePinnedToCore(taskIMU,       "imu",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskHuskyLens, "husky", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskCamSerial, "cam",   2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskUI,        "ui",    4096, NULL, 2, NULL, 0); // NOU

  iniciarWiFi(); // si falla, ja NO reinicia: la resta del sistema continua actiu

  Serial.println("Sistema llest.");
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================
void loop() {
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdut! Reconnectant...");
    // Parada immediata del cotxe: no esperem als 700ms del watchdog
    // de l'Arduino si ja sabem que hem perdut la connexió.
    if (carMoviment) { SerialARDUINO.print('S'); carMoviment = false; }
    WiFi.reconnect(); delay(3000); return;
  }
  if (!serverConnected || !tcpClient.connected()) {
    if (serverConnected) {
      Serial.println("Connexió TCP perduda.");
      tcpClient.stop(); serverConnected = false;
      if (carMoviment) { SerialARDUINO.print('S'); carMoviment = false; } // parada immediata
    }
    if (now - lastReconnMs >= RECONN_INTERVAL_MS) { lastReconnMs = now; serverConnected = connectarServidor(); }
    delay(10); return;
  }
  while (tcpClient.available()) { String line = tcpClient.readStringUntil('\n'); line.trim(); if (line.length() > 0) processarComanda(line); }

  if (carMoviment && (now - lastCarSendMs >= CAR_RESEND_INTERVAL_MS)) {
    lastCarSendMs = now;
    SerialARDUINO.print(lastCarCmd);
  }

  if (bme_ok && (now - lastBMEread >= 2000)) { lastBMEread = now; bme_t = bme.readTemperature(); bme_h = bme.readHumidity(); bme_p = bme.readPressure() / 100.0F; }
  if (now - lastPushMs >= PUSH_INTERVAL_MS) { lastPushMs = now; enviarDades(); }
  if (husky_ok && (now - lastHuskyMs >= HUSKY_INTERVAL_MS)) { lastHuskyMs = now; enviarHusky(); }
  delay(5);
}