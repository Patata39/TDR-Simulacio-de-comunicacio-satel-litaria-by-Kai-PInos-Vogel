#include <WiFi.h>
#include "esp_camera.h"

// ──  SERVER_IP ara és un buffer modificable per UART ────────
// En comptes de  const char* SERVER_IP = "192.168.1.42";
char serverIp[20] = "192.168.1.42";   // valor per defecte
// ──────────────────────────────────────────────────────────────
const char* WIFI_SSID      = "Kai";
const char* WIFI_PASSWORD  = "12345678";
const uint16_t SERVER_PORT = 5000;

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiClient camClient;
bool serverConnected   = false;
uint32_t lastConnectMs = 0;

HardwareSerial camSerial(1);
uint32_t lastUartSend = 0;

// ── buffer de línia entrant per UART ──────────────────────
String uartLine = "";
// ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  camSerial.begin(115200, SERIAL_8N1, 13, 14);
  delay(500);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  delay(500);
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init FAILED → restart");
    delay(1000);
    ESP.restart();
  }
  Serial.println("Càmera OK");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connectant WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.println("\nWiFi OK: " + WiFi.localIP().toString());

  uartLine.reserve(64);   
}

void loop() {
  uint32_t nowMs = millis();
  if (nowMs - lastUartSend >= 1000) {
    lastUartSend = nowMs;
    int    rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100;
    String ip   = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
    float  temp = temperatureRead();
    camSerial.printf("CAM_STAT:%d,%s,%.1f,%s\n", rssi, ip.c_str(), temp, serverIp);
  }
  while (camSerial.available()) {
    char c = camSerial.read();
    static String inputLine = "";

    if (c == '\n') { 
      inputLine.trim();
      if (inputLine.startsWith("SET_IP:")) {
        String nuevaIp = inputLine.substring(7);
        nuevaIp.toCharArray(serverIp, sizeof(serverIp));
        Serial.printf("Nueva IP configurada por UART: %s\n", serverIp);
        
        if (serverConnected) {
          camClient.stop();
          serverConnected = false;
        }
      }
      inputLine = ""; 
    } else if (c != '\r') {
      inputLine += c;
    }
  }

  if (!serverConnected) {
    uint32_t now = millis();
    if (now - lastConnectMs >= 3000) { 
      lastConnectMs = now;
      Serial.printf("Intentando conectar a %s:%d...\n", serverIp, SERVER_PORT);
      
      if (camClient.connect(serverIp, SERVER_PORT)) {
        camClient.setNoDelay(true);
        camClient.println("HELLO:camera_direct");
        serverConnected = true;
        Serial.printf("¡Servidor conectado exitosamente!: %s\n", serverIp);
      } else {
        Serial.printf("Error de conexión a %s. Reintentando...\n", serverIp);
      }
    }
    return;
  }

  if (!camClient.connected()) { 
    serverConnected = false; 
    camClient.stop(); 
    Serial.println("Conexión perdida con el servidor."); 
    return; 
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { 
    delay(5); 
    return; 
  }

  uint8_t hdr[8] = {
    'M','C','A','M',
    (uint8_t)(fb->len >> 24), (uint8_t)(fb->len >> 16),
    (uint8_t)(fb->len >>  8), (uint8_t)(fb->len      )
  };

  bool ok = (camClient.write(hdr, 8) == 8);
  if (ok) {
    size_t sent = 0;
    while (sent < fb->len) {
      int n = camClient.write(fb->buf + sent, fb->len - sent);
      if (n <= 0) { ok = false; break; }
      sent += n;
    }
  }
  esp_camera_fb_return(fb);

  if (!ok) {
    Serial.println("Error al enviar frame, cerrando socket.");
    camClient.stop();
    serverConnected = false;
  }
}
