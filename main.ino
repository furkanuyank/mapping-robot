#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WebSocketsServer.h>
#include "StepMotorController.h"
#include "SimpleKalmanFilter.h"

#define DIR_PIN1 12
#define STEP_PIN1 14
#define ENABLE_PIN1 27

#define DIR_PIN2 25
#define STEP_PIN2 26
#define ENABLE_PIN2 33

#define TRIG_PIN 23
#define ECHO_PIN 19

#define STEP_DELAY 1000 // Mikrosaniye cinsinden adım gecikmesi (hızı artırmak için düşürüldü)

const char* ssid = "stepmotorarac";
const char* password = "12345678";

WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

MPU6050 mpu;
StepMotorController motorController(DIR_PIN1, STEP_PIN1, ENABLE_PIN1, DIR_PIN2, STEP_PIN2, ENABLE_PIN2, STEP_DELAY);

SimpleKalmanFilter rollKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter pitchKalmanFilter(2, 2, 0.01);
SimpleKalmanFilter yawKalmanFilter(2, 2, 0.01);

// Sensör kalibrasyonu için ofset değerleri
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
unsigned long previousTime = 0;
float yaw = 0; // Yaw değeri
int oldYawInt=0;
bool isFirst=true;

// Başlangıç (sıfırlama) değerleri
float initialRoll = 0, initialPitch = 0, initialYaw = 0;
bool initialized = false;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  motorController.begin();

  Wire.begin(21, 22); // SDA -> GPIO21, SCL -> GPIO22
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Sensör kalibrasyonu
  calibrateMPU6050();

  // HC-SR04 sensörü için pin modları
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  webSocket.loop();
  motorController.update(); // Motor hareketini güncelle

   unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0; // Zaman farkı (saniye cinsinden)

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Ofsetleri çıkararak gyro verilerini düzelt
  gx -= gyroXOffset;
  gy -= gyroYOffset;
  gz -= gyroZOffset;

  // Gyro Z verisini dereceye çevir ve entegre ederek yaw değerini hesapla
  float gyroYawRate = gz / 131.0; // Gyro z eksenindeki açı hızı (derece/saniye)
  yaw += gyroYawRate * dt; // Yaw değerini güncelle

  previousTime = currentTime;

  // Mesafe ölçümü (HC-SR04)
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1; // Mesafeyi cm cinsinden hesapla
  
  int yawInt = (int)yaw;
  if(oldYawInt!=yawInt || isFirst==true){
    String rpy ="{\"y\": "+ String(yawInt) + ",\"d\": " + String(distance) +"}";
    webSocket.broadcastTXT(rpy);
    oldYawInt=yawInt;
    isFirst=false;
  }
  

  WiFiClient client = server.available();

  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    if (request.indexOf("/ileri") != -1) {
      motorController.ileri();
    }
    if (request.indexOf("/geri") != -1) {
      motorController.geri();
    }
    if (request.indexOf("/sag") != -1) {
      motorController.sag();
    }
    if (request.indexOf("/sol") != -1) {
      motorController.sol();
    }
    if (request.indexOf("/tara") != -1) {
      motorController.sol();
    }
    if (request.indexOf("/dur") != -1) {
      motorController.dur();
    }

    // HTML sayfasını gönder
    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    client.print("<!DOCTYPE HTML>\n<html>\n");
    client.print("<head><title>Motor Kontrol</title></head>\n");

    client.print("<script>\n");

    client.print("var socket = new WebSocket('ws://' + window.location.hostname + ':81/');\n");
    client.print("socket.onmessage = function(event) { document.getElementById('rpy').innerHTML = event.data; }\n");
    client.print("function sendCommand(command) { if(command=='tara'){for (let i = 1; i <= 10; i++) {socket.send(command);}} else {socket.send(command);}}\n");

    client.print("</script>\n");

    client.print("</head>\n");
    client.print("<body>\n");
    client.print("<h1>Motor Kontrol</h1>\n");
    client.print("<p id='rpy'>{\"y\": 0,\"d\": 0}</p>\n");
    client.print("<button onclick=\"sendCommand('ileri')\">Ileri</button>\n");
    client.print("<button onclick=\"sendCommand('geri')\">Geri</button>\n");
    client.print("<button onclick=\"sendCommand('sag')\">Sag</button>\n");
    client.print("<button onclick=\"sendCommand('sol')\">Sol</button>\n");
    client.print("<button onclick=\"sendCommand('tara')\">Tara</button>\n");
    client.print("</body>\n</html>\n");
    
    delay(1);
    client.stop();
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        Serial.println(ip.toString());
      }
      break;
    case WStype_TEXT:
      String message = String((char*)payload);
      Serial.printf("Received command: %s\n", message.c_str()); // Komutları seri monitörde göster

      if (message == "ileri") {
        motorController.ileri();
      }
      if (message == "geri") {
        motorController.geri();
      }
      if (message == "sag") {
        motorController.sag();
      }
      if (message == "sol") {
        motorController.sol();
      }
      if (message == "tara") {
        motorController.tara();
      }
      if (message == "dur") {
        motorController.dur();
      }
      break;
  }
}

void calibrateMPU6050() {
  const int numReadings = 1000;
  long gxAccum = 0, gyAccum = 0, gzAccum = 0;
  long axAccum = 0, ayAccum = 0, azAccum = 0;

  for (int i = 0; i < numReadings; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gxAccum += gx;
    gyAccum += gy;
    gzAccum += gz;
    axAccum += ax;
    ayAccum += ay;
    azAccum += az;
    delay(3); // Her ölçüm arasında kısa bir bekleme süresi
  }

  gyroXOffset = gxAccum / numReadings;
  gyroYOffset = gyAccum / numReadings;
  gyroZOffset = gzAccum / numReadings;


  Serial.print("Gyro X Offset: ");
  Serial.println(gyroXOffset);
  Serial.print("Gyro Y Offset: ");
  Serial.println(gyroYOffset);
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZOffset);
}
