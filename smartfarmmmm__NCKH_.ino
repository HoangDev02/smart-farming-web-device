#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <DHT.h>
#include <type_traits>
#include <algorithm>
#include <Adafruit_NeoPixel.h>
#define DHTTYPE DHT11
#include <LiquidCrystal_I2C.h>


#define PIN_WS2812B 17  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 30   // The number of LEDs (pixels) on WS2812B LED strip
Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);


uint8_t red = 255;
uint8_t green = 102;
uint8_t blue = 0;
const char* ssid = "Redmi Note 12";
const char* password = "11092002";

const char* mqtt_server = "f63a3874d1364c15ba9d13699c92dc63.s1.eu.hivemq.cloud";
const char* mqtt_username = "esp8266Den";
const char* mqtt_password = "esp8266Den";
const char* mqtt_topic_pump = "PUMP";
const char* mqtt_topic_modePum = "MODEPUMP";
const char* mqtt_topic_Temp = "Temp";
const char* mqtt_topic_Humi = "Humi";
const char* mqtt_topic_RGB = "RGB";
const char* mqtt_topic_light = "Light";
const char* mqtt_topic_tempFan = "TempFan";
const char* mqtt_topic_soilMoisture = "SoilMoisture";

const int mqtt_port = 8883;

//chân nối
int lightSensor = 35;
int RELAY_ledLight = 5;
int RELAY_PIN_Pump = 15;

int pinButtonPump = 27;
int TEMP_SENSOR = 32;

int ledPinWater = 33;
int ledPinWaterStart = 14;
int wateranalog = 34;
int sirensPin = 12;
int soilMoisture = 35;
int pinButtonLedRGB = 4;

int pinButtonLight = 19;

// Khai báo các chân cho cảm biến đo độ pH

// const int pinDo = 36;
// const int pinPo = 39;
// const int pinTo = 35;
// float pH_value;
// float DO_value;
// float PO_value;

//status
bool lightStatus = false;
String pumpStatus = "OFF";
bool pumpLast = true;
String getData = "OFF";
bool pumpState = false;
String pumpMode = "Manual";
String changePumpMode = "Manual";
String LedRGBStatus = "OFF";
String lastPumpStatus = "ON";
float controlTempPan = 30;
int sirensStatus = false;
bool buttonPanState = false;
bool ledWaterState = false;
bool ledStatus = false;
bool previousButtonLightState = LOW;
int lastLightButtonState = LOW;
int lastPumbButtonState = LOW;
//time
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;
const long interval = 5000;

unsigned long currentMillis2 = 0;

//thời gian chống nhiễu của nút
const unsigned long debounceDelay = 50;  // Thời gian chống nhiễu
static const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure espClient;
PubSubClient client(espClient);
DHT dht(TEMP_SENSOR, DHTTYPE);
unsigned long lastMsg = 0;
char msg[50];

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(mqtt_topic_pump);
      client.subscribe(mqtt_topic_modePum);
      // client.subscribe(mqtt_topic_soilMoisture);
      client.subscribe(mqtt_topic_light);
      // client.subscribe(mqtt_topic_fan);
      client.subscribe(mqtt_topic_RGB);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(3000);
    }
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void setup() {
  Serial.begin(115200);
  setup_wifi();
  dht.begin();
  ws2812b.begin();
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);

  // pinMode(ledBox, OUTPUT);
  pinMode(RELAY_PIN_Pump, OUTPUT);
  pinMode(pinButtonPump, INPUT);
  // pinMode(Fan, OUTPUT);
  pinMode(sirensPin, OUTPUT);
  pinMode(ledPinWater, OUTPUT);
  pinMode(ledPinWaterStart, OUTPUT);
  // digitalWrite(pinButtonFan, INPUT);
  pinMode(pinButtonLedRGB, INPUT);
  pinMode(lightSensor, INPUT);
  pinMode(RELAY_ledLight, OUTPUT);
  pinMode(pinButtonLight, INPUT);

  client.publish(mqtt_topic_pump, pumpStatus.c_str());
  client.publish(mqtt_topic_modePum, pumpMode.c_str());
  // client.publish(mqtt_topic_modeFan, moduFanState.c_str());
  client.setCallback(callback);
  // reconnect();
}

void loop() {
    if (!client.connected()) {
    reconnect();
  }
  client.loop();
  unsigned long currentMillis1 = millis();
  currentMillis2 = millis();
  unsigned long currentMillis = millis();
  ws2812b.clear();  // khong sang

  // handlePh();
  //điều khiển máy bơm
  handleModuPump();
  //điều khiển ánh sáng
  light();
  //điều khiển mực nước
  handleWaterLevel();

  //xử lý tự động hoặc vật lý của pump
  // if(pumpMode != changePumpMode) {
  handlePumpAutomation();
  // }
  //xử lý độ ẩm đất
  handleSoilMoisture();
  //Điều khiển button đèn rgb
  int pinButtonRGBState = digitalRead(pinButtonLedRGB);
  bool previousButtonState = LOW;
  if (pinButtonRGBState != previousButtonState) {
    if ((pinButtonRGBState == HIGH) && (!ledStatus)) {
      LedRGBStatus = "ON";
      ledStatus = true;
      client.publish(mqtt_topic_RGB, LedRGBStatus.c_str());
    } else if ((pinButtonRGBState == HIGH) && (ledStatus)) {
      LedRGBStatus = "OFF";
      ledStatus = false;
      Serial.print(pinButtonRGBState);
      client.publish(mqtt_topic_RGB, LedRGBStatus.c_str());
    }
    previousButtonState = pinButtonRGBState;  // Update previous state
  }
}
void handleWaterLevel() {
  int level = analogRead(wateranalog);
  // Serial.println(level);
  if (level >= 3500) {
    digitalWrite(sirensPin, HIGH);
    turnOnLED();
    sirensStatus = false;
    unsigned long startTime = millis();
    while (millis() - startTime <= 3000) {
    }
    digitalWrite(sirensPin, LOW);
    turnOffLED();
    sirensStatus = true;
    delay(100);
  } else {
    digitalWrite(sirensPin, LOW);
    turnOffLED();
    sirensStatus = true;
  }
  delay(100);
}
void handleModuPump() {
  int pinButtonPumpState = digitalRead(pinButtonPump);
  static unsigned long lastDebounceTimePump = 0;
  // Xử lý khi nút nhấn được nhấn
  if (pinButtonPumpState != lastPumbButtonState && millis() - lastDebounceTimePump > debounceDelay) {
    if (pinButtonPumpState != lastPumbButtonState) {
      lastDebounceTimePump = millis();
      if (pinButtonPumpState == HIGH) {
        if (!pumpState) {
          turnOnPump();
          client.publish(mqtt_topic_pump, pumpStatus.c_str());
        } else {
          turnOffPump();
          client.publish(mqtt_topic_pump, pumpStatus.c_str());
        }
        if (pumpMode != changePumpMode) {
          pumpMode = changePumpMode;
          client.publish(mqtt_topic_modePum, pumpMode.c_str());
        }
      }
    }
    //lưu giữa trạng thái pump
    lastPumbButtonState = pinButtonPumpState;
  }

  // if (pumpMode == "Manual") {
  //   if (getData == "ON") {
  //     turnOnPump();
  //   } else if (getData == "OFF") {
  //     turnOffPump();
  //   }
  // }
}
void handlePumpAutomation() {
  if (pumpMode == "Auto") {
    // if (getData == "ON") {
    //   turnOnPump();
    // } else if (getData == "OFF") {
    //   turnOffPump();
    // }
  }
}
void turnOnPump() {
  digitalWrite(RELAY_PIN_Pump, HIGH);
  pumpStatus = "ON";
  pumpState = true;
}

void turnOffPump() {
  digitalWrite(RELAY_PIN_Pump, LOW);
  pumpStatus = "OFF";
  pumpState = false;
}

void turnOnLED() {
  digitalWrite(ledPinWater, HIGH);
  digitalWrite(ledPinWaterStart, LOW);
  ledWaterState = true;
}
void turnOffLED() {
  digitalWrite(ledPinWater, LOW);
  digitalWrite(ledPinWaterStart, HIGH);
  ledWaterState = false;
}
void light() {
  int pinLightButtonState = digitalRead(pinButtonLight);
  static unsigned long lastDebounceTimeLight = 0;
  // Kiểm tra thời gian chống nhiễu
  if (millis() - lastDebounceTimeLight > debounceDelay) {
    // Nếu trạng thái nút thay đổi
    if (pinLightButtonState != lastLightButtonState) {
      lastDebounceTimeLight = millis();  // Cập nhật thời gian chống nhiễu cuối cùng
      // Nếu nút được nhấn
      if (pinLightButtonState == HIGH) {
        // Xử lý chức năng light
        if (!lightStatus) {
          digitalWrite(RELAY_ledLight, HIGH);
          client.publish(mqtt_topic_light, "ON");
          lightStatus = true;
        } else {
          digitalWrite(RELAY_ledLight, LOW);
          client.publish(mqtt_topic_light, "OFF");
          lightStatus = false;
        }
      }
    }
  }

  // Lưu trạng thái nút cuối cùng
  lastLightButtonState = pinLightButtonState;
}

void ledRGB() {
  if (LedRGBStatus == "ON") {
    for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {                // for each pixel
      ws2812b.setPixelColor(pixel, ws2812b.Color(red, green, blue));  // it only takes effect if pixels.show() is called
      ws2812b.show();                                                 // update to the WS2812B Led Strip
      delay(10);                                                      // 500ms pause between each pixel
    }
  } else if (LedRGBStatus == "OFF") {
    ws2812b.setPixelColor(0, 0, 0, 0);  // Tắt pixel đầu tiên
    ws2812b.show();
  }
}
// void handlePh() {
//     // Đọc giá trị từ cảm biến pH
//   int pH_raw = analogRead(pinDo);
//   pH_value = map(pH_raw, 0, 1023, 0, 14); // Chuyển đổi giá trị analog thành giá trị pH từ 0 đến 14

//   // Đọc giá trị từ cảm biến DO
//   // int DO_raw = analogRead(pinPo);
//   // DO_value = map(DO_raw, 0, 1023, 0, 100); // Chuyển đổi giá trị analog thành phần trăm DO từ 0 đến 100

//   // Đọc giá trị từ cảm biến PO
//   int PO_raw = analogRead(pinTo);
//   PO_value = map(PO_raw, 0, 1023, 0, 5); // Chuyển đổi giá trị analog thành giá trị PO từ 0 đến 5 (giả sử PO là dải từ 0 đến 5V)
//    Serial.print("pH: ");
//   Serial.print(pH_value);
//   Serial.print("\tDO: ");
//   Serial.print(DO_value);
//   Serial.print("\tPO: ");
//   Serial.print(PO_value);
//   Serial.println("V");

//   delay(1000);
// }
void handleSoilMoisture() {
  static float lastMoisture = 0;  // Sử dụng static để giữ giá trị sau mỗi lần gọi hàm
  int moisture = analogRead(soilMoisture);
  float percentage = (1 - (float)moisture / 4095) * 100;
  if (millis() - lastMoisture >= 30000) {          // Kiểm tra nếu đã đủ 5 phút
    String percentageStr = String(percentage, 1);  // Chuyển đổi thành chuỗi với một số thập phân
    client.publish(mqtt_topic_soilMoisture, percentageStr.c_str());
    lastMoisture = millis();  // Cập nhật thời gian cuối cùng gửi dữ liệu
  }
  if (pumpMode == "Auto") {
    if (percentage <= 40.0) {
      turnOnPump();
    } else {
      turnOffPump();
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = (char*)payload;
  // Serial.println(message);

  // if(String(topic) == mqtt_topic_tempFan) {
  // }
  if (String(topic) == mqtt_topic_pump) {
    if (message == "ON") {
      turnOnPump();
    } else if (message == "OFF") {
      turnOffPump();
    }
  }
  if (String(topic) == mqtt_topic_modePum) {
    if (message == "Auto" || message == "Manual") {
      pumpMode = message;
    }
  }

  if (String(topic) == mqtt_topic_light) {
    if (message == "ON") {
      digitalWrite(RELAY_ledLight, HIGH);
    } else if (message == "OFF") {
      digitalWrite(RELAY_ledLight, LOW);
    }
  }
  if (String(topic) == mqtt_topic_RGB) {
    // Phân tích mã màu
    if (length == 6) {
      // Chuyển đổi mã hex sang giá trị thập phân
      red = (uint8_t)strtol(message.substring(0, 2).c_str(), NULL, 16);
      green = (uint8_t)strtol(message.substring(2, 4).c_str(), NULL, 16);
      blue = (uint8_t)strtol(message.substring(4, 6).c_str(), NULL, 16);

      // Hiển thị màu lên LED
      ws2812b.fill(ws2812b.Color(red, green, blue));
      ws2812b.show();
    }
    if (message == "ON") {
      LedRGBStatus = "ON";
    } else {
      LedRGBStatus = "OFF";
    }
    ledRGB();
  }
}