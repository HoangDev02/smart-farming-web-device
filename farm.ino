#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#include <DHT.h>
#define DHTTYPE DHT11
#define SensorPin A0         //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.70          //deviation compensate Bù TRỪ PH
unsigned long int avgValue;  //Store the average value of the sensor feedback
#define WIFI_SSID "Redmi Note 12"
#define WIFI_PASSWORD "11092002"
const char* mqtt_server = "f63a3874d1364c15ba9d13699c92dc63.s1.eu.hivemq.cloud";
const char* mqtt_username = "esp8266Den";
const char* mqtt_password = "esp8266Den";
const char* mqtt_topic_ButtonDoor = "DOOR";
const char* mqtt_topic_inforUser = "USER";
const char* mqtt_topic_modeFan = "MODEFAN";
const char* mqtt_topic_Temp = "Temp";
const char* mqtt_topic_Humi = "Humi";
const char* mqtt_topic_fan = "Fan";
const char* mqtt_topic_PH = "PH";
int Fan = 16;
int pinButtonFan = 13;
String fanStatus = "OFF";
String moduFanState = "Manual";
String changeFanMode = "Manual";
bool buttonPanState = false;
float controlTempPan = 30;
int TEMP_SENSOR = 12;

DHT dht(TEMP_SENSOR, DHTTYPE);

SoftwareSerial megaSerial(0, 1);
WiFiClientSecure espClient;
PubSubClient client(espClient);


int prevButtonStateFloor = -1;

String lastMessageButtonDoor = "";
String lastDataButtonFan;

String dataUser = "";
int lastLightButtonState = LOW;

//time
unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;
const long interval = 5000;

unsigned long currentMillis2 = 0;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 20;  // Thời gian chống nhiễu
unsigned long lastMsg = 0;
const long intervalPH = 40000;  // Số mili giây trong một phút
void setup() {
  Serial.begin(115200);
  megaSerial.begin(115200);
  // pinMode(ledPin, OUTPUT);
  // pinMode(pinButtonLed, INPUT);
  // pinMode(ledDistance, OUTPUT);
  // pinMode(distance, INPUT);
  // pinMode(fanPin, OUTPUT);

  pinMode(Fan, OUTPUT);
  digitalWrite(pinButtonFan, INPUT);

  dht.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("---Team 3---");
  lcd.setCursor(0, 1);
  lcd.print("Smart Farming");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Thiết lập kết nối MQTT
  espClient.setInsecure();
  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP8266Client", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(mqtt_topic_ButtonDoor);
      client.subscribe(mqtt_topic_inforUser);
      client.subscribe(mqtt_topic_modeFan);
      client.subscribe(mqtt_topic_fan);
    } else {
      Serial.print("Failed to connect with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop() {
  client.loop();
  unsigned long currentMillis = millis();

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    int commaIndex = data.indexOf(',');
    String buttonStateFloorStr = data.substring(0, commaIndex);  // Lấy trạng thái nút
    String phValueStr = data.substring(commaIndex + 1);          // Lấy giá trị độ pH

    int buttonStateFloor = buttonStateFloorStr.toInt();
    float pHValue = phValueStr.toFloat();
    String phvalueStr = String(pHValue);
    if (currentMillis - lastMsg >= intervalPH ) {
      client.publish(mqtt_topic_PH, phvalueStr.c_str());

      // Cập nhật thời gian lần gửi cuối cùng
      lastMsg = currentMillis;
    }

    if (buttonStateFloor != prevButtonStateFloor) {
      String buttonStateFloorTopic = String(buttonStateFloor);
      if (buttonStateFloorTopic == "1") {
        client.publish(mqtt_topic_ButtonDoor, "ON");
        prevButtonStateFloor = buttonStateFloor;
        if (dataUser.length() >= 3) {
          int commaIndex = dataUser.indexOf(',');
          String firstPart = dataUser.substring(0, commaIndex);  // Lấy phần đầu của chuỗi
          String secondPart = dataUser.substring(commaIndex + 1);
          lcd.clear();
          lcd.setCursor(1, 0);
          lcd.print("--Da diem danh--");
          lcd.setCursor(0, 1);
          lcd.print("ID:" + firstPart + " Name:" + secondPart);
          delay(2000);
          lcd.clear();
          lcd.setCursor(1, 0);
          lcd.print("-----Team 3----");
          lcd.setCursor(0, 1);
          lcd.print("--Smart Farming--");
          dataUser = "";
        } else {
          lcd.clear();
          lcd.setCursor(1, 0);
          lcd.print("Da mo cua...");
          delay(2000);
          lcd.clear();
          lcd.setCursor(1, 0);
          lcd.print("-----Team 3----");
          lcd.setCursor(0, 1);
          lcd.print("--Smart Farming--");
        }

      } else if (buttonStateFloorTopic == "0") {
        client.publish(mqtt_topic_ButtonDoor, "OFF");
        prevButtonStateFloor = buttonStateFloor;
      }
    }
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  String temAndHumi = "Temperature: " + String(t) + "°C, Humidity: " + String(h) + "%";
  String Temp = String(t);
  String Humi = String(h);
  if (currentMillis - previousMillis >= interval) {
    client.publish(mqtt_topic_Temp, Temp.c_str());
    client.publish(mqtt_topic_Humi, Humi.c_str());
    previousMillis = currentMillis;
  }
  handleModuFan(t, h);
  handleAutomationFan(t, h);
}

void turnOnFan() {
  digitalWrite(Fan, HIGH);
  fanStatus == "ON";
  buttonPanState = true;
}

void turnOffFan() {
  digitalWrite(Fan, LOW);
  fanStatus == "OFF";
  buttonPanState = false;
}

// void turnOffLED() {
//   digitalWrite(ledPin, LOW);
//   ledState = false;
// }
void handleModuFan(float t, float h) {
  int pinButtonFanState = digitalRead(pinButtonFan);
  if (millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();
    if (pinButtonFanState == HIGH) {
      if (!buttonPanState) {
        turnOnFan();
        client.publish(mqtt_topic_fan, "ON");
        // delay(1000);
      } else if (buttonPanState) {
        turnOffFan();
        client.publish(mqtt_topic_fan, "OFF");

        // delay(1000);
      }

      if (moduFanState != changeFanMode) {
        moduFanState = changeFanMode;
        client.publish(mqtt_topic_modeFan, moduFanState.c_str());
      }
    }
  }
  lastLightButtonState = pinButtonFanState;
}
void handleAutomationFan(float t, float h) {
  if (moduFanState == "Auto") {
    if (t > controlTempPan) {
      turnOnFan();
    } else {
      turnOffFan();
    }
  } else if (moduFanState == "Manual") {
    if (fanStatus == "ON") {
      turnOnFan();
    } else if (fanStatus == "OFF") {
      turnOffFan();
    }
  }
}
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  // Chuyển payload sang String
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  // Serial.println(message);
  if (String(topic) == mqtt_topic_ButtonDoor) {
    if (message != lastMessageButtonDoor) {
      // megaSerial.print("LEDON");
      if (message == "ON") {
        megaSerial.print("1");

      } else if (message == "OFF") {
        megaSerial.print("0");
      }
      lastMessageButtonDoor = message;
    }
  }
  if (String(topic) == mqtt_topic_inforUser) {
    dataUser = message;
  }
  if (String(topic) == mqtt_topic_modeFan) {
    if (message == "Auto" || message == "Manual") {
      moduFanState = message;
    }
  }
  if (String(topic) == mqtt_topic_fan) {
    if (length >= 2) {
      controlTempPan = message.toFloat();
    }
    if (message == "ON") {
      fanStatus = "ON";
    } else if (message == "OFF") {
      fanStatus = "OFF";
    }
  }
}
