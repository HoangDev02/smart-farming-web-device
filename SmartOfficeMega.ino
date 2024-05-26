#include <Keypad.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Stepper.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Bounce2.h>
#include <Stepper.h>

#define SS_PIN 53
#define RST_PIN 5

#define MOTOR_PIN1 11
#define MOTOR_PIN2 12
#define MOTOR_PIN3 13
#define MOTOR_PIN4 8
#define STEPS_PER_REVOLUTION 200
boolean isMotorRotatedMotor = false;

Stepper myStepper(STEPS_PER_REVOLUTION, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);
Servo servo;

const int servoPin = 6;
const int buttonPin = 22;
int sirensPin = 7;
int sirensStatus = false;
int pinLedButton = 2;
boolean servoActivated = false;
String passwordMfrc522 = "67911b2e";
const int RED_PIN = 24;    // led đỏ
const int GREEN_PIN = 25;  // led xanh lá
const int BLUE_PIN = 26;   // led xanh dương
//set time
const long interval = 5000;
unsigned long previousMillis = 0;
int currentPosition = 0;
MFRC522 mfrc522(SS_PIN, RST_PIN);
#define SensorPin A0       //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.70          //deviation compensate Bù TRỪ PH
unsigned long int avgValue;  //Store the average value of the sensor feedback

float phValue;
void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
  pinMode(pinLedButton, OUTPUT);
  servo.attach(servoPin);
  pinMode(sirensPin, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  SPI.begin();
  servo.write(10);
  mfrc522.PCD_Init();
  myStepper.setSpeed(115);
  myStepper.step(570);
}

void loop() {

  if (Serial.available()) {
    String data = "";
    while (Serial.available()) {
      data = Serial.readStringUntil('\n');
    }
    // Serial.println(data);
    processSerialData(data);
  }

  int buttonState = digitalRead(buttonPin);
  unsigned long currentMillis = millis();
  
  if (buttonState == HIGH && servoActivated) {
    gradualCloseDoor();
    servoActivated = false;
    delay(1500);

  } else if (buttonState == HIGH && !servoActivated) {
    gradualOpenDoor();
    servoActivated = true;
    String sendData = String(servoActivated);
    Serial.println(sendData);
    delay(3000);
    gradualCloseDoor();
    servoActivated = false;
    sendData = String(servoActivated);
    Serial.println(sendData);
    delay(1500);
  }
  if (servoActivated == 1) {
    digitalWrite(pinLedButton, HIGH);
  } else {
    digitalWrite(pinLedButton, LOW);
  }
  if (currentMillis - previousMillis >= interval) {
    // Làm điều gì đó mỗi 5 giây ở đây
    String sendData = String(servoActivated) + "," + String(phValue);

    // Gửi dữ liệu qua Serial (TX)
    Serial.println(sendData);

    // Cập nhật thời gian trước đó
    previousMillis = currentMillis;
  }
  readMfrc522();
  handlereadPh();
}
void handlereadPh() {
  int buf[10];                  //buffer for read analog
  for (int i = 0; i < 10; i++)  //Get 10 sample value from the sensor for smooth the value
  {
    buf[i] = analogRead(SensorPin);
  }
  for (int i = 0; i < 9; i++)  //sort the analog from small to large
  {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        int temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++)  //take the average value of 6 center sample
    avgValue += buf[i];
  phValue = (float)avgValue * 5.0 / 1024 / 6;  //convert the analog into millivolt
  phValue = 2.0 * phValue + Offset;                  //convert the millivolt into pH value  chuyển đổi milivolt thành giá trị pH
  //   Serial.print("    pH:");  
  // Serial.print(phValue,2);
  // Serial.println(" ");
  //  String phStr = String(phValue);
  // client.publish(mqtt_topic_PH,phStr.c_str());
}
void displayColorRed() {
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}
void displayColorBlue() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
}
void displayColorGreen() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, LOW);
}
void toggleDoor() {
  if (gradualOpenDoor) {
    gradualOpenDoor();
    servoActivated = true;
    delay(100);
  } else {
    gradualCloseDoor();
    servoActivated = false;
    delay(100);
  }
}
void playSiren(int onDuration, int offDuration) {
  digitalWrite(sirensPin, HIGH);
  delay(onDuration);

  digitalWrite(sirensPin, LOW);
  delay(offDuration);
}
void gradualOpenDoor() {

  int targetServoPosition = 110;
  int currentServoPosition = servo.read();

  int increment = 1;  // bước tăng mỗi lần

  for (int i = 0; i < 2; i++) {
    playSiren(100, 100);  // Play siren 3 times
  }

  // while (currentServoPosition < targetServoPosition) {
  //   digitalWrite(sirensPin, LOW);
  //   currentServoPosition += 2;
  //   servo.write(currentServoPosition);
  //   delay(15);
  // }
  myStepper.setSpeed(110);
  myStepper.step(-670);


  displayColorBlue();
}

void gradualCloseDoor() {
  int targetServoPosition = 10;
  int currentServoPosition = servo.read();

  for (int i = 0; i < 2; i++) {
    playSiren(100, 100);  // Play siren 3 times
  }
  myStepper.setSpeed(115);
  myStepper.step(1000);
  // while (currentServoPosition > targetServoPosition) {
  //   digitalWrite(sirensPin, LOW);
  //   currentServoPosition -= 3;
  //   servo.write(currentServoPosition);
  //   delay(15);
  // }
  displayColorRed();
}

void processSerialData(String data) {
  if (data == "1" && !servoActivated) {
    gradualOpenDoor();
    delay(1000);
    servoActivated = true;
    String sendData = String(servoActivated);
    Serial.println(sendData);
    delay(3000);
    gradualCloseDoor();
    servoActivated = false;
    sendData = String(servoActivated);
    Serial.println(sendData);
    delay(1500);
  } else if (data == "0" && servoActivated) {
    gradualCloseDoor();
    delay(1000);
    servoActivated = false;
    delay(1500);
  }
}

void readMfrc522() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String uid = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      uid += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
      uid += String(mfrc522.uid.uidByte[i], HEX);
    }

    mfrc522.PICC_HaltA();

    for (int i = 0; i < 3; i++) {
      if (uid != "67911b2e") {
        displayColorGreen();
        delay(1000);  // Đèn sáng trong 3 giây
        displayColorRed();
      }
    }
    if (uid == "67911b2e") {
      switch (servoActivated) {
        case false:
          gradualOpenDoor();
          servoActivated = true;
          delay(100);
          break;
        case true:
          gradualCloseDoor();
          servoActivated = false;
          delay(100);
          break;
      }
    }
  }
}