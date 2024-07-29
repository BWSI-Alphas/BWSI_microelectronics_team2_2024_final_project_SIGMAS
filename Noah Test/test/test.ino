#include <ArduinoJson.h>
#include <Servo.h>

Servo servoX;
Servo servoY;

const int OSCILLATION_DELAY = 10000; // Oscillation delay in milliseconds

bool personFlag = true;
bool cameraFlag = true;
bool status = true;
const int servoXPin = 9; // Pin for servo X
const int servoYPin = 10; // Pin for servo Y
const int defaultPosX = 90;
const int defaultPosY = 90;

void setup() {
  Serial.begin(9600);
  delay(2000); // Wait for Serial Monitor to open
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  servoX.write(defaultPosX);
  servoY.write(defaultPosY);
  //sendHandshake();
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  /*if (currentTime - lastUpdateTime >= OSCILLATION_DELAY) {
    lastUpdateTime = currentTime;

    // Oscillate values
    personFlag = !personFlag;
    status = !status;
    cameraFlag = !cameraFlag;

    // Send JSON data
  }*/

  sendJSON(personFlag, status, cameraFlag);
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input == "handshake") {
      Serial.println("ack");
    } else {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, input);
      if (!error) {
        bool camera_on = doc["camera_on"];
        int servo_x = doc["servo_x"];
        int servo_y = doc["servo_y"];
        servoX.write(servo_x);
        servoY.write(servo_y);
        //Serial.println("Servos updated.");
      } else {
        //Serial.println("Failed to parse JSON.");
      }
    }
  }
}

void sendJSON(bool person, bool status, bool camera) {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["person"] = person;\
  jsonDoc["camera"] = camera;
  jsonDoc["status"] = status;

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  Serial.println(jsonString);
}