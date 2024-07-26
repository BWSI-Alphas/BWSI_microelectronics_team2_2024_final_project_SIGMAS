#include <Servo.h>
#include <ArduinoJson.h>

const int servoPanPin = 10;  // Pin for pan servo
const int servoTiltPin = 9;  // Pin for tilt servo

Servo panServo;
Servo tiltServo;

bool detect = false;
unsigned long lastDetectTime = 0;

void setup() {
  Serial.begin(9600);

  panServo.attach(servoPanPin);
  tiltServo.attach(servoTiltPin);

  tiltServo.write(145);  // Initialize to middle position
  panServo.write(90);
}

void loop() {
  if (!detect) {
    for (int i = 0; i <= 180; i += 5) {
      panServo.write(i);
      delay(50);
      tiltServo.write(90 + (i / 2));
      delay(50);

      // Check for serial data during the scan
      if (Serial.available()) {
        String jsonData = Serial.readStringUntil('\n');
        DynamicJsonDocument doc(1024);

        // Deserialize the JSON data
        DeserializationError error = deserializeJson(doc, jsonData);

        if (error) {
          Serial.print("JSON deserialization failed: ");
          Serial.println(error.c_str());
          return;
        }

        // Extract values from JSON
        detect = doc["detect"];

        if (detect) {
          // Detected person, wait for 2 seconds
          delay(2000);
          lastDetectTime = millis();
          break;
        }
      }
    }
    for (int i = 180; i >= 0; i -= 5) {
      panServo.write(i);
      delay(50);
      tiltServo.write(90 + (i / 2));
      delay(50);

      // Check for serial data during the scan
      if (Serial.available()) {
        String jsonData = Serial.readStringUntil('\n');
        DynamicJsonDocument doc(1024);

        // Deserialize the JSON data
        DeserializationError error = deserializeJson(doc, jsonData);

        if (error) {
          Serial.print("JSON deserialization failed: ");
          Serial.println(error.c_str());
          return;
        }

        // Extract values from JSON
        detect = doc["detect"];

        if (detect) {
          // Detected person, wait for 2 seconds
          delay(2000);
          lastDetectTime = millis();
          break;
        }
      }
    }
  } else {
    // Check if 2 seconds have passed since last detection
    if (millis() - lastDetectTime > 2000) {
      detect = false;
    }
  }

  // Continuously check for serial data
  if (Serial.available()) {
    String jsonData = Serial.readStringUntil('\n');
    DynamicJsonDocument doc(1024);

    // Deserialize the JSON data
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.print("JSON deserialization failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract values from JSON
    detect = doc["detect"];
  }
}
