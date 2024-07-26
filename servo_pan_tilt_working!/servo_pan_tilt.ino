#include <Servo.h>
#include <ArduinoJson.h>

const int servoPanPin = 9; // Pin for pan servo
const int servoTiltPin = 10; // Pin for tilt servo

Servo panServo;
Servo tiltServo;

const int pulsePanMin = 600;
const int pulsePanMax = 2400;
const int pulseTiltMin = 600;
const int pulseTiltMax = 2400;

void setup() {
  Serial.begin(9600);
  
  panServo.attach(servoPanPin, pulsePanMin, pulsePanMax);
  tiltServo.attach(servoTiltPin, pulseTiltMin, pulseTiltMax);
  
  panServo.writeMicroseconds(90); // Initialize to middle position
  tiltServo.write(90); // Initialize to middle position
}

void loop() {
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
    bool cameraOn = doc["camera_on"];
    int servoX = doc["servo_x"];
    int servoY = doc["servo_y"];
    
    // Print values for debugging
    Serial.print("Camera On: ");
    Serial.println(cameraOn);
    Serial.print("Servo X: ");
    Serial.println(servoX);
    Serial.print("Servo Y: ");
    Serial.println(servoY);
    
    // Move servos based on received values
    panServo.write(servoX);
    tiltServo.write(servoY);
  }
}
