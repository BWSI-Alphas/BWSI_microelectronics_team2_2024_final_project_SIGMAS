#include <ArduinoJson.h>
#include <Servo.h>

const int servoXPin = 9;  // Pin connected to servo X
const int servoYPin = 10; // Pin connected to servo Y

Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  servoX.attach(servoXPin); // Attach the servo to the pin
  servoY.attach(servoYPin); // Attach the servo to the pin
}

void loop() {
  if (Serial.available() > 0) {
    String jsonString = Serial.readStringUntil('\n');
    
    // Create a JSON document
    DynamicJsonDocument doc(1024);

    // Deserialize the JSON string into the JSON document
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract the values from the JSON document
    String servo = doc["servo"];
    int angle = doc["angle"];

    // Print the values to the Serial Monitor for debugging
    Serial.print("Received servo: ");
    Serial.print(servo);
    Serial.print(", angle: ");
    Serial.println(angle);

    // Control the servos based on the received data
    if (servo == "x") {
      servoX.write(angle);
    } else if (servo == "y") {
      servoY.write(angle);
    }
  }
}
