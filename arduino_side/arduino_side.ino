#include <ArduinoJson.h>  // Include the ArduinoJson library

// Define pins for servos (example pins, adjust as needed)
const int servoXPin = 9;
const int servoYPin = 10;

#include <Servo.h>  // Include the Servo library

Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  servoX.attach(servoXPin);  // Attach the servo to the pin
  servoY.attach(servoYPin);  // Attach the servo to the pin
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String jsonString = Serial.readStringUntil('\n');  // Read until newline
    StaticJsonDocument<200> doc;  // Create a JSON document (size may vary based on data)
    
    // Deserialize the JSON data
    DeserializationError error = deserializeJson(doc, jsonString);
    
    if (error) {
      Serial.print(F("Failed to parse JSON: "));
      Serial.println(error.f_str());
      return;
    }

    // Extract values from the JSON data
    bool cameraOn = doc["camera_on"] | false;
    int servoXValue = doc["servo_x"] | 90;
    int servoYValue = doc["servo_y"] | 90;

    // Limit the servo values to the range [0, 180]
    servoXValue = constrain(servoXValue, 0, 180);
    servoYValue = constrain(servoYValue, 0, 180);

    // Set servo positions
    servoX.write(servoXValue);
    servoY.write(servoYValue);

    // Optionally, you can implement additional logic to handle cameraOn state
  }

  // Delay to prevent overwhelming the serial buffer
  delay(100);
}
