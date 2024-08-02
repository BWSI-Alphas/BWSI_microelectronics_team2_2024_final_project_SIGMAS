#include <Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_Fingerprint.h>

Servo servoX;
Servo servoY;

const int servoXPin = 9; // Pin for servo X
const int servoYPin = 10; // Pin for servo Y
const int defaultPosX = 90;
const int defaultPosY = 90;
const int trigPin = 12;
const int echoPin = 13;

// Threshold distance in centimeters
const int thresholdDistance = 60;
const int LED = 13;

// Variables
bool turnOnCam = false;
bool onceTurnOnSensor = false;
bool onceTurnOnCam = false;

// Define serial for fingerprint
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(5, 6); // White wire to 6, green wire to 5
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// Timing variables for trigger ultrasonic
unsigned long previousMillis = 0;
const long interval = 1000; // Interval at which to trigger (milliseconds)

StaticJsonDocument<200> sendDoc, receiveDoc;

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(LED, OUTPUT); // LED for testing
  while (!Serial) continue;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  finger.begin(57600);
  delay(5);

  if (finger.verifyPassword()) {
    // Fingerprint sensor found
  } else {
    while (1) { delay(1); }
  }

  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  servoX.write(defaultPosX);
  servoY.write(defaultPosY);

  finger.getTemplateCount();
  // Initialize fingerprint sensor

  sendDoc["led"] = "off";
  Serial.setDebugOutput(true);
  Serial.println();

  // Add any necessary camera initialization here
  
  // WiFi initialization, if needed
  // WiFi.begin(ssid, password);
  // WiFi.setSleep(false);
  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.print(WiFi.status());
  //   delay(500);
  // }
  // Serial.println("");
  // Serial.println("WiFi connected");
  // startCameraServer();
  // Serial.print("Camera Ready! Use 'http://");
  // Serial.print(WiFi.localIP());
  // Serial.println("' to connect");
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= OSCILLATION_DELAY) {
    lastUpdateTime = currentTime;

    // Oscillate values
    personFlag = !personFlag;
    status = !status;
    cameraFlag = !cameraFlag;

    // Send JSON data
  }
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (!onceTurnOnSensor && ultraSonic()) {
      camera_on = true;
    }
  }
  if (camera_on) {
        sendDoc["person"] = "detected";
        if (getFingerprintIDez() != -1) {
          sendDoc["status"] = "AUTHORIZED";
          sendDoc["camera"] = "off";
          onceTurnOnCam = true;
        } else {
          sendDoc["status"] = "UNAUTHORIZED";
          if (!onceTurnOnCam) {
            sendDoc["camera"] = "on";
          } else {
            sendDoc["camera"] = "off";
          }
        }
        serializeJson(sendDoc, Serial);
        Serial.println();
  }
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

      //send data
      if (camera_on) {
        sendDoc["person"] = "detected";
        if (getFingerprintIDez() != -1) {
          sendDoc["status"] = "AUTHORIZED";
          sendDoc["camera"] = "off";
          onceTurnOnCam = true;
        } else {
          sendDoc["status"] = "UNAUTHORIZED";
          if (!onceTurnOnCam) {
            sendDoc["camera"] = "on";
          } else {
            sendDoc["camera"] = "off";
          }
        }
        serializeJson(sendDoc, Serial);
        Serial.println();
      }
    }
  }
}

bool ultraSonic() {
  // Triggering the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reading the echo pulse
  long duration = pulseIn(echoPin, HIGH);

  // Calculating distance in centimeters
  int distance = duration * 0.034 / 2;

  // Check if distance is less than threshold
  if (distance < thresholdDistance) {
    return true; // Detects an object, cue to turn on camera stream
  } else {
    return false;
  }
}

// Returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) return -1;

  // Found a match!
  return finger.fingerID;
}
