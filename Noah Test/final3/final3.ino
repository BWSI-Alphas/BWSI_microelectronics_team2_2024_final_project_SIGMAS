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
const int trigPin = 3;
const int echoPin = 2;

// Threshold distance in centimeters
const int thresholdDistance = 10;
const int LED = 13;

// Variables
bool camera_on = false;

// Define serial for fingerprint
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(6, 7); // White wire to 6, green wire to 5
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
  //while (!Serial) continue;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  finger.begin(57600);
  delay(5);

  if (finger.verifyPassword()) {
    Serial.println("Fingerprint sensor found!");
  } else {
    Serial.println("Fingerprint sensor not found :(");
    while (1) { delay(1); }
  }

  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  servoX.write(defaultPosX);
  servoY.write(defaultPosY);

  finger.getTemplateCount();
  Serial.print("Sensor contains "); 
  Serial.print(finger.templateCount); 
  Serial.println(" templates");
}

void loop() {
  if(ultraSonic()){
    camera_on = true;
  }
  if (getFingerprintIDez() != -1){
    camera_on = false;
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input == "handshake") {
      Serial.println("ack");
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, input);
      if (!error) {
        camera_on = doc["camera_on"];
        int servo_x = doc["servo_x"];
        int servo_y = doc["servo_y"];
        servoX.write(servo_x);
        servoY.write(servo_y);
        sendJSON(camera_on);
        //Serial.println("Servos updated.");
      }
    }
  }
}

float distance1;
long duration;
float ultraSoundRead() {
  digitalWrite(trigPin, LOW);
  delay(2);
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance1 = duration * 0.034 / 2;
  if (distance1 > 40) {
    return -1;
  }
  return distance1;
}

float normalizeUSRead() {
  int count1 = 0;
  float sum = 0.0;
  int timeDelay = 5;
  int normTime = 100;

  for (int i = 0; i < 200 / normTime; i++) {
    float currVal = ultraSoundRead();
    if (currVal != -1) {
      count1++;
      sum += currVal;
    }
    delay(timeDelay);
  }

  if (count1 < 2) {
    return -1.00;
  }
  return (float)sum / (float)(count1);
}

bool ultraSonic() {
  // Calculating distance in centimeters
  int distance = normalizeUSRead(); 

  //Serial.println(distance);

  // Check if distance is less than threshold
  if (distance < thresholdDistance && distance != -1) {
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

void sendJSON(bool camera) {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["camera"] = camera;

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  Serial.println(jsonString);
}