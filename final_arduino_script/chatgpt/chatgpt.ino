#include <Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_Fingerprint.h>

const int trigPin = 12;
const int echoPin = 13;
const int thresholdDistance = 60;
const int LED = 8;

Servo panServo;
Servo tiltServo;
bool stop = false;
bool turn = false;
bool turnOnCam = false;
bool onceTurnOnSensor = false;
bool onceTurnOnCam = false;

#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(5, 6);
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

int fingerprint_data;
unsigned long previousMillis = 0;
const long interval = 1000;

StaticJsonDocument<200> sendDoc, receiveDoc;

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  finger.begin(57600);
  delay(5);
  if (!finger.verifyPassword()) {
    while (1) { delay(1); }
  }
  finger.getTemplateCount();
  
  sendDoc["led"] = "off";
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (!onceTurnOnSensor && ultraSonic()) {
      turnOnCam = ultraSonic();
      onceTurnOnSensor = true;
    }
  }

  if (turnOnCam) {
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

  if (Serial.available() > 0) {
    String incomingJson = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(receiveDoc, incomingJson);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    if (receiveDoc["led"] == "off") {
      digitalWrite(LED, LOW);
    } else {
      digitalWrite(LED, HIGH);
    }
  }
}

bool ultraSonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  return distance < thresholdDistance;
}

int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK) return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) return -1;

  return finger.fingerID;
}
