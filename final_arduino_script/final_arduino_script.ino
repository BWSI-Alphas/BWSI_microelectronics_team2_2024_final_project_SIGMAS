#include <Servo.h>// servos
#include <Wire.h> // servos
#include <ArduinoJson.h>// comm
#include <Adafruit_Fingerprint.h>// fingerprint sensor

const int trigPin = 12;
const int echoPin = 13;

// threshold distance in centimeters
const int thresholdDistance = 60;
const int LED = 8;
//variables
Servo panServo;
Servo tiltServo;
bool stop = false; //true when a person is detected and servos latch onto the person
bool turn = false; //boolean for panning servo from 0-180 degrees
bool turnOnCam;
bool onceTurnOnSensor = false;
bool onceTurnOnCam = false;

//define serial for fingerprint
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(5, 6); // white wire to 6, green wire to 5
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

int fingerprint_data;
// Timing variables for trigger ultrasonic
unsigned long previousMillis = 0;
const long interval = 1000;  // interval at which to trigger (milliseconds)

StaticJsonDocument<200> sendDoc, receiveDoc;

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(LED, OUTPUT); //led for testing
  while (!Serial)
    continue;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    // Serial.println("Found fingerprint sensor!");
  } else {
    while (1) { delay(1); }
  }

  finger.getTemplateCount();
  //initialize fingerprint sensor
  
  sendDoc["led"] = "off";
  
}

void loop() {
  // Check if it's time to trigger the ultrasonic sensor
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if(!onceTurnOnSensor && ultraSonic()){
      turnOnCam = ultraSonic();
      onceTurnOnSensor = true;
    }
  }

  if(turnOnCam){
    sendDoc["person"] = "detected";
    if(getFingerprintIDez() != -1) {
      sendDoc["status"] = "AUTHORIZED";
      sendDoc["camera"] = "off";
      onceTurnOnCam = true;
    } else {
      sendDoc["status"] = "UNAUTHORIZED";
      if(!onceTurnOnCam){
        sendDoc["camera"] = "on";
      }else{
        sendDoc["camera"] = "off";
      }
    }
    serializeJson(sendDoc, Serial);
    Serial.println();
  }

  if (Serial.available() > 0) { // Check if data is available to read
    String incomingJson = Serial.readStringUntil('\n'); // read incoming json data
    deserializeJson(receiveDoc, incomingJson); //deserialize the json data, allows us to work with it

    if (receiveDoc["led"] == "off") {
      digitalWrite(LED, LOW); // Set the pin low
    } else {
      digitalWrite(LED, HIGH); // Set the pin high
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
    return true; //detects an object, cue to turn on camera stream
  } else {
    return false;
  }
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!
  fingerprint_data = finger.fingerID;
  return finger.fingerID;
}
