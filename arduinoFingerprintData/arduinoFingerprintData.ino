//libraries
#include <ArduinoJson.h>
#include <Adafruit_Fingerprint.h>

//define serial for fingerprint
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(10, 11);
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

//define variables
int fingerprint_data;

const int LED = 8;

//define JSON document
JsonDocument doc;

void setup() {
  
  pinMode(LED, OUTPUT);

  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial)
    continue;

  // Initialize the JSON document objects
  doc["camera_on"] = false;
  doc["servo_x"] = 90;
  doc["servo_y"] = 90;

  //initialize fingerprint sensor
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    // Serial.println("Found fingerprint sensor!");
  } else {
    while (1) { delay(1); }
  }

  finger.getTemplateCount();
  // // Add an array.
  // JsonArray data = doc["data"].to<JsonArray>();
  // data.add(48.756080);
  // data.add(2.302038);
}

void loop() {
  if(getFingerprintIDez() != -1) {
    doc["camera_on"] = true;
  } else {
    doc["camera_on"] = false;
  }
  delay(50);            //don't ned to run this at full speed.
  if (doc["camera_on"] == true) {
    serializeJson(doc, Serial);
    Serial.println();
  } 
  if (Serial.available() > 0) { // Check if data is available to read
    String message = Serial.readStringUntil('\n'); // Read the incoming message until a newline character

    message.trim(); // Remove any leading/trailing whitespace

    if (message == "Message Received") { // Check if the message matches
      digitalWrite(LED, HIGH); // Set the pin high
    }
  } else {
    digitalWrite(LED, LOW); // Set the pin low
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
