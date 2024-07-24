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

//define JSON document
JsonDocument doc;

void setup() {
  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial)
    continue;

  // Allocate the JSON document
  // JsonDocument doc;
  doc["Person_verify"] = false;

  //initialize fingerprint sensor
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    // Serial.println("Found fingerprint sensor!");
  } else {
    // Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }

  finger.getTemplateCount();
  // // Add an array.
  // JsonArray data = doc["data"].to<JsonArray>();
  // data.add(48.756080);
  // data.add(2.302038);
}

  void loop()                     // run over and over again
{
  if(getFingerprintIDez() != -1) {
    doc["Person_verify"] = fingerprint_data;
  } else {
    doc["Person_verify"] = -1;
  }
  delay(50);            //don't ned to run this at full speed.
  if (doc["Person_verify"] != -1) {
    serializeJson(doc, Serial);
    Serial.println();
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

