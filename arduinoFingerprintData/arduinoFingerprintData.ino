//libraries
#include <ArduinoJson.h>
#include <Adafruit_Fingerprint.h>

//define serial for fingerprint
#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
SoftwareSerial mySerial(5, 6); // white wire to 6, green wire to 5
#else
#define mySerial Serial1
#endif
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

//define variables
int fingerprint_data;


//define JSON documents; one for sending and one for receiving data
JsonDocument doc;

void setup() {

  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial)
    continue;

  // Initialize the JSON document objects
  doc["camera_on"] = false;

  //initialize fingerprint sensor
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    // Serial.println("Found fingerprint sensor!");
  } else {
    while (1) { delay(1); }
  }

  finger.getTemplateCount();
}

void loop() {
  JsonArray data = doc["data"].to<JsonArray>();
  if(getFingerprintIDez() != -1) {
    doc["camera_on"] = false;
    data.add("AUTHORIZED");
    
  } else {
    doc["camera_on"] = true;
    data.add("UNAUTHORIZED");
  }
        //don't ned to run this at full speed.
  delay(50);
  serializeJson(doc, Serial);
  Serial.println();
 
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

