#include <ArduinoJson.h>

const int HANDSHAKE_DELAY = 1000; // Handshake delay in milliseconds
const int OSCILLATION_DELAY = 2000; // Oscillation delay in milliseconds

bool personFlag = false;
bool cameraFlag = false;
bool status = true;

void setup() {
  Serial.begin(9600);
  delay(2000); // Wait for Serial Monitor to open
  //sendHandshake();
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
  sendJSON(personFlag, status, cameraFlag);
}

void sendJSON(bool person, bool status, bool camera) {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["person"] = person;\
  jsonDoc["camera"] = camera;
  jsonDoc["status"] = status;

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  Serial.println(jsonString);
}