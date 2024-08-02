#include <ArduinoJson.h>

void setup() {
  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial)
    continue;

  // Allocate the JSON document
  JsonDocument doc;

  // Add values in the document
  doc["sensor"] = "gps";
  doc["time"] = 1351824120;

  // Add an array.
  JsonArray data = doc["data"].to<JsonArray>();
  data.add(48.756080);
  data.add(2.302038);

  // Generate the minified JSON and send it to the Serial port.
  serializeJson(doc, Serial);
  // The above line prints:
  // {"sensor":"gps","time":1351824120,"data":[48.756080,2.302038]}

  // Start a new line
  Serial.println();

  // Generate the prettified JSON and send it to the Serial port.
  // serializeJsonPretty(doc, Serial);
  // The above line prints:
  // {
  //   "sensor": "gps",
  //   "time": 1351824120,
  //   "data": [
  //     48.756080,
  //     2.302038
  //   ]
  // }
}

void loop() {
  // not used in this example
}
