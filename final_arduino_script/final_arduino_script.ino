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
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(WiFi.status());
    delay(500);
    
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  
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
