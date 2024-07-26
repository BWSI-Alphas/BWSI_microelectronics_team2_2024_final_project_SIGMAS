
#include <Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>

Servo panServo;
Servo tiltServo;
bool stop = false;
bool turn = false;

void setup() {
  Serial.begin(9600);
  panServo.attach(10);  // Pin 10 for pan servo
  tiltServo.attach(9); // Pin 9 for tilt servo
  panServo.write(90);  // Center position
  tiltServo.write(140); // Center position
}

void loop() {
  //Move the pan servo back and forth
  if (stop && !turn) { //turn one way
    if (panServo.read() < 180) {
      panServo.write(panServo.read() + 1);
    } else {
      turn = true;
    }
  } else if (stop && turn) {
    if (panServo.read() > 0) {//turn the other
      panServo.write(panServo.read() - 1);
    } else {
      turn=false;
    }
  }else{
    delay(100);
  }


  // Check for incoming serial commands
  if (Serial.available() > 0) {
    char received = Serial.read();
    Serial.print("Received command: ");
    Serial.println(received);
    if (received == '1') {
      stop = true;
      Serial.println("Servo movement resumed");
    } else if (received == '0') {
      stop = false;
      Serial.println("Servo movement stopped");
    }
  }

  
}

