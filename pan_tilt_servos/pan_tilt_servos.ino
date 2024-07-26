#include <Servo.h>
#include <Wire.h>

Servo pan, tilt;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pan.attach(10);
  tilt.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int panAngle = 0; panAngle <=180; panAngle++){
    pan.write(panAngle);
    Serial.println(panAngle);
    delay(50);
  }
  for(int tiltAngle = 90; tiltAngle <=180; tiltAngle++){
    tilt.write(tiltAngle);
    Serial.println(tiltAngle);
    delay(50);
   }
   for(int panAngle = 180; panAngle >=0; panAngle--){
    pan.write(panAngle);
    Serial.println(panAngle);
    delay(50);
  }
  for(int tiltAngle = 180; tiltAngle >=90; tiltAngle--){
    tilt.write(tiltAngle);
    Serial.println(tiltAngle);
    delay(50);
   }
  

}
