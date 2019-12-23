#include <Servo.h>
int servoPin = 2 ;
int angle = 0 ;
int incomingByte = 0; // for incoming serial data
Servo servo1;

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  servo1.attach(servoPin);
  servo1.write(0);
  delay(200);
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    angle = Serial.parseInt();   
    pinMode(servoPin,OUTPUT);
    servo1.write(angle);
    delay(100);
    Serial.println(angle);
  }
  pinMode(servoPin,INPUT);
}
