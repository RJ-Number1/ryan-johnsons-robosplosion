#include "Arduino.h"
#include "Sensors.h"
#include "Pins.h"
#include <Wire.h>
Sensors::Sensors() {
  }
  
void Sensors::reset(void) {
  Wire.begin();
  pinMode(SENSOR_LEFT_REAR,OUTPUT);
  pinMode(SENSOR_LEFT_FRONT,OUTPUT);
  pinMode(SENSOR_FRONT_LEFT,OUTPUT);
  pinMode(SENSOR_FRONT_RIGHT,OUTPUT);
  pinMode(SENSOR_RIGHT_FRONT,OUTPUT);
  pinMode(SENSOR_RIGHT_REAR,OUTPUT);
  
  digitalWrite(SENSOR_LEFT_REAR,LOW);
  digitalWrite(SENSOR_LEFT_FRONT,LOW);
  digitalWrite(SENSOR_FRONT_LEFT,LOW);
  digitalWrite(SENSOR_FRONT_RIGHT,LOW);
  digitalWrite(SENSOR_RIGHT_FRONT,LOW);
  digitalWrite(SENSOR_RIGHT_REAR,LOW);
  
  delay(10);
  
  digitalWrite(SENSOR_LEFT_REAR,HIGH);
  digitalWrite(SENSOR_LEFT_FRONT,HIGH);
  digitalWrite(SENSOR_FRONT_LEFT,HIGH);
  digitalWrite(SENSOR_FRONT_RIGHT,HIGH);
  digitalWrite(SENSOR_RIGHT_FRONT,HIGH);
  digitalWrite(SENSOR_RIGHT_REAR,HIGH);
  
  digitalWrite(SENSOR_LEFT_REAR,LOW);
  digitalWrite(SENSOR_LEFT_FRONT,LOW);
  digitalWrite(SENSOR_FRONT_LEFT,LOW);
  digitalWrite(SENSOR_FRONT_RIGHT,LOW);
  digitalWrite(SENSOR_RIGHT_FRONT,LOW);
  digitalWrite(SENSOR_RIGHT_REAR,LOW);
  
  delay(10);
  
  
}
void Sensors::setChannel(void) {

}

//  xbeeComm.print("LB - " + String(leftBack.readRangeContinuousMillimeters()) + " | ");
//  if (leftBack.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
//  xbeeComm.print("LF - " + String(leftFront.readRangeContinuousMillimeters()) + " | ");
//  if (leftFront.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
//  xbeeComm.print("FL - " + String(frontLeft.readRangeContinuousMillimeters()) + " | ");
//  if (frontLeft.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
//  xbeeComm.print("FR - " + String(frontRight.readRangeContinuousMillimeters()) + " | ");
//  if (frontRight.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
//  xbeeComm.print("RF - " + String(rightFront.readRangeContinuousMillimeters()) + " | ");
//  if (rightFront.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
//  xbeeComm.print("RF - " + String(rightFront.readRangeContinuousMillimeters()) + " | ");
//  if (rightBack.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
//  xbeeComm.println();
//  delay(100);


