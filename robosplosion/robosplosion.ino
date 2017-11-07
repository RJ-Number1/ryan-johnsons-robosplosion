#include <SoftwareSerial.h>
#include "DriveMotors.h"
#include "HCSR04.h"
#include "Mallet.h"
#include "Pins.h"
#include <Servo.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X leftBack;
VL53L0X leftFront;
VL53L0X frontLeft;
VL53L0X frontRight;
VL53L0X rightFront;
VL53L0X rightBack;

#define xbeeComm Serial3
DriveMotors myMotors(MOTOR_LEFT_CH0, MOTOR_LEFT_CH1,
                    MOTOR_RIGHT_CH0, MOTOR_RIGHT_CH1);
Servo mallet;
//Mallet myMallet(&mallet);

byte newByte;

void setup() {
  Serial.begin(9600);
  xbeeComm.begin(9600);
  mallet.attach(MALLET_PIN);
  mallet.write(MALLET_RETRACT_POS);
  while (! Serial) {
    delay(1);
  }
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
  
  pinMode(SENSOR_LEFT_REAR, INPUT);
  digitalWrite(SENSOR_LEFT_REAR,HIGH);
  leftBack.init(true);
  leftBack.setTimeout(500);
  leftBack.setAddress(0x30);
  
  pinMode(SENSOR_LEFT_FRONT, INPUT);
  //digitalWrite(SENSOR_LEFT_FRONT,HIGH);
  leftFront.init(true);
  leftFront.setTimeout(500);
  leftFront.setAddress(0x31);
  
  pinMode(SENSOR_FRONT_LEFT, INPUT);
  digitalWrite(SENSOR_FRONT_LEFT,HIGH);
  frontLeft.init(true);
  frontLeft.setTimeout(500);
  frontLeft.setAddress(0x32);
  
  pinMode(SENSOR_FRONT_RIGHT, INPUT);
  digitalWrite(SENSOR_FRONT_RIGHT,HIGH);
  frontRight.init(true);
  frontRight.setTimeout(500);
  frontRight.setAddress(0x33);
  
  pinMode(SENSOR_RIGHT_FRONT, INPUT);
  digitalWrite(SENSOR_RIGHT_FRONT,HIGH);
  rightFront.init(true);
  rightFront.setTimeout(500);
  rightFront.setAddress(0x34);
  
  pinMode(SENSOR_RIGHT_REAR, INPUT);
  digitalWrite(SENSOR_RIGHT_REAR,HIGH);
  rightBack.init(true);
  rightBack.setTimeout(500);
  rightBack.setAddress(0x35);  

  leftBack.startContinuous();
  leftFront.startContinuous();
  frontLeft.startContinuous();
  frontRight.startContinuous();
  rightFront.startContinuous();
  rightBack.startContinuous();
}

void loop() {
  Serial.print("LB - " + String(leftBack.readRangeContinuousMillimeters()) + " | ");
  if (leftBack.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
  Serial.print("LF - " + String(leftFront.readRangeContinuousMillimeters()) + " | ");
  if (leftFront.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
  Serial.print("FL - " + String(frontLeft.readRangeContinuousMillimeters()) + " | ");
  if (frontLeft.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
  Serial.print("FR - " + String(frontRight.readRangeContinuousMillimeters()) + " | ");
  if (frontRight.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
  Serial.print("RF - " + String(rightFront.readRangeContinuousMillimeters()) + " | ");
  if (rightFront.timeoutOccurred()) { Serial.print(" TIMEOUT "); }
  Serial.print("RB - " + String(rightBack.readRangeContinuousMillimeters()) + " | ");
  if (rightBack.timeoutOccurred()) { Serial.print(" TIMEOUT "); }

  Serial.println();
  delay(100);
//  Serial.print(frontRight.readRangeSingleMillimeters());
//  Serial.println();
  newByte = xbeeComm.read();

  if (newByte != -1) {
    switch (newByte) {
      case '\r':
      case '\n':
      case 255:
        break;
        
      case 's':
        Serial.println("Stop command");
        myMotors.driveStop();
        break;

      case 'w':
        Serial.println("Drive forward");
        myMotors.driveForward(DRIVE_SPEED_CRUISE);
        break;

      case 'a':
        Serial.println("Turn left");
        myMotors.turnLeft(TURN_SPEED);
        break;

      case 'd':
        Serial.println("Turn right");
        myMotors.turnRight(TURN_SPEED);
        break;

      case 'q':
        Serial.println("Pivot left");
        myMotors.pivotLeft(TURN_SPEED);
        break;

      case 'e':
        Serial.println("Pivot right");
        myMotors.pivotRight(TURN_SPEED);
        break;

      case 'x':
        Serial.println("Drive reverse");
        myMotors.driveReverse(DRIVE_REVERSE_SPEED);
        break;

      case 'm':
        Serial.println("Measurement");
        byte distances[3];
        //distances[0] = (byte) pingFront.getDistanceInCm();
        
        xbeeComm.write(distances, sizeof(distances)/sizeof(byte));

        break;

      case 'p':
      case ' ':
        Serial.println("Attack!");
        mallet.write(MALLET_EXTEND_POS);
        delay(MALLET_SNAP_DURATION);
        mallet.write(MALLET_SNAP_POS);
        mallet.write(MALLET_RETRACT_POS);
        delay(MALLET_RETRACT_DURATION);
        //myMallet.swing();
        //myMallet.retract();
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(newByte);
    }
  }
}
