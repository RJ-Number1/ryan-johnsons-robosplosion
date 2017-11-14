#include "DriveMotors.h"
#include "HCSR04.h"
#include "Mallet.h"
#include "Pins.h"
#include "Sensors.h"
#include <Servo.h>
#include <SoftwareSerial.h>
#include <VL53L0X.h>
#define xbeeComm Serial3
VL53L0X leftBack;
VL53L0X leftFront;
VL53L0X frontLeft;
VL53L0X frontRight;
VL53L0X rightFront;
VL53L0X rightBack;
DriveMotors myMotors(MOTOR_LEFT_CH0, MOTOR_LEFT_CH1,
                    MOTOR_RIGHT_CH0, MOTOR_RIGHT_CH1);
Sensors sensors;                    
int leftSpeed = 130;
int rightSpeed = 150;

boolean run = false;
int newByte;
void setup() {
  Serial.begin(9600);
  xbeeComm.begin(9600);
  sensors.reset();
  pinMode(SENSOR_LEFT_REAR, INPUT);
  digitalWrite(SENSOR_LEFT_REAR,HIGH);
  leftBack.init(true);
  leftBack.setTimeout(500);
  leftBack.setAddress(0x30);
  pinMode(SENSOR_LEFT_FRONT, INPUT);
  digitalWrite(SENSOR_LEFT_FRONT,HIGH);
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

bool loggingOn = false;
void log (String message) {
  if (loggingOn) {
    xbeeComm.println(message);
  }
}

void turnAround () {
  while (frontRight.readRangeContinuousMillimeters() < 350){
    myMotors.pivotLeft(TURN_SPEED);
  }
  
  myMotors.driveStop();
}

void turnRight () {
  int rightFrontBeforeTurn;
  rightFrontBeforeTurn = rightFront.readRangeSingleMillimeters();
  while ((frontLeft.readRangeContinuousMillimeters() < 350) && (frontLeft.readRangeContinuousMillimeters() < (rightFrontBeforeTurn))) {
    myMotors.pivotRight(TURN_SPEED);
  }
  myMotors.driveStop();

}

void turnLeft () {
  int leftFrontBeforeTurn;
  leftFrontBeforeTurn = leftFront.readRangeSingleMillimeters();
  while ( (frontRight.readRangeContinuousMillimeters() < 350) && (frontRight.readRangeContinuousMillimeters() < (leftFrontBeforeTurn))){
    myMotors.pivotLeft(TURN_SPEED);
  }
  
  myMotors.driveStop();
}


void driveForward(){
  int newLeftSpeed;
  int newRightSpeed;
  int rightFrontSensorReading;
  int leftFrontSensorReading; 

  bool blockedFront = false;
  bool blockedRight = true;
  while (!blockedFront && blockedRight) {
    if (rightFront.readRangeContinuousMillimeters() < 100){
      newRightSpeed = 185;
    } else {
      newRightSpeed = 100;
    }
    if (leftFront.readRangeContinuousMillimeters() < 100){
      newLeftSpeed = 165;
    } else {
      newLeftSpeed = 100;
    }
    myMotors.driveForward(newRightSpeed, newLeftSpeed);
 //front left sensor is not working
    blockedFront = ((frontRight.readRangeContinuousMillimeters() < 100) && (frontLeft.readRangeContinuousMillimeters() < 100));
    blockedRight = ((rightBack.readRangeContinuousMillimeters() < 200));
  }
  myMotors.driveStop();
  // todo: figure these out at run time.
  delay(1000);

  if ( (frontRight.readRangeContinuousMillimeters() < 125) && (frontLeft.readRangeContinuousMillimeters() < 125) && (rightBack.readRangeContinuousMillimeters() < 200) && (leftBack.readRangeContinuousMillimeters() < 200) ) {
    log("Blocked everywhere.");
    turnAround();
  } 
 else if ((frontRight.readRangeContinuousMillimeters() < 100) && (frontLeft.readRangeContinuousMillimeters() < 100) && (rightBack.readRangeContinuousMillimeters() > 350) && (leftBack.readRangeContinuousMillimeters() > 350)) {
   log("Blocked only in the Front.");
   turnRight();
  } 
  else if ((frontRight.readRangeContinuousMillimeters() < 100) && (frontLeft.readRangeContinuousMillimeters() < 100) && (rightBack.readRangeContinuousMillimeters() < 350) && (leftBack.readRangeContinuousMillimeters() > 350)) {
    log("Blocked Front and Right.");
    turnLeft();
  }  
  else if ((frontRight.readRangeContinuousMillimeters() < 100) && (frontLeft.readRangeContinuousMillimeters() < 100) && (rightBack.readRangeContinuousMillimeters() > 350) && (leftBack.readRangeContinuousMillimeters() < 350)) {
    log("Blocked Front and Left.");
    turnRight();
   }
  else {
    myMotors.driveStop();
  }
}

void printMeasurements () {
  Serial.println("Measurement");
  xbeeComm.print("LB - " + String(leftBack.readRangeContinuousMillimeters()) + " | ");
  if (leftBack.timeoutOccurred()) {
    Serial.print(" TIMEOUT ");
  }

  xbeeComm.print("LF - " + String(leftFront.readRangeContinuousMillimeters()) + " | ");
  if (leftFront.timeoutOccurred()) {
    Serial.print(" TIMEOUT ");
  }

  xbeeComm.print("FL - " + String(frontLeft.readRangeContinuousMillimeters()) + " | ");
  if (frontLeft.timeoutOccurred()) {
    Serial.print(" TIMEOUT ");
  }

  xbeeComm.print("FR - " + String(frontRight.readRangeContinuousMillimeters()) + " | ");
  if (frontRight.timeoutOccurred()) {
    Serial.print(" TIMEOUT ");
  }

  xbeeComm.print("LB - " + String(leftBack.readRangeContinuousMillimeters()) + " | ");
  if (rightFront.timeoutOccurred()) {
    Serial.print(" TIMEOUT ");
  }

  xbeeComm.print("RF - " + String(rightFront.readRangeContinuousMillimeters()) + " | ");
  if (rightBack.timeoutOccurred()) {
    Serial.print(" TIMEOUT ");
  }

  log("");
  delay(100);
}

void loop() {
  newByte = xbeeComm.read();
  if (newByte != -1) {
    switch (newByte) {
      case '\r':
      case '\n':
      case 255:
        break;
        
      case 's':
        run=false;
        Serial.println("Stop command");
        myMotors.driveStop();
        break;

      case 'w':
        log("Drive forward");
        driveForward();
        myMotors.driveStop();
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
        printMeasurements();
        break;
        
      default:
        Serial.print("Unknown command: ");
        Serial.println(newByte);
    }
  }
}
