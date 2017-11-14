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
int leftSpeed = 150;
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

void turnAround () {
  return;
}

void turnRight () {
  return;
}

void turnLeft () {
  
}

void driveForward(){
  int newLeftSpeed;
  int newRightSpeed;
  int rightFrontSensorReading;
  int leftFrontSensorReading; 

  // todo: this will not catch right turns that are available when the front is open
  bool blockedFront = false;
  while (!blockedFront) {
    rightFrontSensorReading = rightFront.readRangeSingleMillimeters();
    leftFrontSensorReading = leftFront.readRangeSingleMillimeters();
    
    if (rightFrontSensorReading < 175){
      newRightSpeed = 225;
    } else {
      newRightSpeed = 100;
    }
    if (leftFrontSensorReading < 175){
      newLeftSpeed = 225;
    } else {
      newLeftSpeed = 100;
    }
    myMotors.driveForward(newRightSpeed, newLeftSpeed);

    blockedFront = (frontRight.readRangeSingleMillimeters() < 200) && (frontLeft.readRangeSingleMillimeters() < 200);
  }

  // todo: figure these out at run time.
  bool blockedRight = true;
  bool blockedLeft = true;
  if (blockedFront && blockedRight && blockedLeft) {
    turnAround();
  } else if (blockedFront && blockedRight && !blockedLeft) {
    turnLeft();
  } else if (blockedFront && !blockedRight && blockedLeft) {
    turnRight();
  } else if (blockedFront && !blockedRight && !blockedLeft) {
    turnRight();
  } else {
    // bail.
  }
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
        xbeeComm.println("Drive forward");
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
        Serial.println("Measurement");
        byte distances[3];
        //distances[0] = (byte) pingFront.getDistanceInCm();
        
        xbeeComm.write(distances, sizeof(distances)/sizeof(byte));
        break;
        
      default:
        Serial.print("Unknown command: ");
        Serial.println(newByte);
    }
  }
}
