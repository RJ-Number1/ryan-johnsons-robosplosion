#include <PID_v1.h>
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
boolean dist = false;
boolean test = false;
bool forward = true;
double targetDist;
double LeftDist;
double RightDist;

//PID variables
//double leftWheelInput, rightWheelInput, leftWheelOutput, rightWheelOutput;
//float Kp=1;
//float Ki=0;
//float Kd=0;
//PID leftWheelPID(&leftWheelInput, &leftWheelOutput, &targetDist, Kp, Ki, Kd, DIRECT);  
//PID rightWheelPID(&rightWheelInput, &rightWheelOutput, &targetDist, Kp, Ki, Kd, DIRECT);
//const long serialPing = 500;
//unsigned long now = 0;
//unsigned long lastMessage = 0;

byte newByte;
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

//  leftWheelPID.SetMode(AUTOMATIC);
//  leftWheelPID.SetSampleTime(1);
//  rightWheelPID.SetMode(AUTOMATIC);
//  rightWheelPID.SetSampleTime(1);  
//  lastMessage = millis(); 
}


void loop() {
  newByte = xbeeComm.read();
  int sensDriverFront = leftFront.readRangeSingleMillimeters();
  int sensDriverBack = leftBack.readRangeSingleMillimeters();
  int sensPassFront = rightFront.readRangeSingleMillimeters();
  int sensPassBack = rightBack.readRangeSingleMillimeters();
  int sensFrontLeft = frontLeft.readRangeSingleMillimeters();
  int sensFrontRight = frontRight.readRangeSingleMillimeters();
//  now = millis(); 
//  if(now - lastMessage > serialPing) {
//    //Serial.println("ask for new gain");
//    if (Serial.available() > 0) {
//      for (int x = 0; x < 4; x++) {
//        switch (x) {
//        case 0:
//          Kp = Serial.parseFloat();
//          break;
//        case 1:
//          Ki = Serial.parseFloat();
//          break;
//        case 2:
//          Kd = Serial.parseFloat();
//          break;
//        case 3:
//          for (int y = Serial.available(); y == 0; y--) {
//          Serial.read(); //Clear out any residual junk
//          }
//          break;
//        }
//      }
//      Serial.print(" Kp,Ki,Kd = ");
//      Serial.print(Kp);
//      Serial.print(",");
//      Serial.print(Ki);
//      Serial.print(",");
//      Serial.println(Kd); //Let us know what we just received
//      leftWheelPID.SetTunings(Kp, Ki, Kd); //Set the PID gain constants and start running
//      rightWheelPID.SetTunings(Kp, Ki, Kd); //Set the PID gain constants and start running
//     }
//     else {
//      //Serial.println("not available");
//     }
//    lastMessage = now; //update the time stamp
//  }
  
  
 if(run){
  int newLeftSpeed;
  int newRightSpeed;
  
  if (sensPassFront < 175){
    newRightSpeed = 225;
  }
  else {
    newRightSpeed = 100;
  }
  
  if (sensDriverFront  < 175){
    newLeftSpeed = 225;
  }
  else {
    newLeftSpeed = 100;
  }

  boolean followWall = "right";

  int maxWallDist = 170;

  if (sensPassFront < maxWallDist) {
    followWall = "right";
  }
  else if (sensDriverFront < maxWallDist) {
    followWall = "left";
  }

  
  xbeeComm.println((String)"LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
  xbeeComm.println((String)"LS " + newLeftSpeed);
  xbeeComm.println((String)"RS " + newRightSpeed);
  myMotors.driveForward(300, 0);

 }
 else if(dist) {
   xbeeComm.println((String)"LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
 }
 else if (test) {
  forward = true;
  xbeeComm.println((String)"LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
  int baseSpeed = 125;
  int leftSpeed = 0;
  int rightSpeed = 0;

  if (sensFrontLeft < 50 || sensFrontRight < 50) {
    xbeeComm.println((String)"To close to the front (FL " + sensFrontLeft + " | FR " + sensFrontRight + ")");
    myMotors.driveStop();
    forward = false;
    leftSpeed = 50;
    rightSpeed = 50;
  }
  else if ((sensDriverFront < 300) && (sensPassFront < 300)) {
    delay(100);
    int totalSpace = sensDriverFront + sensPassFront;
    targetDist = (float)totalSpace/2; 
    xbeeComm.println((String)"Using both front sensors (TotalSpace " + totalSpace + " | TargetDist " + targetDist + ")");
//    leftWheelInput = (float)sensDriverFront;
//    rightWheelInput = (float)sensPassFront;
//    leftWheelPID.Compute();
//    rightWheelPID.Compute();
//    xbeeComm.println((String)"PID input | Left: " + leftWheelInput + " | Right: " + rightWheelInput);
//    xbeeComm.println((String)"PID output | Left: " + leftWheelOutput + " | Right: " + rightWheelOutput);
//    leftSpeed = 150 + leftWheelOutput;
//    rightSpeed = 150 + rightWheelOutput;
//    leftSpeed = ((float)targetDist / (float)sensDriverFront) * baseSpeed;
//    rightSpeed = ((float)targetDist / (float)sensPassFront) * baseSpeed;
  }
  else if ((sensDriverBack < 300) && (sensPassBack < 300)) {
    int totalSpace = sensDriverBack + sensPassBack;
    targetDist = (float)totalSpace/2;
    xbeeComm.println((String)"Using both back sensors (TotalSpace " + totalSpace + " | TargetDist " + targetDist + ")");
//    leftWheelInput = (float)sensDriverBack;
//    rightWheelInput = (float)sensPassBack;
//    leftWheelPID.Compute();
//    rightWheelPID.Compute();
//    xbeeComm.println((String)"PID input | Left: " + leftWheelInput + " | Right: " + rightWheelInput);
//    xbeeComm.println((String)"PID output | Left: " + leftWheelOutput + " | Right: " + rightWheelOutput);
//    leftSpeed = leftWheelOutput;
//    rightSpeed = rightWheelOutput;
    leftSpeed = ((float)targetDist / (float)sensDriverBack) * baseSpeed;
    rightSpeed = ((float)targetDist / (float)sensPassBack) * baseSpeed;
  }
//  else if ((sensDriverFront < 300 && sensDriverBack < 300) && (sensFrontLeft < 300 && sensFrontRight < 300)) {
//    xbeeComm.println("Turning right");
//    leftSpeed = baseSpeed;
//    rightSpeed = 0;
//  }
//  else if ((sensPassFront < 300 && sensPassBack < 300) && (sensFrontLeft < 300 && sensFrontRight < 300)) {
//    xbeeComm.println("Turning left");
//    rightSpeed = baseSpeed;
//    leftSpeed = 0;
//  }
  else if (sensDriverFront < 300 || sensDriverBack < 300) {
    xbeeComm.println("No right wall, turning right");
    leftSpeed = baseSpeed;
    rightSpeed = 0;
//    leftSpeed = ((float)targetDist / (float)sensDriverFront) * baseSpeed;
//    rightSpeed = (baseSpeed * 2) - leftSpeed;
  }
  else if (sensPassFront < 300 || sensPassBack < 300) {
    xbeeComm.println("No left wall, turning left");
    rightSpeed = baseSpeed;
    leftSpeed = 0;
//    rightSpeed = ((float)targetDist / (float)sensPassFront) * baseSpeed;
//    leftSpeed = (baseSpeed * 2) - rightSpeed;
  }
  else if (((sensFrontLeft + sensFrontRight) / 2) > 150) {
    xbeeComm.println("There are no walls");
    myMotors.driveStop();
  }
  else {
    xbeeComm.println("Where am I???");
    myMotors.driveStop();
  }
  


//  if(rightSpeed > 250 || leftSpeed > 250) {
//    float reduc = 0;
//    if (rightSpeed > 250) {
//      reduc = (float)250/(float)rightSpeed;
//    }
//    else {
//      reduc = (float)250/(float)leftSpeed;
//    }
//    xbeeComm.println((String)"Reducing speed (RS " + rightSpeed + " | LS " + leftSpeed + " | Reduction " + reduc + ")");
//    rightSpeed = reduc * rightSpeed;
//    leftSpeed = reduc * leftSpeed;
//  }

  if(forward) {
    xbeeComm.println((String)"LeftSpeed: " + leftSpeed + " | RightSpeed: " + rightSpeed);
    myMotors.driveForward(rightSpeed, leftSpeed);
  }
  else {
    xbeeComm.println((String)"LeftSpeed: " + -leftSpeed + " | RightSpeed: " + -rightSpeed);
    myMotors.driveReverse(rightSpeed, leftSpeed);
  }

 }

  if (newByte != -1) {
    switch (newByte) {
      case '\r':
      case '\n':
      case 255:
        break;
        
      case 's':
        run=false;
        dist=false;
        test = false;
        Serial.println("Stop command");
        myMotors.driveStop();
        break;

      case 'w':
        xbeeComm.println("Drive forward");
        run = true;
        dist = false;
        test = false;
        //myMotors.driveForward(200, 200);
        break;
      case ' ':
        xbeeComm.println("Print Distance");
        run = false;
        dist = true;
        test = false;
        //myMotors.driveForward(200, 200);
        break;
      case 't':
        xbeeComm.println("Print Distance");
        run = false;
        dist = false;
        test = true;
        //myMotors.driveForward(200, 200);
        break;
//      case 'a':
//        Serial.println("Turn left");
//        myMotors.turnLeft(TURN_SPEED);
//        break;
//
//      case 'd':
//        Serial.println("Turn right");
//        myMotors.turnRight(TURN_SPEED);
//        break;
//
//      case 'q':
//        Serial.println("Pivot left");
//        myMotors.pivotLeft(TURN_SPEED);
//        break;
//
//      case 'e':
//        Serial.println("Pivot right");
//        myMotors.pivotRight(TURN_SPEED);
//        break;
//
//      case 'x':
//        Serial.println("Drive reverse");
//        myMotors.driveReverse(DRIVE_REVERSE_SPEED);
//        break;
//
//      case 'm':
//        Serial.println("Measurement");
//        byte distances[3];
//        //distances[0] = (byte) pingFront.getDistanceInCm();
//        
//        xbeeComm.write(distances, sizeof(distances)/sizeof(byte));
//
//        break;
//
//     
//
//      default:
//        Serial.print("Unknown command: ");
//        Serial.println(newByte);
    }
  }
}

