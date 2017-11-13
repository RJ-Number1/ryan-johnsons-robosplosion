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

//PID variables
double leftWheelInput, rightWheelInput, leftWheelOutput, rightWheelOutput, leftDist, rightDist;
float Kp=5;
float Ki=0;
float Kd=0;
PID leftWheelPID(&leftWheelInput, &leftWheelOutput, &leftDist, Kp, Ki, Kd, DIRECT);  
PID rightWheelPID(&rightWheelInput, &rightWheelOutput, &rightDist, Kp, Ki, Kd, DIRECT);
const long serialPing = 500;
unsigned long now = 0;
unsigned long lastMessage = 0;

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

  leftWheelPID.SetMode(AUTOMATIC);
  leftWheelPID.SetSampleTime(1);
  leftWheelPID.SetOutputLimits (-50,50);
  rightWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetSampleTime(1);  
  rightWheelPID.SetOutputLimits (-50,50);
  
  lastMessage = millis(); 
}


void loop() {
  newByte = xbeeComm.read();
  int sensDriverFront = leftFront.readRangeSingleMillimeters();
  int sensDriverBack = leftBack.readRangeSingleMillimeters();
  int sensPassFront = rightFront.readRangeSingleMillimeters();
  int sensPassBack = rightBack.readRangeSingleMillimeters();
  int sensFrontLeft = frontLeft.readRangeSingleMillimeters();
  int sensFrontRight = frontRight.readRangeSingleMillimeters();
  now = millis(); 
  if(now - lastMessage > serialPing) {
    //Serial.println("ask for new gain");
    if (Serial.available() > 0) {
      for (int x = 0; x < 4; x++) {
        switch (x) {
        case 0:
          Kp = Serial.parseFloat();
          break;
        case 1:
          Ki = Serial.parseFloat();
          break;
        case 2:
          Kd = Serial.parseFloat();
          break;
        case 3:
          for (int y = Serial.available(); y == 0; y--) {
          Serial.read(); //Clear out any residual junk
          }
          break;
        }
      }
      Serial.print(" Kp,Ki,Kd = ");
      Serial.print(Kp);
      Serial.print(",");
      Serial.print(Ki);
      Serial.print(",");
      Serial.println(Kd); //Let us know what we just received
      leftWheelPID.SetTunings(Kp, Ki, Kd); //Set the PID gain constants and start running
      rightWheelPID.SetTunings(Kp, Ki, Kd); //Set the PID gain constants and start running
     }
     else {
      //Serial.println("not available");
     }
    lastMessage = now; //update the time stamp
  }
  
  if(dist) {
    xbeeComm.println((String)"LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
  }
  else if (test) {
    leftSpeed = -50;
    rightSpeed = 0;
    xbeeComm.println((String)"LeftSpeed: " + leftSpeed + " | RightSpeed: " + rightSpeed);
    myMotors.drive(leftSpeed, rightSpeed);
  }
  else if (run) {
    xbeeComm.println((String)"LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
    
    if (sensFrontLeft < 50 || sensFrontRight < 50) {
      xbeeComm.println((String)"To close to the front (FL " + sensFrontLeft + " | FR " + sensFrontRight + ")");
      myMotors.driveStop();
      leftSpeed = -50;
      rightSpeed = -50;
    }
    else if (sensDriverFront < 300 && sensPassFront < 300 && sensDriverBack < 300 && sensPassBack < 300) {
      int totalSpace = sensDriverFront + sensPassFront;
      targetDist = (float)totalSpace/2; 
      leftDist = targetDist;
      rightDist = targetDist;
      xbeeComm.println((String)"Using all four side sensors (TotalSpace " + totalSpace + " | TargetDist " + targetDist + ")");
      leftWheelInput = ((float)sensDriverFront + (float)sensDriverBack)/2;
      rightWheelInput = ((float)sensPassFront + (float)sensPassBack)/2;
      leftWheelPID.Compute();
      rightWheelPID.Compute();
      xbeeComm.println((String)"PID input | Left: " + leftWheelInput + " | Right: " + rightWheelInput);
      xbeeComm.println((String)"PID output | Left: " + leftWheelOutput + " | Right: " + rightWheelOutput);
      leftSpeed = 100+leftWheelOutput;
      rightSpeed = 100+rightWheelOutput;
    }
    else if (sensDriverFront < 300 && sensPassFront < 300) {
      int totalSpace = sensDriverFront + sensPassFront;
      targetDist = (float)totalSpace/2; 
      leftDist = targetDist;
      rightDist = targetDist;
      xbeeComm.println((String)"Using both front sensors (TotalSpace " + totalSpace + " | TargetDist " + targetDist + ")");
      leftWheelInput = (float)sensDriverFront;
      rightWheelInput = (float)sensPassFront;
      leftWheelPID.Compute();
      rightWheelPID.Compute();
      xbeeComm.println((String)"PID input | Left: " + leftWheelInput + " | Right: " + rightWheelInput);
      xbeeComm.println((String)"PID output | Left: " + leftWheelOutput + " | Right: " + rightWheelOutput);
      leftSpeed = 100+leftWheelOutput;
      rightSpeed = 100+rightWheelOutput;
    }
    else if (sensDriverBack < 300 && sensPassBack < 300) {
      int totalSpace = sensDriverBack + sensPassBack;
      targetDist = (float)totalSpace/2;
      leftDist = targetDist;
      rightDist = targetDist;
      xbeeComm.println((String)"Using both back sensors (TotalSpace " + totalSpace + " | TargetDistLeft: " + leftDist + " | TargetDistRight: " + rightDist + ")");
      leftWheelInput = (float)sensDriverBack;
      rightWheelInput = (float)sensPassBack;
      leftWheelPID.Compute();
      rightWheelPID.Compute();
      xbeeComm.println((String)"PID input | Left: " + leftWheelInput + " | Right: " + rightWheelInput);
      xbeeComm.println((String)"PID output | Left: " + leftWheelOutput + " | Right: " + rightWheelOutput);
      leftSpeed = 100+leftWheelOutput;
      rightSpeed = 100+rightWheelOutput;
    }
    else if (((sensFrontLeft + sensFrontRight) / 2) > 150) {
      xbeeComm.println("There are no walls");
      myMotors.driveStop();
    }
    else {
      xbeeComm.println("Where am I???");
      myMotors.driveStop();
    }
  
    myMotors.drive(leftSpeed, rightSpeed);
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
        test=false;
        Serial.println("Stop command");
        myMotors.driveStop();
        break;
      case 'w':
        xbeeComm.println("Start Driving");
        run = true;
        dist = false;
        test=false;
        break;
      case 't':
        xbeeComm.println("Test Motors");
        run = false;
        dist = false;
        test=true;
        break;
      case ' ':
        xbeeComm.println("Print Distance");
        run = false;
        dist = true;
        break;
      default:
        Serial.print("Unknown command: ");
        Serial.println(newByte);
    }
  }
}

