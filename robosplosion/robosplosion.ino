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
int leftSpeed = 25;
int rightSpeed = 25;

bool forward = true;
double targetDist;
double LeftDist;
double RightDist;

void setup() {
    Serial.begin(9600);
    xbeeComm.begin(9600);
    sensors.reset();

    pinMode(SENSOR_LEFT_REAR, INPUT);
    digitalWrite(SENSOR_LEFT_REAR, HIGH);
    leftBack.init(true);
    leftBack.setTimeout(500);
    leftBack.setAddress(0x30);

    pinMode(SENSOR_LEFT_FRONT, INPUT);
    digitalWrite(SENSOR_LEFT_FRONT, HIGH);
    leftFront.init(true);
    leftFront.setTimeout(500);
    leftFront.setAddress(0x31);

    pinMode(SENSOR_FRONT_LEFT, INPUT);
    digitalWrite(SENSOR_FRONT_LEFT, HIGH);
    frontLeft.init(true);
    frontLeft.setTimeout(500);
    frontLeft.setAddress(0x32);

    pinMode(SENSOR_FRONT_RIGHT, INPUT);
    digitalWrite(SENSOR_FRONT_RIGHT, HIGH);
    frontRight.init(true);
    frontRight.setTimeout(500);
    frontRight.setAddress(0x33);

    pinMode(SENSOR_RIGHT_FRONT, INPUT);
    digitalWrite(SENSOR_RIGHT_FRONT, HIGH);
    rightFront.init(true);
    rightFront.setTimeout(500);
    rightFront.setAddress(0x34);

    pinMode(SENSOR_RIGHT_REAR, INPUT);
    digitalWrite(SENSOR_RIGHT_REAR, HIGH);
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

void printDistance () {
    // todo: read these at run time, don't save in memory.
    int sensDriverFront = leftFront.readRangeSingleMillimeters();
    int sensDriverBack = leftBack.readRangeSingleMillimeters();
    int sensPassFront = rightFront.readRangeSingleMillimeters();
    int sensPassBack = rightBack.readRangeSingleMillimeters();
    int sensFrontLeft = frontLeft.readRangeSingleMillimeters();
    int sensFrontRight = frontRight.readRangeSingleMillimeters();

    xbeeComm.println(
            (String) "LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " +
            sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
}

void ryansAlgorithm () {
    // todo: read these at run time, don't save in memory.
    int sensDriverFront = leftFront.readRangeSingleMillimeters();
    int sensDriverBack = leftBack.readRangeSingleMillimeters();
    int sensPassFront = rightFront.readRangeSingleMillimeters();
    int sensPassBack = rightBack.readRangeSingleMillimeters();
    int sensFrontLeft = frontLeft.readRangeSingleMillimeters();
    int sensFrontRight = frontRight.readRangeSingleMillimeters();

    forward = true;
    xbeeComm.println(
            (String) "LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " +
            sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
    int baseSpeed = 75;
    int leftSpeed = 0;
    int rightSpeed = 0;

    if (sensFrontLeft < 50 || sensFrontRight < 50) {
        xbeeComm.println((String) "To close to the front (FL " + sensFrontLeft + " | FR " + sensFrontRight + ")");
        myMotors.driveStop();
        forward = false;
        leftSpeed = 50;
        rightSpeed = 50;
    } else if ((sensDriverFront < 300) && (sensPassFront < 300)) {
        int totalSpace = sensDriverFront + sensPassFront;
        targetDist = (float) totalSpace / 2;
        xbeeComm.println(
                (String) "Using both front sensors (TotalSpace " + totalSpace + " | TargetDist " + targetDist +
                ")");
        leftSpeed = ((float) targetDist / (float) sensDriverFront) * baseSpeed;
        rightSpeed = ((float) targetDist / (float) sensPassFront) * baseSpeed;
        if (leftSpeed < baseSpeed) {
            leftSpeed = baseSpeed;
        }
        if (rightSpeed < baseSpeed) {
            rightSpeed = baseSpeed;
        }
    } else if ((sensDriverBack < 300) && (sensPassBack < 300)) {
        int totalSpace = sensDriverBack + sensPassBack;
        targetDist = (float) totalSpace / 2;
        xbeeComm.println(
                (String) "Using both back sensors (TotalSpace " + totalSpace + " | TargetDist " + targetDist + ")");
        leftSpeed = ((float) targetDist / (float) sensDriverBack) * baseSpeed;
        rightSpeed = ((float) targetDist / (float) sensPassBack) * baseSpeed;
        if (leftSpeed < baseSpeed) {
            leftSpeed = baseSpeed;
        }
        if (rightSpeed < baseSpeed) {
            rightSpeed = baseSpeed;
        }
    } else if (sensDriverFront < 300 || sensDriverBack < 300) {
        xbeeComm.println("No right wall, turning right");
        leftSpeed = baseSpeed;
        rightSpeed = 0;
    } else if (sensPassFront < 300 || sensPassBack < 300) {
        xbeeComm.println("No left wall, turning left");
        rightSpeed = baseSpeed;
        leftSpeed = 0;
    } else if (((sensFrontLeft + sensFrontRight) / 2) > 150) {
        xbeeComm.println("There are no walls");
        myMotors.driveStop();
    } else {
        xbeeComm.println("Where am I???");
        myMotors.driveStop();
    }


    if (rightSpeed > 250 || leftSpeed > 250) {
        float reduc = 0;
        if (rightSpeed > 250) {
            reduc = (float) 250 / (float) rightSpeed;
        } else {
            reduc = (float) 250 / (float) leftSpeed;
        }
        xbeeComm.println(
                (String) "Reducing speed (RS " + rightSpeed + " | LS " + leftSpeed + " | Reduction " + reduc + ")");
        rightSpeed = reduc * rightSpeed;
        leftSpeed = reduc * leftSpeed;
    }

    if (forward) {
        xbeeComm.println((String) "LeftSpeed: " + leftSpeed + " | RightSpeed: " + rightSpeed);
        myMotors.driveForward(rightSpeed, leftSpeed);
    } else {
        xbeeComm.println((String) "LeftSpeed: " + -leftSpeed + " | RightSpeed: " + -rightSpeed);
        myMotors.driveReverse(rightSpeed, leftSpeed);
    }

}

bool displayDistances = false;
bool stopped = true;
void readUserInput() {
    int newByte = xbeeComm.read();
    if (newByte != -1) {
        switch (newByte) {
            case '\r':
            case '\n':
            case 255:
                break;

            case 's':
                Serial.println("Stop command");
                stopped = true;
                break;
            case 'w':
                xbeeComm.println("Drive forward");
                stopped = false;
                break;
            case ' ':
                xbeeComm.println("Flip distance display.");
                displayDistances = !displayDistances;
                break;
            default:
                Serial.print("Unknown command: ");
                Serial.println(newByte);
        }
    }
}

void loop() {
    readUserInput();

    if (displayDistances) {
        printDistance();
    } else if (stopped) {
        myMotors.driveStop();
    } else if (!stopped) {
        ryansAlgorithm();
    }
}

