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
int sensDriverFront, sensDriverBack, sensPassFront, sensPassBack, sensFrontLeft, sensFrontRight;

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

bool logging = false;
void log (String output) {
    if (logging) {
        xbeeComm.println(output);
    }
}

void printDistance () {
    log((String) "LB " + leftBack.readRangeSingleMillimeters() +
                " | LF " + leftFront.readRangeSingleMillimeters() +
                " | FL " + frontLeft.readRangeSingleMillimeters() +
                " | FR " + frontRight.readRangeSingleMillimeters() +
                " | RF " + rightFront.readRangeSingleMillimeters() +
                " | RB " + rightBack.readRangeSingleMillimeters());
}

void driveStraight(int driverSide, int passengerSide) {
    int totalSpace = driverSide + passengerSide;
    int baseSpeed = 75;
    int leftSpeed = 0;
    int rightSpeed = 0;

    targetDist = (float) totalSpace / 2;
    leftSpeed = ((float) targetDist / (float) driverSide) * baseSpeed;
    rightSpeed = ((float) targetDist / (float) passengerSide) * baseSpeed;
    if (leftSpeed < baseSpeed) {
        leftSpeed = baseSpeed;
    }
    if (rightSpeed < baseSpeed) {
        rightSpeed = baseSpeed;
    }

    log((String) "LeftSpeed: " + leftSpeed + " | RightSpeed: " + rightSpeed);

    myMotors.drive(leftSpeed, rightSpeed);
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
                log("Drive forward");
                stopped = false;
                break;
            case ' ':
                log("Flip distance display.");
                displayDistances = !displayDistances;
                break;
            case 'l':
                xbeeComm.println("Changing logging to " + !logging);
                logging = !logging;
            default:
                Serial.print("Unknown command: ");
                Serial.println(newByte);
        }
    }
}
void readSensorData() {
    sensDriverFront = leftFront.readRangeSingleMillimeters();
    sensDriverBack = leftBack.readRangeSingleMillimeters();
    sensPassFront = rightFront.readRangeSingleMillimeters();
    sensPassBack = rightBack.readRangeSingleMillimeters();
    sensFrontLeft = frontLeft.readRangeSingleMillimeters();
    sensFrontRight = frontRight.readRangeSingleMillimeters();
}

void pivotRobot (int speed) {
    log((String) "LeftSpeed: " + speed / 2 + " | RightSpeed: " + -speed);
    myMotors.drive(speed / 2, -speed);
}

void turnAround () {
    readUserInput();
    readSensorData();

    while (wallInFront && !stopped) {
        emergencyBackup = sensFrontLeft < 50 || sensFrontRight < 50;
        wallInFront = sensFrontLeft < 300 && sensFrontRight < 300;
        if (emergencyBackup) {
            log((String) "LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront);
            driveBackwards(50);
        }
        else if (sensFrontLeft > 100 && sensFrontRight > 100) {
            driveStraight(75, 50);
        }
        else {
            log((String) "Pivoting (FL " + sensFrontLeft + " | FR " + sensFrontRight + ")");
            pivotRobot(50);
        }
    }
}

void ryansAlgorithm () {
    readSensorData();

    forward = true;
    log((String) "LB " + sensDriverBack + " | LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront + " | RB " + sensPassBack);
    int baseSpeed = 75;
    int leftSpeed = 0;
    int rightSpeed = 0;

    bool emergencyBackup = sensFrontLeft < 75 || sensFrontRight < 75;
    bool wallInFront = sensFrontLeft < 200 && sensFrontRight < 200;
    bool wallsOnRightAndLeftFront = (sensDriverFront < 300) && (sensPassFront < 300);
    bool wallsOnRightAndLeftBack = (sensDriverBack < 300) && (sensPassBack < 300);
    bool wallOnLeft = sensDriverFront < 300 || sensDriverBack < 300;
    bool wallOnRight = sensPassFront < 300 || sensPassBack < 300;

    if (emergencyBackup) {
        // panic, back up, then resume...
        stopDriving();
        log((String) "LF " + sensDriverFront + " | FL " + sensFrontLeft + " | FR " + sensFrontRight + " | RF " + sensPassFront);
        driveBackwards(50);
    }
    else if (wallInFront && wallsOnRightAndLeftFront) {
        // This is when we need to pivot out of the dead end
        turnAround();
    }
    else if (wallsOnRightAndLeftFront) {
        log((String) "Using both front sensors");
        driveStraight(sensDriverFront, sensPassFront);
    } else if (wallsOnRightAndLeftBack) {
        log((String) "Using both rear sensors");
        driveStraight(sensDriverBack, sensPassBack);
    } else if (wallOnLeft) {
        log("No right wall, turning right");
        leftSpeed = baseSpeed;
        rightSpeed = 0;

        if (rightSpeed > 250 || leftSpeed > 250) {
            float reduce = 0;
            if (rightSpeed > 250) {
                reduce = (float) 250 / (float) rightSpeed;
            } else {
                reduce = (float) 250 / (float) leftSpeed;
            }
            log((String) "Reducing speed (RS " + rightSpeed + " | LS " + leftSpeed + " | Reduction " + reduce + ")");
            rightSpeed = reduce * rightSpeed;
            leftSpeed = reduce * leftSpeed;
        }

        log((String) "LeftSpeed: " + leftSpeed + " | RightSpeed: " + rightSpeed);
        myMotors.drive(leftSpeed, rightSpeed);
    } else if (wallOnRight) { // falling through implies no wall on left
        log("No left wall, turning left");
        rightSpeed = baseSpeed;
        leftSpeed = 0;

        if (rightSpeed > 250 || leftSpeed > 250) {
            float reduce = 0;
            if (rightSpeed > 250) {
                reduce = (float) 250 / (float) rightSpeed;
            } else {
                reduce = (float) 250 / (float) leftSpeed;
            }
            log(
                    (String) "Reducing speed (RS " + rightSpeed + " | LS " + leftSpeed + " | Reduction " + reduce + ")");
            rightSpeed = reduce * rightSpeed;
            leftSpeed = reduce * leftSpeed;
        }

        log((String) "LeftSpeed: " + leftSpeed + " | RightSpeed: " + rightSpeed);
        myMotors.drive(leftSpeed, rightSpeed);
    } else {
        log("Where am I???");
        myMotors.driveStop();
    }
}

void loop() {
    readUserInput();

    if (displayDistances) {
        printDistance();
    } else if (stopped) {
        myMotors.driveStop();
    } else if (!stopped) {
//        ryansAlgorithm();
        driveStraight(sensFrontLeft, sensFrontRight);
    }

}

