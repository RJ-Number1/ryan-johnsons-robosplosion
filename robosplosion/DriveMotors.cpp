#include "DriveMotors.h"

DriveMotors::DriveMotors(const byte lm0, const byte lm1, const byte rm0, const byte rm1) {
  leftMotorCh0Pin = lm0;
  leftMotorCh1Pin = lm1;
  rightMotorCh0Pin = rm0;
  rightMotorCh1Pin = rm1;

  pinMode(leftMotorCh0Pin, OUTPUT);
  pinMode(leftMotorCh1Pin, OUTPUT);
  pinMode(rightMotorCh0Pin, OUTPUT);
  pinMode(rightMotorCh1Pin, OUTPUT);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.begin(9600);
    DEBUG_SERIAL_PORT.println("[*] Initialized drive motors");
    DEBUG_SERIAL_PORT.print("    leftMotorCh0Pin: ");
    DEBUG_SERIAL_PORT.println(leftMotorCh0Pin);
    DEBUG_SERIAL_PORT.print("    leftMotorCh1Pin: ");
    DEBUG_SERIAL_PORT.println(leftMotorCh1Pin);
    DEBUG_SERIAL_PORT.print("    rightMotorCh0Pin: ");
    DEBUG_SERIAL_PORT.println(rightMotorCh0Pin);
    DEBUG_SERIAL_PORT.print("    rightMotorCh1Pin: ");
    DEBUG_SERIAL_PORT.println(rightMotorCh1Pin);
  }
}

void DriveMotors::driveForward(const byte speed) {
  analogWrite(leftMotorCh0Pin, speed);
  analogWrite(leftMotorCh1Pin, 0);

  analogWrite(rightMotorCh0Pin, speed);
  analogWrite(rightMotorCh1Pin, 0);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] Driving forward (speed: ");
    DEBUG_SERIAL_PORT.print(speed);
    DEBUG_SERIAL_PORT.println(")");
  }
}

void DriveMotors::driveReverse(const byte speed) {
  analogWrite(leftMotorCh0Pin, 0);
  analogWrite(leftMotorCh1Pin, speed);

  analogWrite(rightMotorCh0Pin, 0);
  analogWrite(rightMotorCh1Pin, speed);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] Driving in reverse (speed: ");
    DEBUG_SERIAL_PORT.print(speed);
    DEBUG_SERIAL_PORT.println(")");
  }
}

void DriveMotors::driveStop(void) {
  analogWrite(leftMotorCh0Pin, 0);
  analogWrite(leftMotorCh1Pin, 0);
  analogWrite(rightMotorCh0Pin, 0);
  analogWrite(rightMotorCh1Pin, 0);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] Stopping");
  }
}

void DriveMotors::pivotLeft(const byte speed) {
  analogWrite(leftMotorCh0Pin, 0);
  analogWrite(leftMotorCh1Pin, speed);

  analogWrite(rightMotorCh0Pin, speed);
  analogWrite(rightMotorCh1Pin, 0);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] Pivoting left (speed: ");
    DEBUG_SERIAL_PORT.print(speed);
    DEBUG_SERIAL_PORT.println(")");
  }
}

void DriveMotors::pivotRight(const byte speed) {
  analogWrite(leftMotorCh0Pin, speed);
  analogWrite(leftMotorCh1Pin, 0);

  analogWrite(rightMotorCh0Pin, 0);
  analogWrite(rightMotorCh1Pin, speed);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] Pivoting right (speed: ");
    DEBUG_SERIAL_PORT.print(speed);
    DEBUG_SERIAL_PORT.println(")");
  }
}

void DriveMotors::turnLeft(const byte speed) {
  analogWrite(leftMotorCh0Pin, 0);
  analogWrite(leftMotorCh1Pin, 0);

  analogWrite(rightMotorCh0Pin, speed);
  analogWrite(rightMotorCh1Pin, 0);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] Turning left (speed: ");
    DEBUG_SERIAL_PORT.print(speed);
    DEBUG_SERIAL_PORT.println(")");
  }
}

void DriveMotors::turnRight(const byte speed) {
  analogWrite(leftMotorCh0Pin, speed);
  analogWrite(leftMotorCh1Pin, 0);

  analogWrite(rightMotorCh0Pin, 0);
  analogWrite(rightMotorCh1Pin, 0);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] Turning right (speed: ");
    DEBUG_SERIAL_PORT.print(speed);
    DEBUG_SERIAL_PORT.println(")");
  }
}
