#ifndef _DRIVE_MOTORS_H_
#define _DRIVE_MOTORS_H_

#define DEBUG             false
#define DEBUG_SERIAL_PORT (Serial)

#define DRIVE_SPEED_CRUISE  128
#define DRIVE_REVERSE_SPEED 128
#define TURN_SPEED          128

#include "Arduino.h"

class DriveMotors {
  private:
    byte leftMotorCh0Pin;
    byte leftMotorCh1Pin;
    byte rightMotorCh0Pin;
    byte rightMotorCh1Pin;
    
  public:
    DriveMotors(const byte lm0, const byte lm1, const byte rm0, const byte rm1);
    void driveForward(const byte speed);
    void driveReverse(const byte speed);
    void driveStop(void);
    void pivotLeft(const byte speed);
    void pivotRight(const byte speed);
    void turnLeft(const byte speed);
    void turnRight(const byte speed);
};

#endif

