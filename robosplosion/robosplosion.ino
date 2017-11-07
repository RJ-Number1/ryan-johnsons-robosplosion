#include <SoftwareSerial.h>
#include "DriveMotors.h"
#include "HCSR04.h"
#include "Mallet.h"
#include "Pins.h"
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define xbeeComm Serial3
DriveMotors myMotors(MOTOR_LEFT_CH0, MOTOR_LEFT_CH1,
                    MOTOR_RIGHT_CH0, MOTOR_RIGHT_CH1);
Servo mallet;
//Mallet myMallet(&mallet);

byte newByte;

void setup() {
  digitalWrite(SENSOR_LEFT_REAR,LOW);
  digitalWrite(SENSOR_LEFT_FRONT,LOW);
  digitalWrite(SENSOR_FRONT_LEFT,LOW);
  digitalWrite(SENSOR_FRONT_RIGHT,LOW);
  digitalWrite(SENSOR_LEFT_FRONT,LOW);
  digitalWrite(SENSOR_LEFT_REAR,LOW);
  delay(10);
  digitalWrite(SENSOR_LEFT_REAR,HIGH);
  lox.begin(0x30);
  digitalWrite(SENSOR_LEFT_FRONT,HIGH);
  lox.begin(0x31);
  digitalWrite(SENSOR_FRONT_LEFT,HIGH);
  lox.begin(0x32);
  digitalWrite(SENSOR_FRONT_RIGHT,HIGH);
  lox.begin(0x33);
  digitalWrite(SENSOR_LEFT_FRONT,HIGH);
  lox.begin(0x34);
  digitalWrite(SENSOR_LEFT_REAR,HIGH);
  lox.begin(0x35);
  
  Serial.begin(9600);
  xbeeComm.begin(9600);
  mallet.attach(MALLET_PIN);
  mallet.write(MALLET_RETRACT_POS);
  
  while (!xbeeComm.available()) {
    // Just wait until something is received
    Serial.println("Waiting for a command...");
  }
  Serial.println("we got it yo");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
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
