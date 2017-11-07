#include <SoftwareSerial.h>
#include "DriveMotors.h"
#include "HCSR04.h"
#include "Mallet.h"
#include "Pins.h"
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X frontRight = Adafruit_VL53L0X();
Adafruit_VL53L0X frontLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X leftFront = Adafruit_VL53L0X();
Adafruit_VL53L0X leftBack = Adafruit_VL53L0X();
Adafruit_VL53L0X rightFront = Adafruit_VL53L0X();
Adafruit_VL53L0X rightLeft = Adafruit_VL53L0X();


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
  
  digitalWrite(SENSOR_LEFT_REAR,HIGH);
  digitalWrite(SENSOR_LEFT_FRONT,LOW);
  digitalWrite(SENSOR_FRONT_LEFT,LOW);
  digitalWrite(SENSOR_FRONT_RIGHT,LOW);
  digitalWrite(SENSOR_RIGHT_FRONT,LOW);
  digitalWrite(SENSOR_RIGHT_REAR,LOW);
  
  frontRight.init(true);
  frontRight.setAddress(0x50);
  

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  frontRight.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

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
