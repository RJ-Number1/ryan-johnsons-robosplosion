#include "HCSR04.h"

HCSR04::HCSR04(const byte t, const byte e) {
  trigPin = t;
  echoPin = e;
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.begin(9600);
    DEBUG_SERIAL_PORT.print("[*] Initialized ping sensor (trigPin: ");
    DEBUG_SERIAL_PORT.print(trigPin);
    DEBUG_SERIAL_PORT.print(" , echoPin: ");
    DEBUG_SERIAL_PORT.print(echoPin);
    DEBUG_SERIAL_PORT.println(")");
  }
}

void HCSR04::update(void) {
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(TRIG_HOLD_US);
  digitalWrite(trigPin, LOW);

  rtt_us = pulseIn(echoPin, HIGH, timeout_us);

  if (DEBUG) {
    DEBUG_SERIAL_PORT.print("[*] rtt_us: ");
    DEBUG_SERIAL_PORT.println(rtt_us); 
  }
}

unsigned int HCSR04::getDistanceInCm(void) {
  //update();
  return ((rtt_us / 2) * speedOfSound_cmps / MICROSEC_PER_SEC);
}

unsigned int HCSR04::getDistanceInInch(void) {
  //update();
  return ((rtt_us / 2) * speedOfSound_inchps / MICROSEC_PER_SEC);
}

unsigned int HCSR04::getRtt(void) {
  return rtt_us;
}

