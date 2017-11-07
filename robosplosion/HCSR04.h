#ifndef _HCSR04_H_
#define _HCSR04_H_

#define DEBUG             false
#define DEBUG_SERIAL_PORT (Serial)
#define TRIG_HOLD_US      10
#define MICROSEC_PER_SEC  1000000

#include "Arduino.h"

class HCSR04 {
  private:
    static const unsigned long speedOfSound_cmps = 34029;
    static const unsigned long speedOfSound_inchps = 13504;
    const unsigned long timeout_us = 23509;

    byte trigPin;
    byte echoPin;
    unsigned int rtt_us;

  public:
    HCSR04(const byte t, const byte e);
    unsigned int getDistanceInCm(void);
    unsigned int getDistanceInInch(void);
    unsigned int getRtt(void);
    void update(void);
};

#endif
