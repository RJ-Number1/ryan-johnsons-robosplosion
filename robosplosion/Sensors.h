#ifndef Sensors_h
#define Sensors_h
#include "Arduino.h"

class Sensors {
  public:
    Sensors();
    void reset();
    void setChannel(); 
};
#endif

