#ifndef SimpleMotorControl_h
#define SimpleMotorControl_h

#include "Arduino.h"

class SimpleMotorControl {
  private:
    int pwmPin;
    int in1Pin;
    int in2Pin;

  public:
    SimpleMotorControl(int _pwmPin, int _in1Pin, int _in2Pin);
    void run(int pwmValue);
};

#endif
