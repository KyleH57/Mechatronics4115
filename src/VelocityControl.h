#ifndef VELOCITYCONTROL_H
#define VELOCITYCONTROL_H

#include "Encoder.h"
#include "SimpleMotorControl.h"

class VelocityControl {
private:
    Encoder encoder;
    SimpleMotorControl motor;
    
    float kp, ki, kd, kf;  // Control gains
    int setpoint;
    float previousError;
    float integral;

    long lastCount;
    unsigned long lastTimestamp;

    static const int BUFFER_SIZE = 5;  // Size of the moving average buffer; adjust as necessary
    float velocityBuffer[BUFFER_SIZE];
    int bufferIndex;

    float computeMovingAverage(float newVal);

public:
    VelocityControl(int encoderPinA, int encoderPinB, int motorPWMPin, int motorIn1Pin, int motorIn2Pin);

    void setSetpoint(int _setpoint);
    void setKp(float _kp);
    void setKi(float _ki);
    void setKd(float _kd);
    void setKf(float _kf);
    void control();
};

#endif // VELOCITYCONTROL_H
