#include "VelocityControl.h"


VelocityControl::VelocityControl(
    int encoderPinA, int encoderPinB,
    int motorPWMPin, int motorIn1Pin, int motorIn2Pin)
    : encoder(encoderPinA, encoderPinB),           // Using initializer list here
      motor(motorPWMPin, motorIn1Pin, motorIn2Pin) // Assuming you meant `SimpleMotorControl`
{

    setpoint = 0;
    previousError = 0;
    integral = 0;
    kp = 0;
    ki = 0;
    kd = 0;
    kf = 0;
    lastCount = 0;
    lastTimestamp = 0;
    bufferIndex = 0;

    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        velocityBuffer[i] = 0.0;
    }
}

void VelocityControl::setSetpoint(int _setpoint)
{
    setpoint = _setpoint;
}

void VelocityControl::setKp(float _kp)
{
    kp = _kp;
}

void VelocityControl::setKi(float _ki)
{
    ki = _ki;
}

void VelocityControl::setKd(float _kd)
{
    kd = _kd;
}

void VelocityControl::setKf(float _kf)
{
    kf = _kf;
}

float VelocityControl::computeMovingAverage(float newVal)
{
    // Add new value to the buffer
    velocityBuffer[bufferIndex] = newVal;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE; // Move index in a circular fashion

    // Compute average
    float sum = 0.0;
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        sum += velocityBuffer[i];
    }
    return sum / BUFFER_SIZE;
}

void VelocityControl::control()
{
    long currentCount = encoder.getCounts();
    unsigned long currentTimestamp = micros();

    // Calculate raw velocity
    float rawVelocity = (currentCount - lastCount) / float(currentTimestamp - lastTimestamp) * 1e6;

    // Apply moving average filter
    float actualVelocity = computeMovingAverage(rawVelocity);

    Serial.println(actualVelocity);

    float error = setpoint - actualVelocity;
    integral += error; // Accumulate the error for the integral term
    float derivative = error - previousError;

    int pwmValue = kp * error + ki * integral + kd * derivative + kf * setpoint;

    // Clamp pwmValue to [-255, 255]
    if (pwmValue > 255)
    {
        pwmValue = 255;
    }
    else if (pwmValue < -255)
    {
        pwmValue = -255;
    }

    motor.run(pwmValue);

    previousError = error;            // Store current error for the next cycle
    lastCount = currentCount;         // Store current count for the next cycle
    lastTimestamp = currentTimestamp; // Store current timestamp for the next cycle
}