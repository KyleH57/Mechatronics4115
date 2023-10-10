#include <Arduino.h>

class MotorControl {
  private:
    int pwmPin;
    int dirPin;
    int maxVel;
    int maxAccel;
    volatile long encoderCounts;
    long targetPos;
    int currentVel;
    
    // PID and FF gains
    float Kp, Ki, Kd, Kff;
    float integralError;
    float prevError;

  public:
    MotorControl(int _pwmPin, int _dirPin, int _maxVel, int _maxAccel) 
      : pwmPin(_pwmPin), dirPin(_dirPin), maxVel(_maxVel), maxAccel(_maxAccel), 
        encoderCounts(0), targetPos(0), currentVel(0), 
        Kp(0), Ki(0), Kd(0), Kff(0), integralError(0), prevError(0) {
      pinMode(pwmPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
    }

    void setTargetPos(long pos) {
      targetPos = pos;
    }

    void updateEncoderCounts(int counts) {
      encoderCounts += counts;
    }

    void setGains(float _Kp, float _Ki, float _Kd, float _Kff) {
      Kp = _Kp;
      Ki = _Ki;
      Kd = _Kd;
      Kff = _Kff;
    }

    void run() {
      long error = targetPos - encoderCounts;
      integralError += error;
      float derivativeError = error - prevError;
      
      // Calculate control signal with PID and FF control
      currentVel = Kp * error + Ki * integralError + Kd * derivativeError + Kff * targetPos;

      // Clamp velocity to maximum limits
      currentVel = constrain(currentVel, -maxVel, maxVel);

      // Set motor direction
      digitalWrite(dirPin, currentVel >= 0 ? HIGH : LOW);
      
      // Set motor speed
      analogWrite(pwmPin, abs(currentVel));

      prevError = error; // Update previous error for next iteration
    }

    // Additional methods can be added as needed
};
