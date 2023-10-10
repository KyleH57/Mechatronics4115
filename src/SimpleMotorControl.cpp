#include <Arduino.h>
#include "SimpleMotorControl.h"

SimpleMotorControl::SimpleMotorControl(int _pwmPin, int _in1Pin, int _in2Pin) 
  : pwmPin(_pwmPin), in1Pin(_in1Pin), in2Pin(_in2Pin) {
  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
}

void SimpleMotorControl::run(int pwmValue) {
  // Determine motor direction based on the sign of pwmValue
  if (pwmValue > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (pwmValue < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW); // Can be set to HIGH as well to stop the motor
    digitalWrite(in2Pin, LOW); // Can be set to HIGH as well to stop the motor
  }

  // Set motor speed
  analogWrite(pwmPin, abs(pwmValue));
}
