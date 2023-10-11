#include <Arduino.h>

#include "Encoder.h"
#include "SimpleMotorControl.h"
#include "VelocityControl.h"

// // Create a SimpleMotorControl object for a motor
// SimpleMotorControl motorFL(7, 8, 9);
// SimpleMotorControl motorRL(11, 4, 5);
// SimpleMotorControl motorFR(6, 2, 3);
// SimpleMotorControl motorRR(10, 13, 12);

unsigned long previousMillis = 0; // will store last time motor was updated
const long interval = 50;         // interval at which to run motor (milliseconds)

VelocityControl motorControl(19, 18, 10, 13, 12, 0.5); // RR

void setup()
{
  pinMode(22, INPUT);
  // analog read on pin A8
  pinMode(23, INPUT);
  Serial.begin(9600);

  // Set the desired velocity setpoint in counts/sec
  motorControl.setSetpoint(600);

  // Set the PID and feed-forward gains based on tuning
  motorControl.setKp(0.3); // Proportional gain
  motorControl.setKi(0.0); // Integral gain
  motorControl.setKd(0);   // Derivative gain
  motorControl.setKf(0.3); // Feed-forward gain
}

void loop()
{

  unsigned long currentMillis = millis();

  motorControl.control();
}
