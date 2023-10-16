#include <Arduino.h>

#include "Encoder.h"
#include "SimpleMotorControl.h"
#include "VelocityControl.h"


// Create a SimpleMotorControl object for a motor
// SimpleMotorControl motorFL(7, 8, 9);
// SimpleMotorControl motorRL(11, 4, 5);
// SimpleMotorControl motorFR(6, 2, 3);
// SimpleMotorControl motorRR(10, 13, 12);

unsigned long previousMillis = 0; // will store last time motor was updated
const long interval = 50;         // interval at which to run motor (milliseconds)


// Encoder encoder0(14, 15); // Assuming you're using pins 18 and 19 for the encoder
// Encoder encoder1(16, 17); // Assuming you're using pins 18 and 19 for the encoder
// Encoder encoder2(18, 19); // Assuming you're using pins 18 and 19 for the encoder
//Encoder encoder3(20, 21); // Assuming you're using pins 18 and 19 for the encoder

VelocityControl motorControl(20, 21, 11, 4, 5, 0.5); // RR


void setup()
{
  pinMode(22, INPUT);
  // analog read on pin A8
  pinMode(23, INPUT);
  Serial.begin(9600);


    //   // Set the desired velocity setpoint in counts/sec
    motorControl.setSetpoint(100);

    // Set the PID and feed-forward gains based on tuning
    motorControl.setKp(0.3);  // Proportional gain
    motorControl.setKi(0.0003); // Integral gain
    motorControl.setKd(0);  // Derivative gain
    motorControl.setKf(0.3);  // Feed-forward gain
}

void loop()
{


  motorControl.control();
  motorControl.printError();

}

