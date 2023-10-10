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
// int state = 0;  // 0 for first state, 1 for second state, 2 for third state

// // Pins for encoder
// const int encoderPinA = 18; // INT5
// const int encoderPinB = 19; // INT4

// // Variable to store the encoder value
// volatile long encoderValue = 0;

// // Last state of encoder pin A
// volatile int lastEncoderPinA = LOW;

// void encoderISR();

// Encoder encoder0(14, 15); // Assuming you're using pins 18 and 19 for the encoder
// Encoder encoder1(16, 17); // Assuming you're using pins 18 and 19 for the encoder
// Encoder encoder2(18, 19); // Assuming you're using pins 18 and 19 for the encoder
// Encoder encoder3(20, 21); // Assuming you're using pins 18 and 19 for the encoder

VelocityControl motorControl(19, 18, 10, 13, 12, 0.5); // RR


void setup()
{
  pinMode(22, INPUT);
  // analog read on pin A8
  pinMode(23, INPUT);
  Serial.begin(9600);

  // // Set the encoder pins as inputs
  // pinMode(encoderPinA, INPUT_PULLUP);
  // pinMode(encoderPinB, INPUT_PULLUP);

  // // Attach interrupts
  // attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, CHANGE);

      // Set the desired velocity setpoint in counts/sec
    motorControl.setSetpoint(600);

    // Set the PID and feed-forward gains based on tuning
    motorControl.setKp(0.3);  // Proportional gain
    motorControl.setKi(0.0); // Integral gain
    motorControl.setKd(0);  // Derivative gain
    motorControl.setKf(0.3);  // Feed-forward gain
}

void loop()
{
  // encoder3.update();
  // long position = encoder3.getCounts();
  // float speed = encoder3.getVelocity();
  // Serial.print("Position: ");
  // Serial.print(position);
  // Serial.print(" Velocity: ");
  // Serial.println(speed);


  unsigned long currentMillis = millis();

  motorControl.control();


  // // read pin 22 and print value
  // int val = digitalRead(22);
  // Serial.println(val);

  // if (currentMillis - previousMillis >= interval)
  // {
  //   // save the last time motor command was executed
  //   previousMillis = currentMillis;

  //   if (val == 0)
  //   {
  //     motorFL.run(66);
  //     motorRL.run(66);
  //     motorFR.run(44);
  //     motorRR.run(44);
  //   }
  //   else if (val == 1)
  //   {
  //     motorFL.run(44);
  //     motorRL.run(44);
  //     motorFR.run(55);
  //     motorRR.run(55);
  //   }
  // }
}

// void encoderISR() {
//   int currentEncoderPinA = digitalRead(encoderPinA);  // Read the current state of encoderPinA
//   int encoderPinBState = digitalRead(encoderPinB);    // Read the current state of encoderPinB

//   // Determine the direction of rotation
//   // If the state of encoderPinA has changed
//   if (lastEncoderPinA != currentEncoderPinA) {
//     // If encoderPinB state is different than encoderPinA state, we're moving clockwise
//     if (encoderPinBState != currentEncoderPinA) {
//       encoderValue++;
//     } else {
//       encoderValue--;
//     }
//   }

//   // Save the current state of encoderPinA as the last state for comparison in the next cycle
//   lastEncoderPinA = currentEncoderPinA;
// }