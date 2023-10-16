#include "Encoder.h"

Encoder* Encoder::encoderPtr = nullptr;

Encoder::Encoder(int _pinA, int _pinB)
{
    pinA = _pinA;
    pinB = _pinB;
    counts = 0;
    lastPinAState = LOW;
    lastUpdateTime = 0;
    encoderPtr = this;  // Set the encoderPtr pointer to this object
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), isrWrapper, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isrWrapper, CHANGE);
}

void Encoder::isrWrapper() {
    if(encoderPtr) {
        encoderPtr->isr();
    }
}

void Encoder::isr() {
    int currentPinAState = digitalRead(pinA);
    int pinBState = digitalRead(pinB);

    unsigned long currentTime = micros();
    unsigned long timeDiff = currentTime - lastUpdateTime;

    // Determine rotation direction
    if (currentPinAState != lastPinAState) {
        if (pinBState != currentPinAState) {
            counts++;
        } else {
            counts--;
        }

        // Calculate velocity
        if (timeDiff > 0) {
            velocity = 1e6 / float(timeDiff);  // Convert time difference from microseconds to seconds
            if (pinBState == currentPinAState) {
                velocity = -velocity;  // Reverse direction
            }
        }

        lastUpdateTime = currentTime;
    }

    lastPinAState = currentPinAState;
}

long Encoder::getCounts() const {
    noInterrupts();  // Ensure counts is read atomically
    long retVal = counts;
    interrupts();
    return retVal;
}

float Encoder::getVelocity() const {
    noInterrupts();  // Ensure velocity is read atomically
    float retVal = velocity;
    interrupts();
    return retVal;
}

void Encoder::resetCounts() {
    noInterrupts();
    counts = 0;
    interrupts();
}

void Encoder::update() {
    unsigned long currentTime = micros();
    if (currentTime - lastUpdateTime > TIMEOUT) {
        velocity = 0.0;
    }
}