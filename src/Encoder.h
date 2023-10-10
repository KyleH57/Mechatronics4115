#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder
{
private:
    int pinA;
    int pinB;
    volatile long counts;
    volatile int lastPinAState;
    volatile unsigned long lastUpdateTime;
    volatile float velocity;
    static Encoder *instance; // Pointer to the instance of the Encoder class

    static const unsigned long TIMEOUT = 50000; // Timeout in microseconds, e.g., 100ms

    static void isrWrapper();
    void isr();


public:
    Encoder(int _pinA, int _pinB);

    long getCounts() const;
    float getVelocity() const;
    void resetCounts();

    void update(); // New method to update velocity based on timeout
};

#endif // ENCODER_H
