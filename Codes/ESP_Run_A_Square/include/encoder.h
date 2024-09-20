#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"
#include "QEI.h"

#define ENCODER_PULSES_PER_REVOLUTION 256
#define WHEEL_DIAMETER 0.08f // Unit: m.
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265358979323846f) // Unit: m.
#define WHEEL_DISTANCE 0.15f // Unit: m. The distance between the two wheels.
#define ENCODER_UPDATE_PERIOD 0.01f // Unit: s.

class Encoder: public QEI {
    private: 
        volatile int lastPulses;
        volatile int currentPulses;
        volatile int deltaPulses;
        volatile float distance;
        volatile float velocity;
        volatile float angularVelocity;
        Ticker ticker;

        void ISR_encoderUpdate();

    public: 
        Encoder(PinName channelA, PinName channelB);
        void encoderStart();
        void encoderStop();
        int getLastPulses();
        int getCurrentPulses();
        int getDeltaPulses();
        float getDistance();
        float getVelocity();
        float getAngularVelocity();
};

#endif
