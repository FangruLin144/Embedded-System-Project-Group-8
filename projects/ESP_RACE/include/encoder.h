#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"
#include "QEI.h"
#include "info.h"

#define ENCODER_PULSES_PER_REVOLUTION 512 // 256*2
// #define ENCODER_UPDATE_PERIOD 0.001f // Unit: s.
#define VELOCITY_WINDOW_SIZE 5
#define ENCODER_OVERFLOW_THRESHOLD 1E9

class Encoder: public QEI {
    private: 
        int lastPulses;
        volatile int currentPulses;
        int deltaPulses;
        float distance;
        float velocity;
        float angularVelocity;
        float totalDistance;
        float absDistance;

        

    public: 
        Encoder(PinName channelA, PinName channelB);
        void update();
        void encoder_reset();
        int getCurrentPulses();
        int getDeltaPulses();
        float getDistance();
        float getTotalDistance();
        float getVelocity();
        float getAngularVelocity();
        float getAbsDistance();
};

#endif
