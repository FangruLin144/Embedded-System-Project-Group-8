#include "./include/encoder.h"

Encoder::Encoder(PinName channelA, PinName channelB)
    : QEI(channelA, channelB, NC, ENCODER_PULSES_PER_REVOLUTION) {
    lastPulses = 0;
    currentPulses = 0;
    deltaPulses = 0;
    distance = 0.0f;
    velocity = 0.0f;
    angularVelocity = 0.0f;

    ticker.attach(callback(this, &Encoder::encoderUpdate), ENCODER_UPDATE_PERIOD);
}

void Encoder::encoderUpdate() {
    lastPulses = currentPulses;
    currentPulses = getPulses();
    deltaPulses = currentPulses - lastPulses;
    if(deltaPulses < 0) {
        deltaPulses = currentPulses + ENCODER_PULSES_PER_REVOLUTION - lastPulses;
    }
    distance = (float)deltaPulses / (float)ENCODER_PULSES_PER_REVOLUTION * WHEEL_CIRCUMFERENCE;
    velocity = distance / ENCODER_UPDATE_PERIOD;
    angularVelocity = velocity / WHEEL_DISTANCE;
}

void Encoder::encoderStart() {
    reset(); // Reset the encoder.
    ticker.attach(callback(this, &Encoder::encoderUpdate), ENCODER_UPDATE_PERIOD);
    currentPulses = getPulses();
}

void Encoder::encoderStop() {
    ticker.detach();
}

int Encoder::getLastPulses(){
    return lastPulses;
}

int Encoder::getCurrentPulses(){
    return currentPulses;
}

int Encoder::getDeltaPulses(){
    return deltaPulses;
}

float Encoder::getDistance(){
    return distance;
}

float Encoder::getVelocity(){
    return velocity;
}

float Encoder::getAngularVelocity(){
    return angularVelocity;
}
