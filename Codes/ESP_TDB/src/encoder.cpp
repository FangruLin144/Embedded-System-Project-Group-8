#include "./include/encoder.h"

Encoder::Encoder(PinName channelA, PinName channelB)
    : QEI(channelA, channelB, NC, ENCODER_PULSES_PER_REVOLUTION) {
    lastPulses = 0;
    currentPulses = 0;
    deltaPulses = 0;
    distance = 0;
    velocity = 0;
    angularVelocity = 0;
    totalDistance = 0;

    // ticker.attach(callback(this, &Encoder::ISR_encoderUpdate), ENCODER_UPDATE_PERIOD);
}

void Encoder::update() {
    currentPulses = getPulses();
    deltaPulses = currentPulses - lastPulses;
    lastPulses = currentPulses;
    // if(deltaPulses < 0) {
    //     deltaPulses = currentPulses + ENCODER_PULSES_PER_REVOLUTION - lastPulses;
    // }
    /* ANTI OVERFLOW
    if((currentPulses > ENCODER_OVERFLOW_THRESHOLD) || (currentPulses < -ENCODER_OVERFLOW_THRESHOLD)) {
        reset();
    }
    */
    distance = (float)deltaPulses / (float)ENCODER_PULSES_PER_REVOLUTION * WHEEL_CIRCUMFERENCE;
    totalDistance += distance;
    velocity = distance / CONTROL_UPDATE_PERIOD;
    angularVelocity = velocity / WHEEL_DISTANCE;
}
/*
void Encoder::start() {
    reset(); // Reset the encoder.
    ticker.attach(callback(this, &Encoder::update), CONTROL_UPDATE_PERIOD);
    currentPulses = getPulses();
}

void Encoder::stop() {
    ticker.detach();
}
*/
void Encoder::reset() {
    lastPulses = 0;
    currentPulses = 0;
    deltaPulses = 0;
    distance = 0.0f;
    velocity = 0.0f;
    angularVelocity = 0.0f;
    totalDistance = 0;
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

float Encoder::getTotalDistance() {
    return totalDistance;
}

float Encoder::getVelocity(){
    return velocity;
}

float Encoder::getAngularVelocity(){
    return angularVelocity;
}
