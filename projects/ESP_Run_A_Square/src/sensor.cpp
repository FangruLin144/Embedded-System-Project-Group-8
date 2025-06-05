#include "./include/sensor.h"

Sensor::Sensor (PinName enablePin, PinName signalPin1, PinName signalPin2, PinName signalPin3, PinName signalPin4) 
    : enable(enablePin), signal1(signalPin1), signal2(signalPin2), signal3(signalPin3), signal4(signalPin4) {
    
    // Set the enable pin to low.
    enable.write(0);
}

void Sensor::setEnable (int en) {
    enable.write(en);
}

int Sensor::getEnable (void){
    return (int)enable.read();
}

void Sensor::getAmplitudeVolts (float amplitude[]){
    amplitude[0] = signal1.read();
    amplitude[1] = signal2.read();
    amplitude[2] = signal3.read();
    amplitude[3] = signal4.read();
}
