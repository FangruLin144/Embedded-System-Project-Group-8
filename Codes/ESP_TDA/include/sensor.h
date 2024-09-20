#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"

class Sensor {
    private:
        DigitalOut enable;
        AnalogIn signal1;
        AnalogIn signal2;
        AnalogIn signal3;
        AnalogIn signal4;

    public:
        Sensor (PinName enablePin, PinName signalPin1, PinName signalPin2, PinName signalPin3, PinName signalPin4);
        void setEnable (int);
        int getEnable (void);
        void getAmplitudeVolts (float[]);
};

#endif
