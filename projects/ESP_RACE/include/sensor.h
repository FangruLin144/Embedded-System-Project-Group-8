#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"
#include "info.h"

#define SENSOR_WINDOW_SIZE 5 

// Each sensor's maximum reading (normalized) above the white track.
#define SENSOR_0_MAX 0.43f
#define SENSOR_1_MAX 0.42f
#define SENSOR_2_MAX 0.41f
#define SENSOR_3_MAX 0.3f
#define SENSOR_0_MIN 0.08f
#define SENSOR_1_MIN 0.08f
#define SENSOR_2_MIN 0.07f
#define SENSOR_3_MIN 0.045f

class Sensor {
    private:
        DigitalOut enable;
        AnalogIn signal1;
        AnalogIn signal2;
        AnalogIn signal3;
        AnalogIn signal4;
        float sensor_readings[SENSOR_COUNT];
        float avg_sensor_readings[SENSOR_COUNT];
        float sensorArrayOutput;
        void updateSensorReadings ();

    public:
        Sensor (PinName enablePin, PinName signalPin1, PinName signalPin2, PinName signalPin3, PinName signalPin4);
        void setEnable (int);
        int getEnable (void);
        void getAmplitudeVolts (float*);
        void update ();
        float getSensorArrayOutput ();
};

#endif
