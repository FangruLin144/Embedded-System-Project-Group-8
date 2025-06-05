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

void Sensor::updateSensorReadings () {
    sensor_readings[0] = signal1.read();
    sensor_readings[1] = signal2.read();
    sensor_readings[2] = signal3.read();
    sensor_readings[3] = signal4.read();

    // Normalize each sensor's reading, as sensors have various max and min voltage readings.
    sensor_readings[0] /= (SENSOR_0_MAX - SENSOR_0_MIN);
    sensor_readings[1] /= (SENSOR_1_MAX - SENSOR_1_MIN);
    sensor_readings[2] /= (SENSOR_2_MAX - SENSOR_2_MIN);
    sensor_readings[3] /= (SENSOR_3_MAX - SENSOR_3_MIN);
}

void Sensor::getAmplitudeVolts (float *res){
    res[0] = sensor_readings[0];
    res[1] = sensor_readings[1];
    res[2] = sensor_readings[2];
    res[3] = sensor_readings[3];
}

void Sensor::update () {
    setEnable(1);
    updateSensorReadings(); 

    /* Get average sensor readings for all the sensors.*/
    /*for (int i = 0; i < SENSOR_WINDOW_SIZE; i ++) {
        updateSensorReadings(); 
        if (i == 0) {
            avg_sensor_readings[i] = sensor_readings[i];
        } else {
            avg_sensor_readings[i] += sensor_readings[i];
        }

        if (i == SENSOR_WINDOW_SIZE-1) {
            avg_sensor_readings[i] /= SENSOR_WINDOW_SIZE;
        }
    }*/

    /* Calculate the weighted sensor array output.*/
    // sensorArrayOutput == 0 denotes aligning with the white line.
    sensorArrayOutput =   (-3.0f) * sensor_readings[0]
                        + (-1.0f) * sensor_readings[1]
                        + (1.0f)  * sensor_readings[2]
                        + (3.0f)  * sensor_readings[3];
    
    setEnable(0);
}

float Sensor::getSensorArrayOutput () {
    return sensorArrayOutput;
}
