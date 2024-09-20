#include "mbed.h"
#include "./include/pid.h"
#include "./include/info.h"

PID::PID(float kp_, float ki_, float kd_, float tau_,
         float minOutput_, float maxOutput_, 
         float minIntegral_, float maxIntegral_)
        : kp(kp_), ki(ki_), kd(kd_), tau(tau_), 
          minOutput(minOutput_), maxOutput(maxOutput_), 
          minIntegral(minIntegral_), maxIntegral(maxIntegral_) {
    // Controller memory
    proportional = 0;
    integral = 0; 
    derivative = 0;
    error = 0;
    prevError = 0; 
    prevMeasurement = 0;    
    prevDerivative = 0; 

    // controller output
    output = 0;

    // sample time
    sampleTime = CONTROL_UPDATE_PERIOD;
}

void PID::update(float setPoint, float measurement) {
    error = setPoint - measurement;
    //error = measurement - setPoint;

    proportional = kp * error;
    integral += ki * error * sampleTime;
    derivative = kd * (measurement - prevMeasurement) / sampleTime + tau / (1 + tau) * prevDerivative;

    derivative = 0; // Derivative term not in use.

    if (integral > maxIntegral) {
        integral = maxIntegral;
    } else if (integral < minIntegral) {
        integral = minIntegral;
    }

    // Compute Output
    output = proportional + integral + derivative;

    // Apply limits
    if (output > maxOutput) {
        output = maxOutput;
    } else if (output < minOutput) {
        output = minOutput;
    }
    
    // store values for future update
    prevError = error;
    prevMeasurement = measurement;
    prevDerivative = derivative;
}    

float PID::getOutput(void) {
    return output;
}

void PID::reset(void) {
    // Controller memory
    proportional = 0;
    integral = 0; 
    derivative = 0;
    error = 0;
    prevError = 0; 
    prevMeasurement = 0;   
    prevDerivative = 0;  

    // controller output
    output = 0;

    // sample time
    sampleTime = CONTROL_UPDATE_PERIOD;
}

float PID::getError(void) {
    return error;
}

float PID::getProportional(void) {
    return proportional;
}

float PID::getIntegral(void) {
    return integral;
}

float PID::getDerivative(void) {
    return derivative;
}
