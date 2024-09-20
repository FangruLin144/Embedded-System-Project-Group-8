#ifndef PID_H
#define PID_H

#include "mbed.h"

/*
There are two sets of PIDs in the systsem: angle and motors. 

The motors follow a constant speed, namely a set target. 
The angle PID output contributes the speed adjustment on the left motor. 
*/

/* output = kp*[e] + ki*sigma[e] + kd*delta[e] */

class PID {
    private:
        // Controller gains
        float kp; 
        float ki; 
        float kd; 
        float tau;

        float minOutput; 
        float maxOutput; 

        float minIntegral; 
        float maxIntegral; 

        float proportional;
        float integral; 
        float derivative;
        float error;
        float prevError; 
        float prevMeasurement; 
        float prevDerivative;

        // Controller output
        float output;

        // Sample time (s)
        float sampleTime;

    public:
        PID(float kp_, float ki_, float kd_, float tau_,
            float minOutput_, float maxOutput_, 
            float minIntegrator_, float maxIntegrator_);
        void update(float setPoint, float measurement);
        void reset();
        float getOutput(void);
        float getError(void);
        float getProportional(void);
        float getIntegral(void);
        float getDerivative(void);
};

#endif 
