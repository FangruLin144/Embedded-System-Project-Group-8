#include "mbed.h"
#include "C12832.h" // URL: http://os.mbed.com/users/askksa12543/code/C12832/

// The following macros are used for PWM output for different speeds and slopes. 
// They are randomly set values for now (5 FEB 2024) and needs adjustment in debugging sessions.
#define PWM_OUTPUT_HIGH 0.6F
#define PWM_OUTPUT_MID  0.5F
#define PWM_OUTPUT_LOW  0.4F

#define PWM_DUTY_CYCLE_FREQ 50000 // Unit: Hz

class Potentiometer  {                              //Begin Potentiometer class definition
    private:                                            //Private data member declaration
        AnalogIn inputSignal;                           //Declaration of AnalogIn object
        float VDD, currentSampleNorm, currentSampleVolts; //Float variables to speficy the value of VDD and most recent samples

    public:                                             // Public declarations
        Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {}   //Constructor - user provided pin name assigned to AnalogIn...
                                                                            //VDD is also provided to determine maximum measurable voltage
        float amplitudeVolts(void)                      //Public member function to measure the amplitude in volts
        {
            return (inputSignal.read()*VDD);            //Scales the 0.0-1.0 value by VDD to read the input in volts
        }
        
        float amplitudeNorm(void)                       //Public member function to measure the normalised amplitude
        {
            return inputSignal.read();                  //Returns the ADC value normalised to range 0.0 - 1.0
        }
        
        void sample(void)                               //Public member function to sample an analogue voltage
        {
            currentSampleNorm = inputSignal.read();       //Stores the current ADC value to the class's data member for normalised values (0.0 - 1.0)
            currentSampleVolts = currentSampleNorm * VDD; //Converts the normalised value to the equivalent voltage (0.0 - 3.3 V) and stores this information
        }
        
        float getCurrentSampleVolts(void)               //Public member function to return the most recent sample from the potentiometer (in volts)
        {
            return currentSampleVolts;                  //Return the contents of the data member currentSampleVolts
        }
        
        float getCurrentSampleNorm(void)                //Public member function to return the most recent sample from the potentiometer (normalised)
        {
            return currentSampleNorm;                   //Return the contents of the data member currentSampleNorm  
        }
};

C12832 lcd(D11, D13, D12, D7, D10);
PwmOut pwmMotorLeft(PC_8);
PwmOut pwmMotorRight(PC_6);
Potentiometer potLeft(A0, 3.3f);  // Control the duty cycle.
Potentiometer potRight(A1, 3.3f); // Control the PWM frequency.

DigitalOut motorEnable(PB_3);
DigitalOut motorDirection(PB_15);

int main() {
    float pwm_period;

    motorDirection.write(1);
    motorEnable.write(1);

    while(1) {
        pwm_period = 1.0f / ((int(potRight.amplitudeNorm() * 10.0) / 10.0 + 0.01) * PWM_DUTY_CYCLE_FREQ);

        pwmMotorLeft.period(pwm_period);                 // Set the PWM frequency.
        pwmMotorLeft.write(potLeft.amplitudeNorm());     // Set the PWM duty cycle.

        pwmMotorRight.period(pwm_period);
        pwmMotorRight.write(potLeft.amplitudeNorm());

        lcd.locate(0, 0);
        lcd.printf("PWM frequency: %.0f Hz\nPWM duty cycle: %.0f %", (int(potRight.amplitudeNorm() * 10.0) / 10.0 + 0.01) * PWM_DUTY_CYCLE_FREQ, potLeft.amplitudeNorm() * 100);
        wait(1.0);
    }
}
