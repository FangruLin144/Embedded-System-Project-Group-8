#include "mbed.h"
#include "C12832.h"
#include <cstdint>
#include <cmath>
#include "include/motor.h"
#include "include/sensor.h"

// macro definition

#define JOYSTICK_TIME_MARGIN 0.5f // unit: s
//#define SIDE_PULSE 1200
//#define TURN_PULSE 370
#define ROTATE_PULSE 1200
#define LEFT_STRAIGHT_DUTY_CYCLE 0.6f
#define RIGHT_STRAIGHT_DUTY_CYCLE 0.6f
#define LEFT_TURN_DUTY_CYCLE 0.6f
#define RIGHT_TURN_DUTY_CYCLE 0.6f
#define STOP_DUTY_CYCLE 1.0f
#define WAIT_TIME 1.5f // Unit: s.
#define LCD_REFRESH_PERIOD 0.1f

// class and type definition

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

// Entity instantiation.
    // MotorModule(PinName mEnable, 
    //             PinName lMotorPwm, PinName lMotorDir, PinName lMotorBip,
    //             PinName rMotorPwm, PinName rMotorDir, PinName rMotorBip,
    //             PinName lEncoderChA, PinName lEncoderChB, 
    //             PinName rEncoderChA, PinName rEncoderChB);
MotorModule motorModule(PC_4, 
                        PC_8, PB_15, PB_1, 
                        PC_6, PB_13, PB_14, 
                        PA_13, PA_14, 
                        PA_15, PB_7); 
Sensor sensor(PB_2, PC_2, PC_3, PC_1, PC_0);

C12832 lcd(D11, D13, D12, D7, D10);
Potentiometer leftPot(A0, 3.3f);
Potentiometer rightPot(A1, 3.3f);
Ticker pot_refresh;
int l_rot,r_rot,rot;

int SIDE_PULSE = 1300;
int TURN_PULSE = 320;
int RIGHT_TURN_PULSE = 500;

void lcd_refresh() {
    lcd.locate(0, 0);
    /*
    lcd.printf("Encoder readings:       \n");
    lcd.printf("Left pulses:  %6d    \n", motorModule.leftEncoder.getCurrentPulses());
    lcd.printf("Right pulses: %6d    \n", motorModule.rightEncoder.getCurrentPulses());
    */
    lcd.printf("Thresholds:             \n");
    lcd.printf("SIDE_PULSE: %4d     \n", SIDE_PULSE);
    lcd.printf("TURN_PULSE: %4d     \n", TURN_PULSE);
}

void pot_refresh_function (void) {
    SIDE_PULSE = (int(leftPot.amplitudeNorm() * 50.0) / 50.0f) *  1500; // 1470
    TURN_PULSE = (int(rightPot.amplitudeNorm() * 100.0) / 100.0f) * 600; // 467
    RIGHT_TURN_PULSE = int(TURN_PULSE);
    // TURN_PULSE 444 good
}

int main() {
    motorModule.setMotorEnable(1);

    pot_refresh.attach(&pot_refresh_function, 0.5);
    /*
    // --- First round.
    for(int i = 0; i < 4; i++) {
        // Reset the encoder.
        motorModule.leftEncoder.encoderStart();
        motorModule.rightEncoder.encoderStart();

        // Go forward for 1 m. 
        motorModule.moveForward();
        motorModule.leftMotor.setPwmDutyCycle(RIGHT_STRAIGHT_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(LEFT_STRAIGHT_DUTY_CYCLE);
        while(abs(motorModule.leftEncoder.getCurrentPulses()) < SIDE_PULSE) {lcd_refresh();wait_ms(1);}

        // Stop the buggy.
        motorModule.leftMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);

        wait(WAIT_TIME);

        // Reset the encoder.
        motorModule.leftEncoder.encoderStart();
        motorModule.rightEncoder.encoderStart();

        // Turn right. 
        motorModule.turnRight();
        motorModule.leftMotor.setPwmDutyCycle(LEFT_TURN_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
        while(abs(motorModule.leftEncoder.getCurrentPulses()) < TURN_PULSE) {lcd_refresh();wait_ms(1);}

        // Stop the buggy.
        motorModule.leftMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);

        wait(WAIT_TIME);
    }
    */

    /*
    // --- Turn right.
    // Reset the encoder.
    motorModule.leftEncoder.encoderStart();
    motorModule.rightEncoder.encoderStart();

    wait(0.1);

    // Turn right. 
    motorModule.turnRight();
    motorModule.leftMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
    motorModule.rightMotor.setPwmDutyCycle(RIGHT_TURN_DUTY_CYCLE);
    while(abs(motorModule.rightEncoder.getCurrentPulses()) < int(RIGHT_TURN_PULSE)) {lcd_refresh();wait_ms(1);}

    // Stop the buggy.
    motorModule.leftMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
    motorModule.rightMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);

    wait(WAIT_TIME);
    */

    // --- Second round.
    for(int i = 0; i < 4; i++) {
        // Reset the encoder.
        motorModule.leftEncoder.encoderStart();
        motorModule.rightEncoder.encoderStart();

        // Go forward for 1 m. 
        motorModule.moveForward();
        motorModule.leftMotor.setPwmDutyCycle(LEFT_STRAIGHT_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(RIGHT_STRAIGHT_DUTY_CYCLE);
        while(abs(motorModule.leftEncoder.getCurrentPulses()) < SIDE_PULSE) {lcd_refresh();wait_ms(1);}

        // Stop the buggy.
        motorModule.leftMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);

        wait(WAIT_TIME);

        // Reset the encoder.
        motorModule.leftEncoder.encoderStart();
        motorModule.rightEncoder.encoderStart();

        wait(0.1);

        // Turn left. 
        motorModule.turnLeft();
        motorModule.leftMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(RIGHT_TURN_DUTY_CYCLE);
        while(abs(motorModule.rightEncoder.getCurrentPulses()) < RIGHT_TURN_PULSE) {lcd_refresh();wait_ms(1);}

        // Stop the buggy.
        motorModule.leftMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);
        motorModule.rightMotor.setPwmDutyCycle(STOP_DUTY_CYCLE);

        wait(WAIT_TIME);
    }

    motorModule.setMotorEnable(0);

    while(1) {lcd_refresh();wait(0.1);}

}
