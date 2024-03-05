#include "mbed.h"
#include "C12832.h"
#include <cstdint>
#include "include/motor.h"
#include "include/sensor.h"

// macro definition

#define JOYSTICK_TIME_MARGIN 0.5f // unit: s

// class and type definition

typedef enum {e_init, 
              e_pwm_info,
              e_encoder_info,
              e_sensor_info,
              e_bluetooth_info,
              e_pid_info, 
              e_toggle_enable} Program_State;
Program_State e_program_state, e_last_program_state;

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

// function prototype definition

void joystick_up_pressed();
void joystick_down_pressedISR();
void joystick_fire_pressedISR();

// functions

void joystick_down_pressedISR() {

    // Avoid joystick jiggling behaviour.
    static Timer down_timer;
    static bool first_enter = true;
    if (first_enter) down_timer.start();
    else {
        if (down_timer.read() <= JOYSTICK_TIME_MARGIN) return;
        down_timer.reset();
    }

    // Update the last program state.
    e_last_program_state = e_program_state;

    // --- Joystick down pressed state machine transition starts.

    switch (e_program_state) {
        case (e_init): 
        case (e_pwm_info): e_program_state = e_encoder_info; break;
        case (e_encoder_info): e_program_state = e_sensor_info; break;
        case (e_sensor_info): e_program_state = e_bluetooth_info; break;
        case (e_bluetooth_info): e_program_state = e_pid_info; break;
        case (e_pid_info): e_program_state = e_pwm_info; break;
        default:
            e_program_state = e_init;
    }

    // --- Joystick down pressed state machine transition ends.

    first_enter = false;
}

void joystick_up_pressedISR() {

    // Avoid joystick jiggling behaviour.
    static Timer up_timer;
    static bool first_enter = true;
    if (first_enter) up_timer.start();
    else {
        if (up_timer.read() <= JOYSTICK_TIME_MARGIN) return;
        up_timer.reset();
    }

    // Update the last program state.
    e_last_program_state = e_program_state;

    // --- Joystick up pressed state machine transition starts.

    switch (e_program_state) {
        case (e_init): 
        case (e_pwm_info): e_program_state = e_pid_info; break;
        case (e_pid_info): e_program_state = e_bluetooth_info; break;
        case (e_bluetooth_info): e_program_state = e_sensor_info; break;
        case (e_sensor_info): e_program_state = e_encoder_info; break;
        case (e_encoder_info): e_program_state = e_pwm_info; break;
        default:
            e_program_state = e_init;
    }

    // --- Joystick up pressed state machine transition ends.

    first_enter = false;
}

void joystick_fire_pressedISR() {

    // Avoid joystick jiggling behaviour.
    static Timer fire_timer;
    static bool first_enter = true;
    if (first_enter) fire_timer.start();
    else {
        if (fire_timer.read() <= JOYSTICK_TIME_MARGIN) return;
        fire_timer.reset();
    }

    // Update the last program state.
    e_last_program_state = e_program_state;

   // --- Joystick fire pressed state machine transition starts.
    e_program_state = e_toggle_enable;
   // --- Joystick fire pressed state machine transition end.

    first_enter = false;
}

Serial hm10(PA_11,PA_12);
Serial pc(USBTX, USBRX); //setting up pins for the bluetooth 
DigitalOut LED(D8);
char s,w;
void serial_config();

/* serial_config allows you to set up your HM-10 module via USB serial port*/
void serial_config(){
    if (pc.readable()) {
    w = pc.getc();
    hm10.putc(w);
    }
}

int main() {

    InterruptIn joystick_up(PA_4);   // A2
    InterruptIn joystick_down(PB_0); // A3
    InterruptIn joystick_fire(PB_5); // D4

    joystick_up.rise(&joystick_up_pressedISR);
    joystick_down.rise(&joystick_down_pressedISR);
    joystick_fire.rise(&joystick_fire_pressedISR);

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

    // Temporary variables.
    float sensorReadings[4];
    
    // Bluetooth configuration.
    pc.baud(9600);
    hm10.baud(9600);//Set up baud rate for serial communication
    while (!hm10.writeable()) { } //wait until the HM10 is ready

    // Enable the motor driver board.
    motorModule.setMotorEnable(1);

    while(1) {
        // Bluetooth receiving process.
        if (hm10.readable()) {
            s = hm10.getc();
            pc.putc(s);
        
            if(s == '1'){
                LED = 1;
            }
            if(s == '0'){
            LED = 0;
            }
        } 
        serial_config();

        // State machine.
        switch (e_program_state) {
            case (e_init):
            case (e_pwm_info):
                lcd.locate(0, 0);
                lcd.printf("PWM frequency: 25000 Hz \n");
                lcd.printf("Left  : %.0f %          \n", 100.0f - leftPot.amplitudeNorm() * 100.0f);
                lcd.printf("Right : %.0f %          ", 100.0f - rightPot.amplitudeNorm() * 100.0f);
                break;
            case (e_encoder_info):
                lcd.locate(0, 0);
                lcd.printf("Encoder readings:       \n");
                lcd.printf("Left pulses:  %6d    \n", motorModule.leftEncoder.getCurrentPulses());
                lcd.printf("Right pulses: %6d    \n", motorModule.rightEncoder.getCurrentPulses());
                break;
            case (e_sensor_info):
                lcd.locate(0, 0);
                lcd.printf("Sensor readings:        \n");
                lcd.printf("1/ %0.3f   2/ %0.3f     \n", sensorReadings[0], sensorReadings[1]);
                lcd.printf("3/ %0.3f   4/ %0.3f     ", sensorReadings[2], sensorReadings[3]);
                break;
            case (e_bluetooth_info):
                lcd.locate(0, 0);
                lcd.printf("Bluetooth info:         \n");
                lcd.printf("                        \n");
                lcd.printf("                        \n");
                break;
            case (e_pid_info):
                lcd.locate(0, 0);
                lcd.printf("PID info:               \n");
                lcd.printf("                        \n");
                lcd.printf("                        \n");
                break;
            case (e_toggle_enable):
                if (motorModule.getMotorEnable()) {
                    motorModule.setMotorEnable(0);
                } else {
                    motorModule.setMotorEnable(1);
                }
                e_program_state = e_last_program_state;
                break;
            default:
                lcd.locate(0, 0);
                lcd.printf("PWM frequency: 25000 Hz \n");
                lcd.printf("Left  : %.0f %          \n", 100.0f - leftPot.amplitudeNorm() * 100.0f);
                lcd.printf("Right : %.0f %          ", 100.0f - rightPot.amplitudeNorm() * 100.0f);
        }

        // Update the PWM duty cycle.
        motorModule.leftMotor.setPwmDutyCycle(leftPot.amplitudeNorm());
        motorModule.rightMotor.setPwmDutyCycle(rightPot.amplitudeNorm());

        // Update sensor readings.
        sensor.getAmplitudeVolts(sensorReadings);

    }

}
