#include "mbed.h"
#include "C12832.h"
#include <cstdint>
#include "include/motor.h"
#include "include/sensor.h"
#include "include/pid.h"
#include "include/info.h"

/* --- START OF MACRO DEFINITION --- */
#define JOYSTICK_TIME_MARGIN 0.5f // unit: s
/* --- END OF MACRO DEFINITION --- */

/* --- START OF GLOBAL VARIABLE DEFINITION --- */
Program_State e_program_state, e_last_program_state;
Ble_State e_bluetooth_state;

MotorModule motorModule(PB_12, 
                        PC_8, PB_15, PB_1, 
                        PC_6, PB_13, PB_14, 
                        PA_13, PA_14, 
                        PA_15, PB_7); 
Sensor sensor(PB_2, PC_2, PC_3, PC_5, PC_4);
C12832 lcd(D11, D13, D12, D7, D10);

PID pidMotorLeft(PID_MOTOR_KP, PID_MOTOR_KI, PID_MOTOR_KD, PID_MOTOR_TAU, 
                   PID_MOTOR_MIN_OUTPUT, PID_MOTOR_MAX_OUTPUT,
                   PID_MOTOR_MIN_INTEGRAL, PID_MOTOR_MAX_INTEGRAL);
PID pidMotorRight(PID_MOTOR_KP*2.5f, PID_MOTOR_KI, PID_MOTOR_KD, PID_MOTOR_TAU, 
                PID_MOTOR_MIN_OUTPUT, PID_MOTOR_MAX_OUTPUT,
                PID_MOTOR_MIN_INTEGRAL, PID_MOTOR_MAX_INTEGRAL);                
PID pidAngle(PID_ANGLE_KP, PID_ANGLE_KI, PID_ANGLE_KD, PID_ANGLE_TAU, 
             PID_ANGLE_MIN_OUTPUT, PID_ANGLE_MAX_OUTPUT,
             PID_ANGLE_MIN_INTEGRAL, PID_ANGLE_MAX_INTEGRAL);

// Bluetooth entities
Serial hm10(PA_11,PA_12);
Serial pc(USBTX, USBRX);
DigitalOut bleEnable(PC_10);
DigitalOut LED(D8);

// ISR tickers
Ticker controlTicker;
Ticker bleTicker;

// Test and debug
Timer globalTimer;
Timer trackTimer;
int controlISR_execTime = 0; // Unit: us.
float SET_LINEAR_SPEED = 1.2f;//1.0f
/* --- END OF GLOBAL VARIABLE DEFINITION --- */

/* --- START OF GLOBAL CLASS DEFINITION --- */
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
/* --- END OF GLOBAL CLASS DEFINITION --- */

/* --- START OF FUNCTION PROTOTYPE DEFINITION --- */
void joystick_up_pressed();
void joystick_down_pressedISR();
void joystick_fire_pressedISR();
/* --- END OF FUNCTION PROTOTYPE DEFINITION --- */

/* --- START OF FUNCTION DEFINITION --- */
// State machine.
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

float motorSpeedMap(float input) {
    // Match the motor PID output to the bipolar output. 
    // The output is in the form of a PWM duty cycle under bipolar mode. 
    float output = (input - PID_MOTOR_MIN_OUTPUT) / (PID_MOTOR_MAX_OUTPUT - PID_MOTOR_MIN_OUTPUT) * PWM_MAX_OUTPUT; 
    //float output = input / 100.0f;
    return output;
}

float dutyCycleLimit (float dc) {
    if (dc >= PWM_MAX_OUTPUT) return PWM_MAX_OUTPUT;
    if (dc <= 0) return 0;
    return dc;
}

// ISR definition. 
void ISR_controlUpdate(void) {
    static bool turningFlag = false; // turningFlag(true) indicates it has already entered the turning mode.
    static float sensorReadings[SENSOR_COUNT];
    static float autoStopTotalDistance = 0;
    static bool detectFlag = false;

    // Start the current ISR timing.
    controlISR_execTime = globalTimer.read_us();

    // Update sensor information.
    motorModule.update();
    sensor.update();
    sensor.getAmplitudeVolts(sensorReadings);

    // If no track then stop.
    if ((detectFlag == true) && (trackTimer.read() >= 2.0f)) {
        if (   (sensorReadings[1] <= NO_TRACK_SENSOR_THRESHOLD)
            && (sensorReadings[1] <= NO_TRACK_SENSOR_THRESHOLD)
            && sensor.getSensorArrayOutput() <= NO_TRACK_SENSOR_OUTPUT_THRESHOLD) {
            // No track is present. If keeps inpresent for distance >= 0.1 m then stop.
            autoStopTotalDistance += (motorModule.leftEncoder.getDistance() + motorModule.rightEncoder.getDistance()) / 2.0f;
            if (autoStopTotalDistance >= 0.1f) {
                e_bluetooth_state = e_stop; 
            }
        } else {
            autoStopTotalDistance = (motorModule.leftEncoder.getDistance() + motorModule.rightEncoder.getDistance()) / 2.0f;
        }
    }

    // Bluetooth instructions.
    if ((e_bluetooth_state == e_normal) || (e_bluetooth_state == e_start)) {
        if (detectFlag == false) {
            detectFlag = true;
            trackTimer.start();
        }

        motorModule.setMotorEnable(1);
        // Update PID set information and measurements.
        pidAngle.update(0, sensor.getSensorArrayOutput());  
        pidMotorLeft.update(SET_LINEAR_SPEED - pidAngle.getOutput(), motorModule.leftEncoder.getVelocity());
        pidMotorRight.update(SET_LINEAR_SPEED + pidAngle.getOutput(), motorModule.rightEncoder.getVelocity());
        
        // Update motor duty cycles.
        motorModule.leftMotor.setPwmDutyCycle(motorSpeedMap(pidMotorLeft.getOutput()));
        motorModule.rightMotor.setPwmDutyCycle(motorSpeedMap(pidMotorRight.getOutput()));
    } else if (e_bluetooth_state == e_stop) {
        detectFlag = false;
        trackTimer.stop();
        trackTimer.reset();

        motorModule.setMotorEnable(0);
    } else if (e_bluetooth_state == e_turn) {
        if (turningFlag == false) { // The first time it enters the turning state.
            motorModule.setMotorEnable(0);
            wait(1.0f);
            motorModule.leftEncoder.encoder_reset();
            motorModule.rightEncoder.encoder_reset();
            motorModule.leftMotor.setPwmDutyCycle(0.7f); // Left motor forward.
            motorModule.rightMotor.setPwmDutyCycle(0.3f); // Right motor backward.
            motorModule.setMotorEnable(1);
            turningFlag = true;
        }
        if (motorModule.leftEncoder.getAbsDistance() + motorModule.rightEncoder.getAbsDistance() > PI * WHEEL_DISTANCE * 0.87f) {
            motorModule.setMotorEnable(0);
            e_bluetooth_state = e_normal;
            turningFlag = false;
            wait(0.3f);
        }

        trackTimer.stop();
        trackTimer.reset();
        trackTimer.start();
    }

    // Record the ISR timing.
    controlISR_execTime = globalTimer.read_us() - controlISR_execTime;
}

void ISR_bleUpdate(void) {
    char bleInstruction;

    if (hm10.readable()) {
        bleInstruction = hm10.getc();
        pc.putc(bleInstruction);

        if (bleInstruction == '0') e_bluetooth_state = e_stop; 
        else if (bleInstruction == '1') e_bluetooth_state = e_start;
        else if (bleInstruction == 't') {
            e_bluetooth_state = e_turn;
        } else if (bleInstruction == 's') {
            if (SET_LINEAR_SPEED == 0.6f) SET_LINEAR_SPEED = 1.2f;
            else if (SET_LINEAR_SPEED == 1.2f) SET_LINEAR_SPEED = 0.6f;
        }
    }

}

/* --- END OF FUNCTION DEFINITION --- */

int main() {

    /* --- START OF ENTITY INSTANTIATION --- */
    InterruptIn joystick_up(PA_4);   // A2
    InterruptIn joystick_down(PB_0); // A3
    InterruptIn joystick_fire(PB_5); // D4

    joystick_up.rise(&joystick_up_pressedISR);
    joystick_down.rise(&joystick_down_pressedISR);
    joystick_fire.rise(&joystick_fire_pressedISR);

    Ticker controlTicker;
    /* --- END OF ENTITY INSTANTIATION --- */
    
    /* --- START OF BLUETOOTH CONFIGURATION --- */
    // The bluetooth state won't change until the next instruction input.
    bleEnable.write(1);
    pc.baud(9600);
    hm10.baud(9600);//Set up baud rate for serial communication
    while (!hm10.writeable()) { } //wait until the HM10 is ready
    /* --- END OF BLUETOOTH CONFIGURATION --- */

    /* --- START OF INITIALIZATION --- */
    // Start the global timer (for testing and debugging).
    globalTimer.start();
    // Disable the motor driver board. Press the fire to enable.
    motorModule.setMotorEnable(0);
    // Starts the control ISR. 
    controlTicker.attach(&ISR_controlUpdate, CONTROL_UPDATE_PERIOD);
    bleTicker.attach(&ISR_bleUpdate, CONTROL_UPDATE_PERIOD);
    /* --- END OF INITIALIZATION --- */

    /* --- START OF TEMPORARY VARIABLE DEFINITION --- */
    float sensorReadings[SENSOR_COUNT];
    /* --- END OF TEMPORARY VARIABLE DEFINITION --- */

    while(1) {

        sensor.getAmplitudeVolts(sensorReadings);

        
        // State machine.
        switch (e_program_state) {
            case (e_init):
            case (e_pwm_info):
                lcd.locate(0, 0);
                lcd.printf("PWM frequency: 25000 Hz \n");
                lcd.printf("Left  : %.3f %          \n", motorModule.leftMotor.getPwmDutyCycle());
                lcd.printf("Right : %.3f %          ", motorModule.rightMotor.getPwmDutyCycle());
                break;
            case (e_encoder_info):
                lcd.locate(0, 0);
                lcd.printf("L/R: [%6d] [%6d] \n", motorModule.leftEncoder.getCurrentPulses(), motorModule.rightEncoder.getCurrentPulses());
                lcd.printf("L/R vel: [%4.3f] [%4.3f]  \n", motorModule.leftEncoder.getVelocity(), motorModule.rightEncoder.getVelocity());
                lcd.printf("L/R dis:  [%4.3f] [%4.3f] \n", motorModule.leftEncoder.getTotalDistance(), motorModule.rightEncoder.getTotalDistance());
                break;
            case (e_sensor_info):
                lcd.locate(0, 0);
                lcd.printf("Sensor readings: %0.1f  \n", sensor.getSensorArrayOutput());
                lcd.printf("1/ %0.3f   2/ %0.3f     \n", sensorReadings[0], sensorReadings[1]);
                lcd.printf("3/ %0.3f   4/ %0.3f     ", sensorReadings[2], sensorReadings[3]);
                break;
            case (e_bluetooth_info):
                lcd.locate(0, 0);
                lcd.printf("left PWM output  %0.3f  \n", motorSpeedMap(pidMotorLeft.getOutput()));
                lcd.printf("right PWM output %0.3f  \n", motorSpeedMap(pidMotorLeft.getOutput()));
                lcd.printf("[e]%4.3f [P]%4.3f [I]%4.3f [D]%4.3f", pidMotorLeft.getError(), pidMotorLeft.getProportional(), pidMotorLeft.getIntegral(), pidMotorLeft.getDerivative());
                break;
            case (e_pid_info):
                lcd.locate(0, 0);
                lcd.printf("controlISR exec time %3d \n", controlISR_execTime);
                lcd.printf("Right PID output %0.3f  \n", pidMotorRight.getOutput());
                lcd.printf("Angle PID output %0.3f  ", pidAngle.getOutput());
                break;
            default:
                lcd.locate(0, 0);
                lcd.printf("PWM frequency: 25000 Hz \n");
                lcd.printf("Left  : %.3f %          \n", motorModule.leftMotor.getPwmDutyCycle());
                lcd.printf("Right : %.3f %          ", motorModule.rightMotor.getPwmDutyCycle());
                break;
        }
    }

}
