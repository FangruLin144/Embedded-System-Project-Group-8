#include "./include/motor.h"

Motor::Motor(PinName pwmPin, PinName dirPin, PinName bipPin) : pwm(pwmPin), direction(dirPin), bipolar(bipPin){}

void Motor::setPwmFrequency (float freq) {
    pwmFrequency = freq;
}

float Motor::getPwmFrequency (void) {
    return pwmFrequency;
}

void Motor::setPwmDutyCycle (float dc) {
    pwmDutyCycle = dc; 
}

float Motor::getPwmDutyCycle (void) {
    return pwmDutyCycle;
}

void Motor::setDirection (int dir) {
    direction.write(dir);
}

int Motor::getDirection (void) {
    return direction.read();
}

void Motor::setBipolar (int bi) {
    bipolar.write(bi);
}

int Motor::getBipolar (void) {
    return bipolar.read();
}

MotorModule::MotorModule(PinName mEnable, 
            PinName lMotorPwm, PinName lMotorDir, PinName lMotorBip,
            PinName rMotorPwm, PinName rMotorDir, PinName rMotorBip,
            PinName eEnable,
            PinName lEncoderChA, PinName lEncoderChB, 
            PinName rEncoderChA, PinName rEncoderChB)
    : motorEnable(mEnable), leftMotor(lMotorPwm, lMotorDir, lMotorBip), rightMotor(rMotorPwm, rMotorDir, rMotorBip), 
      encoderEnable(eEnable), leftEncoder(lEncoderChA, lEncoderChB), rightEncoder(rEncoderChA, rEncoderChB) {

    

    // Set the direction of the motors to forward by default.
    leftMotor.setDirection(1);
    rightMotor.setDirection(0);

    leftMotor.setBipolar(0);
    rightMotor.setBipolar(0);
    
    // Set the default PWM frequency to 25 kHz.
    leftMotor.setPwmFrequency(MOTOR_PWM_FREQ);
    rightMotor.setPwmFrequency(MOTOR_PWM_FREQ);

    leftMotor.setPwmDutyCycle(0);
    rightMotor.setPwmDutyCycle(0);

    // Set the motor & encoder motorEnable pin to high by default.
    motorEnable.write(1);
    encoderEnable.write(1);

}

void MotorModule::setMotorEnable (int en) {
    motorEnable.write(en);
}

int MotorModule::getMotorEnable (void){
    return (int)motorEnable.read();
}

void MotorModule::setEncoderEnable (int en) {
    encoderEnable.write(en);
}

int MotorModule::getEncoderEnable (void){
    return (int)encoderEnable.read();
}
