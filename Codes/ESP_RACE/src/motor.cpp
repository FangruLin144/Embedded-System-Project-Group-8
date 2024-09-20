#include "./include/motor.h"

Motor::Motor(PinName pwmPin, PinName dirPin, PinName bipPin) : pwm(pwmPin), direction(dirPin), bipolar(bipPin){}

void Motor::setPwmPeriod (float period) {
    pwmPeriod = period;
    pwm.period(pwmPeriod);
}

float Motor::getPwmPeriod (void) {
    return pwmPeriod;
}

void Motor::setPwmDutyCycle (float dc) {
    pwmDutyCycle = dc; 
    pwm.write(pwmDutyCycle);
}

float Motor::getPwmDutyCycle (void) {
    return pwmDutyCycle;
}

// Set direction only works for unipolar mode.
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
            PinName lEncoderChA, PinName lEncoderChB, 
            PinName rEncoderChA, PinName rEncoderChB)
    : motorEnable(mEnable), leftMotor(lMotorPwm, lMotorDir, lMotorBip), rightMotor(rMotorPwm, rMotorDir, rMotorBip), 
      leftEncoder(lEncoderChA, lEncoderChB), rightEncoder(rEncoderChA, rEncoderChB) {

    

    // Set the direction of the motors to forward by default.
    leftMotor.setDirection(1);
    rightMotor.setDirection(0);

    leftMotor.setBipolar(1);
    rightMotor.setBipolar(1);
    
    // Set the default PWM frequency to 25 kHz.
    leftMotor.setPwmPeriod(1.0 / float(MOTOR_PWM_FREQ));
    rightMotor.setPwmPeriod(1.0 / float(MOTOR_PWM_FREQ));

    leftMotor.setPwmDutyCycle(0.5f);
    rightMotor.setPwmDutyCycle(0.5f);

    // Set the motor & encoder motorEnable pin to high by default.
    motorEnable.write(0);
}

void MotorModule::setMotorEnable (int en) {
    motorEnable.write(en);
}

int MotorModule::getMotorEnable (void) {
    return (int)motorEnable.read();
}

void MotorModule::update (void) {
    leftEncoder.update();
    rightEncoder.update();
}

// Set direction only works for unipolar mode.
void MotorModule::turnRight (void) {
    leftMotor.setDirection(0);
    rightMotor.setDirection(1);
}

void MotorModule::turnLeft (void) {
    leftMotor.setDirection(1);
    rightMotor.setDirection(0);
}

void MotorModule::moveBackward (void) {
    leftMotor.setDirection(1);
    rightMotor.setDirection(1);
}

void MotorModule::moveForward (void) {
    leftMotor.setDirection(0);
    rightMotor.setDirection(0);
}
