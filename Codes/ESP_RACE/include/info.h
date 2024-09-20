#ifndef INFO_H
#define INFO_H



#define WHEEL_DIAMETER 0.08f // Unit: m.
#define WHEEL_CIRCUMFERENCE 0.2513f // Unit: m. WHEEL_DIAMETER*PI
#define WHEEL_DISTANCE 0.22f // Unit: m. The distance between the two wheels.
#define SENSOR_COUNT 4 // The number of sensors.
#define PI 3.14159f

#define NO_TRACK_SENSOR_THRESHOLD 0.3f
#define NO_TRACK_SENSOR_OUTPUT_THRESHOLD 0.3f

#define CONTROL_UPDATE_PERIOD 0.001f // Unit: s.

//#define SET_LINEAR_SPEED 0.8 // Unit: m/s.
#define PWM_MAX_OUTPUT   0.9f

#define PID_MOTOR_KP              2.5f//0.7f//10.0f//2.0f
#define PID_MOTOR_KI              2.0f//1.0f//2.0f
#define PID_MOTOR_KD              0
#define PID_MOTOR_TAU             1.0f
#define PID_MOTOR_MIN_OUTPUT      -0.9f
#define PID_MOTOR_MAX_OUTPUT      0.9f
#define PID_MOTOR_MIN_INTEGRAL    -0.5f
#define PID_MOTOR_MAX_INTEGRAL    0.5f

#define PID_ANGLE_KP              0.9f
#define PID_ANGLE_KI              0
#define PID_ANGLE_KD              0
#define PID_ANGLE_TAU             0.1f
#define PID_ANGLE_MIN_OUTPUT      -0.5f
#define PID_ANGLE_MAX_OUTPUT      0.5f
#define PID_ANGLE_MIN_INTEGRAL    -0.5f
#define PID_ANGLE_MAX_INTEGRAL    0.5f

// State machine states.
typedef enum {e_init, 
              e_pwm_info,
              e_encoder_info,
              e_sensor_info,
              e_bluetooth_info,
              e_pid_info, 
              e_toggle_enable} Program_State;

typedef enum {e_stop, e_start, e_turn, e_normal, 
              e_reset_encoder} Ble_State;

#endif
