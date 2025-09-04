#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "HCSR04Sensor.h"
#include "MotorControl.h"

// HC SR04 Sensor data
constexpr gpio_num_t HC_SR04_TRIGGER = GPIO_NUM_27;
constexpr gpio_num_t HC_SR04_ECHO = GPIO_NUM_26;

// Driver motor data
constexpr gpio_num_t MOTOR_A_IN_1 = GPIO_NUM_18;
constexpr gpio_num_t MOTOR_A_IN_2 = GPIO_NUM_19;
constexpr gpio_num_t MOTOR_A_PWM = GPIO_NUM_23;

constexpr gpio_num_t MOTOR_B_IN_1 = GPIO_NUM_16;
constexpr gpio_num_t MOTOR_B_IN_2 = GPIO_NUM_17;
constexpr gpio_num_t MOTOR_B_PWM = GPIO_NUM_5;

constexpr gpio_num_t STBY = GPIO_NUM_33;

constexpr ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;

constexpr int MAX_DRIVE_SPEED = 1023;
constexpr int DRIVE_SPEED = 600;
constexpr int SCAN_SPEED = 1023;

typedef enum {
    RIGHT = 0,
    X_CENTER,
    LEFT,
} X_Direction;

typedef enum {
    FORWARD = 0,
    Y_CENTER,
    BACKWARD,
} Y_Direction;

struct Direction {
    X_Direction horizontal;
    Y_Direction vertical;
};

class RobotDefinition {
    HCSR04Sensor* hcsr04 = nullptr;
    MotorDefinition leftMotor;
    MotorDefinition rightMotor;

    PinGPIODefinition stby;

    int leftCorrection;
    int rightCorrection;

    bool* clockwise = new bool(true);

    public:
        RobotDefinition();
        RobotDefinition(MotorDefinition leftMotor, MotorDefinition rightMotor, PinGPIODefinition stby, int leftCorrection, int rightCorrection);

        // Configuration
        void Configure();

        // Motor control
        void Drive(Direction dir, int speed);
        void ScanEnvironment();
        void Stop();

        // Sensor data
        float GetDistance();
        const char* SensorErrorMsg(int code) { return hcsr04->errorMsg(code); }

        const char* X_DirectionToString(X_Direction dir);
        const char* Y_DirectionToString(Y_Direction dir);
};

#endif // __ROBOT_H__