#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "HCSR04Sensor.h"
#include "MotorControl.h"
#include "QRD1114Sensor.h"
#include <esp_adc_cal_types_legacy.h>

// HC SR04 Sensor data
constexpr gpio_num_t HC_SR04_TRIGGER = GPIO_NUM_27;
constexpr gpio_num_t HC_SR04_ECHO = GPIO_NUM_26;

// LEFT|RIGHT qrd1114 SENSOR
constexpr adc_atten_t ATTENUATION = ADC_ATTEN_DB_12;           // Full 3.3V range
constexpr adc_bits_width_t WIDTH = ADC_WIDTH_BIT_12;
constexpr adc1_channel_t LEFT_QRD1114_CHANNEL = ADC1_CHANNEL_6; // GPIO34 → ADC1_CHANNEL_6
constexpr adc1_channel_t RIGHT_QRD1114_CHANNEL = ADC1_CHANNEL_7; // GPIO35 → ADC1_CHANNEL_7

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
constexpr int DRIVE_SPEED = 850;
constexpr int SCAN_SPEED = 600;
constexpr int MAX_SCAN_STEP = 6;

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

struct QRD1114Data {
    sensor_data_t left;
    sensor_data_t right;
};

class RobotDefinition {
    HCSR04Sensor hcsr04;
    QRD1114Sensor leftQrd1114;
    QRD1114Sensor rightQrd1114;
    esp_adc_cal_characteristics_t adc_chars;

    MotorDefinition leftMotor;
    MotorDefinition rightMotor;

    PinGPIODefinition stby;

    int leftCorrection;
    int rightCorrection;

    // scan control
    bool clockwise = true;
    int scanStep = 0;

    // stored data
    float distance;
    QRD1114Data qrdData;

public:
    RobotDefinition();
    // Motor control
    void Drive(Direction dir, int speed);
    void Stop();
    void ScanEnvironment();
    // Sensors
    void UpdateSensors();
    // Sensor data
    float GetDistance();
    const char* SensorErrorMsg(int code) { return hcsr04.errorMsg(code); }
    const char* X_DirectionToString(X_Direction dir);
    const char* Y_DirectionToString(Y_Direction dir);
    // QRD1114 sensor data
    QRD1114Data GetQRD1114Data();
    
    ~RobotDefinition();
};

#endif // __ROBOT_H__