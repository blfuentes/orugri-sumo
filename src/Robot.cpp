#include <array>
#include <esp_log.h>

#include "Robot.h"

// loggers
static const char *HC_SR04_TAG = "hc-sr04_log";
static const char *MOTOR_TAG = "motor_log";
static const char* QRD_TAG = "qrd_log";

RobotDefinition::RobotDefinition(){};

constexpr std::array<const char*, 3> X_DirectionStrings = { "Right", "Center", "Left" };
constexpr std::array<const char*, 3> Y_DirectionStrings = { "Forward", "Center", "Backward" };

const char* RobotDefinition::X_DirectionToString(X_Direction dir)
{
    return X_DirectionStrings.at(static_cast<size_t>(dir));
};

const char* RobotDefinition::Y_DirectionToString(Y_Direction dir)
{
    return Y_DirectionStrings.at(static_cast<size_t>(dir));
}

// HC-SR04
void init_hcsr04(HCSR04Sensor* hcsr04)
{
    ESP_LOGI(HC_SR04_TAG, "Creating HC-SR04 sensor...");
    hcsr04 = new HCSR04Sensor(HC_SR04_TRIGGER, HC_SR04_ECHO);
    ESP_LOGI(HC_SR04_TAG, "Initialisating HC-SR04 sensor...");
    hcsr04->init();
    ESP_LOGI(HC_SR04_TAG, "HC-SR04 sensor initialized.");
}
////////////////////////////////////////////////////////////////////////

// MOTOR 
void init_motor_control(MotorDefinition *rightMotor, MotorDefinition *leftMotor, PinGPIODefinition *stby)
{
    ESP_LOGI(MOTOR_TAG, "Configuring motors...");
    *rightMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    ESP_LOGI(MOTOR_TAG, "Right motor configured on GPIO %d and %d", MOTOR_A_IN_1, MOTOR_A_IN_2);
    *leftMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);
    ESP_LOGI(MOTOR_TAG, "Left motor configured on GPIO %d and %d", MOTOR_B_IN_1, MOTOR_B_IN_2);
    *stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);

    // Configure motor control instances
    rightMotor->Configure();
    leftMotor->Configure();

    // Configure STBY pin as output
    stby->Configure();
    gpio_set_level(stby->   Pin(), 1); // Set STBY high to enable the motors
}

void drive_motors(MotorDefinition* rightMotor, MotorDefinition* leftMotor, int speed)
{
    rightMotor->Drive(speed);
    leftMotor->Drive(speed);
}

void scan_environment(MotorDefinition* rightMotor, MotorDefinition* leftMotor,bool* clockwise)
{
    if (*clockwise)
    {
        ESP_LOGI(MOTOR_TAG, "Scanning clockwise");
        rightMotor->Drive(SCAN_SPEED);
        leftMotor->Drive(-SCAN_SPEED);
    }
    else
    {
        ESP_LOGI(MOTOR_TAG, "Scanning counter-clockwise");
        rightMotor->Drive(-SCAN_SPEED);
        leftMotor->Drive(SCAN_SPEED);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    drive_motors(rightMotor, leftMotor, SCAN_SPEED);
    vTaskDelay(pdMS_TO_TICKS(500));
    *clockwise = !(*clockwise);
}

////////////////////////////////////////////////////////////////////////


void RobotDefinition::Configure()
{
    // init hr-sr04
    init_hcsr04(hcsr04);

    // init motors
    init_motor_control(&rightMotor, &leftMotor, &stby);
}

void RobotDefinition::Drive(Direction dir, int speed)
{
    drive_motors(&rightMotor, &leftMotor, speed);
}

void RobotDefinition::ScanEnvironment()
{
    scan_environment(&rightMotor, &leftMotor, clockwise);
}

void RobotDefinition::Stop()
{
    rightMotor.Stop();
    leftMotor.Stop();
}

float RobotDefinition::GetDistance()
{
    return hcsr04->getDistance();
}