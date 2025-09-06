#include <array>
#include <esp_log.h>

#include "Robot.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>

// loggers
static const char *HC_SR04_TAG = "hc-sr04_log";
static const char *MOTOR_TAG = "motor_log";
static const char* QRD_TAG = "qrd_log";

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


// MOTOR 
void drive_motors(MotorDefinition* rightMotor, MotorDefinition* leftMotor, int speed)
{
    rightMotor->Drive(speed);
    leftMotor->Drive(speed);
}


////////////////////////////////////////////////////////////////////////
RobotDefinition::RobotDefinition()
    : leftMotor(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1),
      rightMotor(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0),
      stby(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE),
      clockwise(true)
{
    // init hr-sr04
    ESP_LOGI(HC_SR04_TAG, "Creating HC-SR04 sensor...");
    hcsr04 = HCSR04Sensor(HC_SR04_TRIGGER, HC_SR04_ECHO);
    ESP_LOGI(HC_SR04_TAG, "Initialisating HC-SR04 sensor...");
    hcsr04.init();
    ESP_LOGI(HC_SR04_TAG, "HC-SR04 sensor initialized.");

    // init qrd1114 sensors
    adc1_config_width(WIDTH);
    ESP_LOGI(QRD_TAG, "Creating QRD1114 sensors...");
    leftQrd1114 = QRD1114Sensor(LEFT_QRD1114_CHANNEL);
    rightQrd1114 = QRD1114Sensor(RIGHT_QRD1114_CHANNEL);
    ESP_LOGI(QRD_TAG, "Initialisating left QRD1114 sensor...");
    leftQrd1114.init(ATTENUATION);
    ESP_LOGI(QRD_TAG, "Initialisating right QRD1114 sensor...");
    rightQrd1114.init(ATTENUATION);
    ESP_LOGI(QRD_TAG, "QRD1114 sensors initialized.");
    ESP_ERROR_CHECK(esp_adc_cal_characterize(ADC_UNIT_1, ATTENUATION, WIDTH, 1100, &adc_chars)); // 1100mV default Vref

    // init motors
    ESP_LOGI(MOTOR_TAG, "Creating motors...");
    ESP_LOGI(MOTOR_TAG, "Creating right motor...");
    rightMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    ESP_LOGI(MOTOR_TAG, "Creating left motor...");
    leftMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);
    ESP_LOGI(MOTOR_TAG, "Creating STBY pin...");
    stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);
    
    // Configure motor control instances
    rightMotor.Configure();
    ESP_LOGI(MOTOR_TAG, "Right motor configured on GPIO %d and %d", MOTOR_A_IN_1, MOTOR_A_IN_2);
    leftMotor.Configure();
    ESP_LOGI(MOTOR_TAG, "Left motor configured on GPIO %d and %d", MOTOR_B_IN_1, MOTOR_B_IN_2);

    // Configure STBY pin as output
    stby.Configure();
    ESP_LOGI(MOTOR_TAG, "STBY pin configured on GPIO %d", STBY);
    ESP_ERROR_CHECK(gpio_set_level(stby.Pin(), 1)); // Set STBY high to enable the motors
}

void RobotDefinition::Drive(Direction dir, int speed)
{
    drive_motors(&rightMotor, &leftMotor, speed);
}

void RobotDefinition::ScanEnvironment()
{
    bool continueScan = true;
    if (scanStep++ > MAX_SCAN_STEP)
    {
        scanStep = 0;
        // move forward
        drive_motors(&rightMotor, &leftMotor, SCAN_SPEED);
        vTaskDelay(pdMS_TO_TICKS(500));
        Stop();
        vTaskDelay(pdMS_TO_TICKS(100));
    } 
    else
    {
        if (clockwise)
        {
            ESP_LOGD(MOTOR_TAG, "Scanning clockwise");
            // step back 1/5 second
            drive_motors(&rightMotor, &leftMotor, -SCAN_SPEED);
            vTaskDelay(pdMS_TO_TICKS(100));
            Stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            // turn right
            for(int c = 0; c < 20; c++) // turn for 2 seconds
            {
                rightMotor.Drive(-SCAN_SPEED);
                leftMotor.Drive(SCAN_SPEED);
                vTaskDelay(pdMS_TO_TICKS(50));
                UpdateSensors();
                if (distance < 15.0)
                {
                    ESP_LOGI(MOTOR_TAG, "Object detected during scan at %.2f cm, stopping scan", distance);
                    continueScan = false;
                    break;
                }
            }
            if (!continueScan)
            {
                scanStep = 0;
                return;
            }
            // rightMotor.Drive(-SCAN_SPEED);
            // leftMotor.Drive(SCAN_SPEED);
            // vTaskDelay(pdMS_TO_TICKS(1000));
            Stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            // step forward 1/5 second
            drive_motors(&rightMotor, &leftMotor, SCAN_SPEED);
            vTaskDelay(pdMS_TO_TICKS(100));
            Stop();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            ESP_LOGD(MOTOR_TAG, "Scanning counter-clockwise");
            rightMotor.Drive(-SCAN_SPEED);
            leftMotor.Drive(SCAN_SPEED);
        }
    }
}

void RobotDefinition::Stop()
{
    rightMotor.Stop();
    leftMotor.Stop();
}

QRD1114Data RobotDefinition::GetQRD1114Data()
{
    return this->qrdData;
}

float RobotDefinition::GetDistance()
{
    ESP_LOGD("RobotDefinition", "Getting distance");
    return this->distance;
}

void RobotDefinition::UpdateSensors()
{
    distance = hcsr04.getDistance();
    qrdData.left = leftQrd1114.readData(&adc_chars);
    qrdData.right = rightQrd1114.readData(&adc_chars);
}

RobotDefinition::~RobotDefinition()
{
}