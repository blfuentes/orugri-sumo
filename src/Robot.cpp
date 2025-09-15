#include <array>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

#include "Robot.h"

// loggers
static const char *ROBOT_TAG = "robot_log";
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
    // Create mutex for shared data
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) {
        ESP_LOGE("RobotDefinition", "Failed to create mutex for shared data");
    }
    
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

void RobotDefinition::ScanAndAct()
{
    ESP_LOGI("RobotDefinition", "Scanning and acting...");

    // Pin the sensor task to Core 0
    xTaskCreatePinnedToCore(
        &RobotDefinition::sensor_update_task_wrapper,
        "Sensor Task",
        4096,
        this, // Pass a pointer to this instance
        5,    // Higher priority
        &sensorTaskHandle,
        0);

    // Pin the behavior task to Core 1
    xTaskCreatePinnedToCore(
        &RobotDefinition::robot_behavior_task_wrapper,
        "Behavior Task",
        4096,
        this, // Pass a pointer to this instance
        4,    // Lower priority
        &behaviorTaskHandle,
        1);
}

// Task implementation
void RobotDefinition::sensor_update_task_wrapper(void *pvParameters)
{
    RobotDefinition *robot = static_cast<RobotDefinition*>(pvParameters);
    robot->sensor_update_task();
}

void RobotDefinition::sensor_update_task()
{
    ESP_LOGI(ROBOT_TAG, "Sensor task started on Core 0");
    for(;;)
    {
        UpdateSensors();
        vTaskDelay(pdMS_TO_TICKS(50)); // Update sensors every 50 ms
    }
}

void RobotDefinition::robot_behavior_task_wrapper(void *pvParameters)
{
    RobotDefinition *robot = static_cast<RobotDefinition*>(pvParameters);
    robot->robot_behavior_task();
}

void RobotDefinition::robot_behavior_task()
{
    ESP_LOGI(ROBOT_TAG, "Behavior task started on Core 1");
    float distance = 0.0;
    QRD1114Data qrdData = { 0 };

    for(;;)
    {
        distance = GetDistance();
        qrdData = GetQRD1114Data();


        if (distance < 0)
        {
            ESP_LOGE(ROBOT_TAG, "Distance error: %s", SensorErrorMsg(distance));
        }
        else
        {
            ESP_LOGI(ROBOT_TAG, "Distance: %.2f cm", distance);

            if (distance < 15.0)
            {
                ESP_LOGW(ROBOT_TAG, "Object detected within 15 cm!");
                Drive(Direction{X_Direction::X_CENTER, Y_Direction::FORWARD}, MAX_DRIVE_SPEED);
            }
            else
            {
                ScanEnvironment();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Scan environment every 100 ms
    }
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
    ESP_LOGD("RobotDefinition", "Getting QRD1114 data");
    QRD1114Data qrd_copy = this->qrdData;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // copy the data
        qrd_copy = this->qrdData;
        xSemaphoreGive(dataMutex);
    } else {
        ESP_LOGE("RobotDefinition", "Failed to take mutex for getting QRD1114 data");
    }
    return qrd_copy;
}

float RobotDefinition::GetDistance()
{
    ESP_LOGD("RobotDefinition", "Getting distance");
    float dist_copy;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        dist_copy = this->distance;
        xSemaphoreGive(dataMutex);
    } else {
        ESP_LOGE("RobotDefinition", "Failed to take mutex for getting distance");
        dist_copy = -1.0; // Indicate error
    }
    return dist_copy;
}

void RobotDefinition::UpdateSensors()
{
    // Read sensors and update shared data
    float current_distance = hcsr04.getDistance();
    QRD1114Data current_qrdData;
    current_qrdData.left = leftQrd1114.readData(&adc_chars);
    current_qrdData.right = rightQrd1114.readData(&adc_chars);

    // Update shared data with mutex protection
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        distance = current_distance;
        qrdData = current_qrdData;

        // release the mutex
        xSemaphoreGive(dataMutex);
    } else {
        ESP_LOGE("RobotDefinition", "Failed to take mutex for updating sensor data");
    }
}

RobotDefinition::~RobotDefinition()
{
    if (dataMutex != NULL) {
        vSemaphoreDelete(dataMutex);
    }
}