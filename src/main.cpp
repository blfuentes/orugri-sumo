#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <HCSR04Sensor.h>
#include <MotorControl.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";
static const char *HC_SR04_TAG = "hc-sr04_log";
static const char *MOTOR_TAG = "motor_log";
static const char* QRD_TAG = "qrd_log";

// HC SR04 Sensor pins
constexpr gpio_num_t HC_SR04_TRIGGER = GPIO_NUM_27;
constexpr gpio_num_t HC_SR04_ECHO = GPIO_NUM_26;

// Driver motor pins
constexpr gpio_num_t MOTOR_A_IN_1 = GPIO_NUM_18;
constexpr gpio_num_t MOTOR_A_IN_2 = GPIO_NUM_19;
constexpr gpio_num_t MOTOR_A_PWM = GPIO_NUM_23;

constexpr gpio_num_t MOTOR_B_IN_1 = GPIO_NUM_16;
constexpr gpio_num_t MOTOR_B_IN_2 = GPIO_NUM_17;
constexpr gpio_num_t MOTOR_B_PWM = GPIO_NUM_5;

constexpr gpio_num_t STBY = GPIO_NUM_33;

constexpr ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;

// LEFT qrd1114 SENSOR
constexpr gpio_num_t LEFT_QRD1114 = GPIO_NUM_34;
constexpr adc1_channel_t LEFT_QRD1114_CHANNEL = ADC1_CHANNEL_6; // GPIO34 → ADC1_CHANNEL_6

// RIGHT qrd1114 SENSOR
constexpr gpio_num_t RIGHT_QRD1114 = GPIO_NUM_35;
constexpr adc1_channel_t RIGHT_QRD1114_CHANNEL = ADC1_CHANNEL_7; // GPIO35 → ADC1_CHANNEL_7

constexpr adc_atten_t ATTENUATION = ADC_ATTEN_DB_12;           // Full 3.3V range
constexpr adc_bits_width_t WIDTH = ADC_WIDTH_BIT_12;

static esp_adc_cal_characteristics_t adc_chars;

// HC-SR04 Sensor instance
HCSR04Sensor* hcsr04 = nullptr;

// Robot parts
MotorDefinition rightMotor;
MotorDefinition leftMotor;
PinGPIODefinition stby;

constexpr int MAX_DRIVE_SPEED = 1023;
constexpr int DRIVE_SPEED = 600;
constexpr int SCAN_SPEED = 1023;

extern "C" void app_main();

void init_hcsr04()
{
    ESP_LOGI(HC_SR04_TAG, "Creating HC-SR04 sensor...");
    hcsr04 = new HCSR04Sensor(HC_SR04_TRIGGER, HC_SR04_ECHO);
    ESP_LOGI(HC_SR04_TAG, "Initialisating HC-SR04 sensor...");
    hcsr04->init();
    ESP_LOGI(HC_SR04_TAG, "HC-SR04 sensor initialized.");
}

void init_motor_control()
{
    ESP_LOGI(MAIN_TAG, "Configuring motors...");
    rightMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    ESP_LOGI(MOTOR_TAG, "Right motor configured on GPIO %d and %d", MOTOR_A_IN_1, MOTOR_A_IN_2);
    leftMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);
    ESP_LOGI(MOTOR_TAG, "Left motor configured on GPIO %d and %d", MOTOR_B_IN_1, MOTOR_B_IN_2);
    stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);

    // Configure motor control instances
    rightMotor.Configure();
    leftMotor.Configure();

    // Configure STBY pin as output
    stby.Configure();
    gpio_set_level(stby.Pin(), 1); // Set STBY high to enable the motors
}

void drive_motors(int speed)
{
    rightMotor.Drive(speed);
    leftMotor.Drive(speed);
}

void scan_environment(bool* clockwise)
{
    if (*clockwise)
    {
        ESP_LOGI(MAIN_TAG, "Scanning clockwise");
        rightMotor.Drive(SCAN_SPEED);
        leftMotor.Drive(-SCAN_SPEED);
    }
    else
    {
        ESP_LOGI(MAIN_TAG, "Scanning counter-clockwise");
        rightMotor.Drive(-SCAN_SPEED);
        leftMotor.Drive(SCAN_SPEED);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    drive_motors(SCAN_SPEED);
    vTaskDelay(pdMS_TO_TICKS(500));
    *clockwise = !(*clockwise);
}

void init_qrd1114_adc() {
    // Configure ADC width and attenuation
    adc1_config_width(WIDTH);
    adc1_config_channel_atten(LEFT_QRD1114_CHANNEL, ATTENUATION);
    adc1_config_channel_atten(RIGHT_QRD1114_CHANNEL, ATTENUATION);

    // Characterize ADC for voltage conversion
    esp_adc_cal_characterize(ADC_UNIT_1, ATTENUATION, WIDTH, 1100, &adc_chars); // 1100mV default Vref
}

uint32_t read_qrd1114_raw(adc1_channel_t channel) {
    return adc1_get_raw(channel);
}

uint32_t read_qrd1114_voltage_mv(adc1_channel_t channel) {
    uint32_t raw = read_qrd1114_raw(channel);
    return esp_adc_cal_raw_to_voltage(raw, &adc_chars);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize the sensor
    init_hcsr04();

    // Configuring motors
    init_motor_control();

    // Configure LEFT_QRD1114 sensor
    init_qrd1114_adc();

    // variables
    float distance = 0.0;
    bool* clockwise = new bool(true);
    for (;;)
    {
        distance = hcsr04->getDistance();
        if (distance < 0)
        {
            ESP_LOGE(HC_SR04_TAG, "Error: %s", hcsr04->errorMsg(distance));
        }
        else
        {
            ESP_LOGI(HC_SR04_TAG, "Distance: %.2f cm", distance);

            if (distance < 10.0)
            {
                ESP_LOGW(HC_SR04_TAG, "Object detected within 10 cm!");
                drive_motors(DRIVE_SPEED);
            }
            else
            {
                scan_environment(clockwise);
            }
            uint32_t raw = read_qrd1114_raw(LEFT_QRD1114_CHANNEL);
            uint32_t voltage = read_qrd1114_voltage_mv(LEFT_QRD1114_CHANNEL);
            ESP_LOGI(QRD_TAG, "left QRD1114 raw: %d, voltage: %dmV", raw, voltage);
            raw = read_qrd1114_raw(RIGHT_QRD1114_CHANNEL);
            voltage = read_qrd1114_voltage_mv(RIGHT_QRD1114_CHANNEL);
            ESP_LOGI(QRD_TAG, "right QRD1114 raw: %d, voltage: %dmV", raw, voltage);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}