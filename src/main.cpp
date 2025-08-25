#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <HCSR04Sensor.h>
#include <MotorControl.h>

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";
static const char *HC_SR04_TAG = "hc-sr04_log";
static const char *MOTOR_TAG = "motor_log";

// HC SR04 Sensor pins
constexpr gpio_num_t HC_SR04_TRIGGER = GPIO_NUM_12;
constexpr gpio_num_t HC_SR04_ECHO = GPIO_NUM_14;

// Driver motor pins
constexpr gpio_num_t MOTOR_A_IN_1 = GPIO_NUM_18;
constexpr gpio_num_t MOTOR_A_IN_2 = GPIO_NUM_19;
constexpr gpio_num_t MOTOR_A_PWM = GPIO_NUM_23;

constexpr gpio_num_t MOTOR_B_IN_1 = GPIO_NUM_26;
constexpr gpio_num_t MOTOR_B_IN_2 = GPIO_NUM_27;
constexpr gpio_num_t MOTOR_B_PWM = GPIO_NUM_5;

constexpr gpio_num_t STBY = GPIO_NUM_33;

constexpr ledc_mode_t LEDC_SPEED_MODE = LEDC_LOW_SPEED_MODE;

extern "C" void app_main();

// HC-SR04 Sensor instance
HCSR04Sensor* hcsr04 = nullptr;

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize the sensor
    ESP_LOGI(HC_SR04_TAG, "Creating HC-SR04 sensor...");
    hcsr04 = new HCSR04Sensor(HC_SR04_TRIGGER, HC_SR04_ECHO);
    ESP_LOGI(HC_SR04_TAG, "Initialisating HC-SR04 sensor...");
    hcsr04->init();
    ESP_LOGI(HC_SR04_TAG, "HC-SR04 sensor initialized.");

    // Robot parts
    MotorDefinition rightMotor;
    MotorDefinition leftMotor;
    PinGPIODefinition stby;

    // Configuring motors
    ESP_LOGI(MAIN_TAG, "Configuring motors...");
    rightMotor = MotorDefinition(MOTOR_A_IN_1, MOTOR_A_IN_2, 0, 1, MOTOR_A_PWM, LEDC_CHANNEL_0, LEDC_SPEED_MODE, LEDC_TIMER_0);
    ESP_LOGI(MOTOR_TAG, "Right motor configured on GPIO %d and %d", MOTOR_A_IN_1, MOTOR_A_IN_2);
    leftMotor = MotorDefinition(MOTOR_B_IN_1, MOTOR_B_IN_2, 1, 0, MOTOR_B_PWM, LEDC_CHANNEL_1, LEDC_SPEED_MODE, LEDC_TIMER_1);
    ESP_LOGI(MOTOR_TAG, "Left motor configured on GPIO %d and %d", MOTOR_B_IN_1, MOTOR_B_IN_2);
    stby = PinGPIODefinition(STBY, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE);

    // Configure the motors
    rightMotor.Configure();
    leftMotor.Configure();

    // Configure STBY pin as output
    stby.Configure();
    gpio_set_level(stby.Pin(), 1); // Set STBY high to enable the motors

    // variables
    float distance = 0.0;

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

            if (distance < 20.0)
            {
                // ESP_LOGW(HC_SR04_TAG, "Object detected within 10 cm!");
                // Stop the motors
                rightMotor.Drive(600);
                leftMotor.Drive(600);
            }
            else
            {
                // Move forward
                rightMotor.Stop();
                leftMotor.Stop();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}