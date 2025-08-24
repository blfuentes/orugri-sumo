#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <HCSR04Sensor.h>

// Logger tag for ESP-IDF logging
static const char *HC_SR04_TAG = "hc-sr04_log";

// HC SR04 Sensor pins
constexpr gpio_num_t HC_SR04_TRIGGER = GPIO_NUM_9;
constexpr gpio_num_t HC_SR04_ECHO = GPIO_NUM_10;

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
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}