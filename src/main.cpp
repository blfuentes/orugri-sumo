#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

#include "Robot.h"

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize Robot
    RobotDefinition robot = RobotDefinition();

    // variables
    float distance = 0.0;
    QRD1114Data qrdData = {0};
    for (;;)
    {
        // Update sensor readings
        ESP_LOGI(MAIN_TAG, "Updating sensors...");
        robot.UpdateSensors();
        distance = robot.GetDistance();
        qrdData = robot.GetQRD1114Data();

        if (distance < 0)
        {
            ESP_LOGE(MAIN_TAG, "Distance error: %s", robot.SensorErrorMsg(distance));
        }
        else
        {
            ESP_LOGI(MAIN_TAG, "Distance: %.2f cm", distance);

            if (distance < 5.0)
            {
                ESP_LOGW(MAIN_TAG, "Object detected within 10 cm!");
                robot.Drive(Direction{X_Direction::X_CENTER, Y_Direction::FORWARD}, DRIVE_SPEED);
            }
            else
            {
                robot.ScanEnvironment();
            }

            ESP_LOGI(MAIN_TAG, "Left QRD1114 - Raw: %d, Voltage: %dmV", qrdData.left.raw, qrdData.left.voltage_mv);
            ESP_LOGI(MAIN_TAG, "Right QRD1114 - Raw: %d, Voltage: %dmV", qrdData.right.raw, qrdData.right.voltage_mv);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}