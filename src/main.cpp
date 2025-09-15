#include "Robot.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Logger tag for ESP-IDF logging
static const char *MAIN_TAG = "main_log";

// Create the robot object as a global static instance.
static RobotDefinition robot;

extern "C" void app_main();

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // The robot object is already constructed.
    ESP_LOGI(MAIN_TAG, "Robot initialized.");

    // Start the robot's main operation.
    robot.ScanAndAct();

    ESP_LOGI(MAIN_TAG, "Robot is now running. app_main will exit, but tasks will continue.");
}