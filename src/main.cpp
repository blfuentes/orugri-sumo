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

// LEFT qrd1114 SENSOR
constexpr gpio_num_t LEFT_QRD1114 = GPIO_NUM_34;
constexpr adc1_channel_t LEFT_QRD1114_CHANNEL = ADC1_CHANNEL_6; // GPIO34 → ADC1_CHANNEL_6

// RIGHT qrd1114 SENSOR
constexpr gpio_num_t RIGHT_QRD1114 = GPIO_NUM_35;
constexpr adc1_channel_t RIGHT_QRD1114_CHANNEL = ADC1_CHANNEL_7; // GPIO35 → ADC1_CHANNEL_7

constexpr adc_atten_t ATTENUATION = ADC_ATTEN_DB_12;           // Full 3.3V range
constexpr adc_bits_width_t WIDTH = ADC_WIDTH_BIT_12;

static esp_adc_cal_characteristics_t adc_chars;

extern "C" void app_main();

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

    // Initialize Robot
    RobotDefinition robot = RobotDefinition();
    robot.Configure();

    // Configure LEFT_QRD1114 sensor
    init_qrd1114_adc();

    // variables
    float distance = 0.0;
    for (;;)
    {
        distance = robot.GetDistance();
        if (distance < 0)
        {
            ESP_LOGE(MAIN_TAG, "Distance error: %s", robot.SensorErrorMsg(distance));
        }
        else
        {
            ESP_LOGI(MAIN_TAG, "Distance: %.2f cm", distance);

            if (distance < 10.0)
            {
                ESP_LOGW(MAIN_TAG, "Object detected within 10 cm!");
                robot.Drive(Direction{X_Direction::X_CENTER, Y_Direction::FORWARD}, DRIVE_SPEED);
            }
            else
            {
                robot.ScanEnvironment();
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