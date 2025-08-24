#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "HCSR04Sensor.h"
#include "PinDefinition.h"

HCSR04Sensor::HCSR04Sensor(){}

HCSR04Sensor::HCSR04Sensor(gpio_num_t triggerPin, gpio_num_t echoPin)
    : triggerPin(triggerPin, GPIO_MODE_OUTPUT, GPIO_PULLDOWN_DISABLE),
      echoPin(echoPin, GPIO_MODE_INPUT, GPIO_PULLDOWN_ENABLE)
{
}

void HCSR04Sensor::init()
{
    // Initialize the sensor
    this->triggerPin.Configure();
    this->echoPin.Configure();
}

float HCSR04Sensor::getDistance()
{
    // Trigger the sensor
    gpio_set_level(triggerPin.Pin(), 0);
    esp_rom_delay_us(2);
    gpio_set_level(triggerPin.Pin(), 1);
    esp_rom_delay_us(10);
    gpio_set_level(triggerPin.Pin(), 0);

    // Wait for echo response
    int64_t startTime = esp_timer_get_time();
    while (gpio_get_level(echoPin.Pin()) == 0)
    {
        if (esp_timer_get_time() - startTime > 50000) // 50ms timeout
        {
            return -1;
        }
    }

    // Measure the echo duration
    startTime = esp_timer_get_time();
    while (gpio_get_level(echoPin.Pin()) == 1)
    {
        if (esp_timer_get_time() - startTime > 50000) // 50ms timeout
        {
            return -2;
        }
    }

    int64_t duration = esp_timer_get_time() - startTime;
    // Speed of sound is ~343 m/s or 0.0343 cm/us
    float distance = (duration * 0.0343) / 2; // Calculate distance in cm

    return distance;
}


const char* HCSR04Sensor::errorMsg(int code)
{
    switch (code)
    {
    case -1:
        return "Echo timeout";
    case -2:
        return "Pulse duration timeout";
    default:
        return "Unknown error";
    }
}