#ifndef __HCSR04SENSOR_H__
#define __HCSR04SENSOR_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "PinDefinition.h"

class HCSR04Sensor
{
public:
    HCSR04Sensor();
    HCSR04Sensor(gpio_num_t triggerPin, gpio_num_t echoPin);
    void init();
    float getDistance();
    const char* errorMsg(int code);

private:
    PinGPIODefinition triggerPin;
    PinGPIODefinition echoPin;
};

#endif // __HCSR04SENSOR_H__