#ifndef __QRD1114SENSOR_H__
#define __QRD1114SENSOR_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/adc_types_legacy.h>
#include <esp_adc_cal_types_legacy.h>
#include <esp_log.h>

#include "PinDefinition.h"

struct sensor_data_t {
    uint32_t raw;
    uint32_t voltage_mv;
};

class QRD1114Sensor
{
public:
    QRD1114Sensor();
    QRD1114Sensor(adc1_channel_t sensorChannel);
    void init(adc_atten_t attenuation);

    sensor_data_t readData(esp_adc_cal_characteristics_t* adc_chars);
    const char* errorMsg(int code);

private:
    adc1_channel_t sensorChannel;
};

#endif // __QRD1114SENSOR_H__