#ifndef __QRD1114SENSOR_H__
#define __QRD1114SENSOR_H__

#include <esp_log.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#include "PinDefinition.h"

struct sensor_data_t {
    int raw;
    int voltage_mv;
};

class QRD1114Sensor
{
public:
    QRD1114Sensor();
    QRD1114Sensor(adc_unit_t unit, adc_channel_t channel);

    void init(adc_oneshot_unit_handle_t adc_handle, adc_cali_handle_t cali_handle, adc_atten_t attenuation);
    sensor_data_t readData();
    const char* errorMsg(int code);

private:
    adc_unit_t adc_unit;
    adc_channel_t sensorChannel;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool do_cali;
};

#endif // __QRD1114SENSOR_H__