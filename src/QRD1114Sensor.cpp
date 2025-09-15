#include "QRD1114Sensor.h"

QRD1114Sensor::QRD1114Sensor() : adc_unit(ADC_UNIT_1), sensorChannel(ADC_CHANNEL_0), do_cali(false) {}

QRD1114Sensor::QRD1114Sensor(adc_unit_t unit, adc_channel_t channel) 
    : adc_unit(unit), sensorChannel(channel), do_cali(false) {}

void QRD1114Sensor::init(adc_oneshot_unit_handle_t adc_handle, adc_cali_handle_t cali_handle, adc_atten_t attenuation) {
    this->adc_handle = adc_handle;
    this->cali_handle = cali_handle;

    //-------------ADC Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, sensorChannel, &config));

    do_cali = (cali_handle != NULL);
}

sensor_data_t QRD1114Sensor::readData() {
    sensor_data_t data = {0, 0};
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, sensorChannel, &data.raw));
    if (do_cali) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, data.raw, &data.voltage_mv));
    }
    return data;
}

const char* QRD1114Sensor::errorMsg(int code)
{
    // This can be expanded with new error codes if needed
    return "Unknown error";
}