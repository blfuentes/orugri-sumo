#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "QRD1114Sensor.h"

uint32_t read_qrd1114_raw(adc1_channel_t channel) {
    return adc1_get_raw(channel);
}

uint32_t read_qrd1114_voltage_mv(adc1_channel_t channel, esp_adc_cal_characteristics_t* adc_chars) {
    uint32_t raw = read_qrd1114_raw(channel);
    return esp_adc_cal_raw_to_voltage(raw, adc_chars);
}

QRD1114Sensor::QRD1114Sensor() {}
QRD1114Sensor::QRD1114Sensor(adc1_channel_t sensorChannel) : sensorChannel(sensorChannel) {}

void QRD1114Sensor::init(adc_atten_t attenuation) {
    adc1_config_channel_atten(sensorChannel, attenuation);
}

sensor_data_t QRD1114Sensor::readData(esp_adc_cal_characteristics_t* adc_chars) {
    sensor_data_t data;
    data.raw = read_qrd1114_raw(sensorChannel);
    data.voltage_mv = read_qrd1114_voltage_mv(sensorChannel, adc_chars);
    return data;
}