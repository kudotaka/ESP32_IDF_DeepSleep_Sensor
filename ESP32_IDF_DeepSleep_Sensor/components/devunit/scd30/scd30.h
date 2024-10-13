// https://github.com/Seeed-Studio/Seeed_SCD30/blob/master/SCD30.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

esp_err_t Scd30_Init(i2c_master_bus_handle_t i2c_master_bus_handle);
esp_err_t Scd30_DeInit();
bool Scd30_IsDataAvailable(void);
esp_err_t Scd30_SetAutoSelfCalibration(bool enable);
esp_err_t Scd30_SetMeasurementInterval(uint16_t interval);
esp_err_t Scd30_StartPeriodicMeasurement(void);
esp_err_t Scd30_StopMeasurement(void);
esp_err_t Scd30_SetTemperatureOffset(uint16_t offset);
esp_err_t Scd30_ReadMeasurement(float* result);

#ifdef __cplusplus
}
#endif