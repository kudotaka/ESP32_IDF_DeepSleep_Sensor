
// https://github.com/Seeed-Studio/Seeed_SCD30/blob/master/SCD30.cpp
#include "freertos/FreeRTOS.h"
#include "scd30.h"
#include "string.h"

#define SCD30_ADDR (0x61)
#define SCD30_STANDARD_BAUD (100000)
#define SCD30_TIMEOUT_VALUE_MS (100)

#define SCD30_CONTINUOUS_MEASUREMENT            (0x0010)
#define SCD30_SET_MEASUREMENT_INTERVAL          (0x4600)
#define SCD30_GET_DATA_READY                    (0x0202)
#define SCD30_READ_MEASUREMENT                  (0x0300)
#define SCD30_STOP_MEASUREMENT                  (0x0104)
#define SCD30_AUTOMATIC_SELF_CALIBRATION        (0x5306)
#define SCD30_SET_FORCED_RECALIBRATION_FACTOR   (0x5204)
#define SCD30_SET_TEMPERATURE_OFFSET            (0x5403)
#define SCD30_SET_ALTITUDE_COMPENSATION         (0x5102)
#define SCD30_READ_SERIALNBR                    (0xD033)
#define SCD30_READ_FW_VERSION                   (0xD100)

#define SCD30_SET_TEMP_OFFSET                   (0x5403)

#define SCD30_POLYNOMIAL                        (0x31)

static i2c_master_dev_handle_t dev_scd30_device_handle;

static const char *TAG = "MY-SCD30";

uint8_t Scd30_CalculateCrc(uint8_t data[], uint8_t len) {
    uint8_t bit, byteCtr, crc = 0xff;

    // calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < len; byteCtr ++) {
        crc ^= (data[byteCtr]);

        for (bit = 8; bit > 0; -- bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ SCD30_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

esp_err_t Scd30_ReadBuffer(uint8_t* data, uint8_t len) {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_receive(dev_scd30_device_handle, data, len, SCD30_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "I2CRead i2c_master_receive is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2CRead I2C i2c_master_receive(0) error");
    }
    vTaskDelay(10);
    return ret;
}

esp_err_t Scd30_WriteBuffer(uint8_t* data, uint8_t len) {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_transmit(dev_scd30_device_handle, data, len, SCD30_TIMEOUT_VALUE_MS);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "I2CWrite i2c_master_transmit(1) is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2CWrite I2C i2c_master_transmit(1) error");
    }
    vTaskDelay( pdMS_TO_TICKS(100) );
    return ret;
}

esp_err_t Scd30_WriteCommand(uint16_t command) {
    uint8_t data[2] = { 0 };
    data[0] = command >> 8;
    data[1] = command & 0xff;
    return Scd30_WriteBuffer(data, 2);
}

esp_err_t Scd30_WriteCommandWithArguments(uint16_t command, uint16_t arguments) {
    uint8_t checkSum, buf[5] = { 0 };
    buf[0] = command >> 8;
    buf[1] = command & 0xff;
    buf[2] = arguments >> 8;
    buf[3] = arguments & 0xff;
    checkSum = Scd30_CalculateCrc(&buf[2], 2);
    buf[4] = checkSum;
    return Scd30_WriteBuffer(buf, 5);
}

uint16_t Scd30_ReadRegister(uint16_t address) {
    uint8_t buf[2] = { 0 };
    Scd30_WriteCommand(address);
    Scd30_ReadBuffer(buf, 2);
    return ((((uint16_t)buf[0]) << 8) | buf[1]);
}

esp_err_t Scd30_SetTemperatureOffset(uint16_t offset) {
    return Scd30_WriteCommandWithArguments(SCD30_SET_TEMP_OFFSET, offset);
}

bool Scd30_IsDataAvailable(void) {
    uint16_t ret = Scd30_ReadRegister(SCD30_GET_DATA_READY);
    if (ret == 1)
    {
        return true;
    }
    return false;
}

esp_err_t Scd30_SetAutoSelfCalibration(bool enable) {
    if (enable) {
        return Scd30_WriteCommandWithArguments(SCD30_AUTOMATIC_SELF_CALIBRATION, 1);    //Activate continuous ASC
    } else {
        return Scd30_WriteCommandWithArguments(SCD30_AUTOMATIC_SELF_CALIBRATION, 0);    //Deactivate continuous ASC
    }
}

esp_err_t Scd30_SetMeasurementInterval(uint16_t interval) {
    return Scd30_WriteCommandWithArguments(SCD30_SET_MEASUREMENT_INTERVAL, interval);
}

esp_err_t Scd30_StartPeriodicMeasurement(void) {
    return Scd30_WriteCommandWithArguments(SCD30_CONTINUOUS_MEASUREMENT, 0x0000);
}

esp_err_t Scd30_StopMeasurement(void) {
    return Scd30_WriteCommand(SCD30_STOP_MEASUREMENT);
}

esp_err_t Scd30_ReadMeasurement(float* result) {
    esp_err_t ret = ESP_OK;
    uint8_t buf[18] = { 0 };
    uint32_t co2U32 = 0;
    uint32_t tempU32 = 0;
    uint32_t humU32 = 0;
    float co2Concentration = 0;
    float temperature = 0;
    float humidity = 0;

    ret = Scd30_WriteCommand(SCD30_READ_MEASUREMENT);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Scd30_ReadBuffer(buf, 18);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    co2U32 = (uint32_t)((((uint32_t)buf[0]) << 24) | (((uint32_t)buf[1]) << 16) |
                        (((uint32_t)buf[3]) << 8) | ((uint32_t)buf[4]));

    tempU32 = (uint32_t)((((uint32_t)buf[6]) << 24) | (((uint32_t)buf[7]) << 16) |
                         (((uint32_t)buf[9]) << 8) | ((uint32_t)buf[10]));

    humU32 = (uint32_t)((((uint32_t)buf[12]) << 24) | (((uint32_t)buf[13]) << 16) |
                        (((uint32_t)buf[15]) << 8) | ((uint32_t)buf[16]));

    memcpy(&result[0], &co2U32, sizeof(co2Concentration));
    memcpy(&result[1], &tempU32, sizeof(temperature));
    memcpy(&result[2], &humU32, sizeof(humidity));

    return ret;
}

esp_err_t Scd30_CheckFwVersion()
{
    esp_err_t ret = ESP_OK;
    uint8_t buf[2] = { 0 };

    ret = Scd30_WriteCommand(SCD30_READ_FW_VERSION);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    ret = Scd30_ReadBuffer(buf, 2);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Scd30_PrintFwVersion  Major:%d, Minor:%d", buf[0], buf[1]);
    return ret;
}

esp_err_t Scd30_DeInit() {
    esp_err_t ret = ESP_OK;
    ret = i2c_master_bus_rm_device(dev_scd30_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_rm_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_rm_device error");
    }   
    return ret;
}

esp_err_t Scd30_Init(i2c_master_bus_handle_t i2c_master_bus_handle)
{
    ESP_LOGI(TAG, "SCD30_Init Init()");
    esp_err_t ret = ESP_OK;

    i2c_device_config_t dev_scd30_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD30_ADDR,
        .scl_speed_hz = SCD30_STANDARD_BAUD,
        .flags.disable_ack_check = false,
    };
    ret = i2c_master_bus_add_device(i2c_master_bus_handle, &dev_scd30_cfg, &dev_scd30_device_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "i2c_master_bus_add_device is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_master_bus_add_device error");
        return ret;
    }

    if (Scd30_CheckFwVersion() != ESP_OK)
    {
        return ESP_ERR_NOT_FOUND;
    }

    Scd30_SetMeasurementInterval(2); // 2 seconds between measurements
    Scd30_StartPeriodicMeasurement(); // start periodic measuments
    //Scd30_SetAutoSelfCalibration(true); // Enable auto-self-calibration

    return ret;
}
