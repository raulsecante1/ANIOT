#include "neo_si7021.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_FREQ_HZ      100000
#define I2C_DEV_ADDR     0x40    // Dirección I2C del Si7021

static const char *TAG = "si7021";

uint8_t si7021_crc(const uint8_t *data, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

float si7021_read_temperature(i2c_master_dev_handle_t dev)
{
    uint8_t data[3];
    uint8_t cmd = 0xE3;  //hold master
//    esp_err_t ret = i2c_master_transmit_receive(dev, &cmd, 1, data, 3, -1);
    esp_err_t ret = i2c_master_transmit(dev, &cmd, 1, -1);
    vTaskDelay(pdMS_TO_TICKS(100));  // Esperar 100ms
    ret = i2c_master_receive(dev, data, 3, -1);

    uint16_t raw = (data[0] << 8) | data[1];

    uint8_t crc_loc = si7021_crc(data, 2);
    if (crc_loc != data[2]) {
        ESP_LOGE(TAG, "CRC mismatch! crc should be 0x%02X received 0x%02X", crc_loc, data[2]);
    }
    else{
        ESP_LOGE(TAG, "CRC match");
    }

    float temp_c = ((175.72f * raw) / 65536.0f) - 46.85f;
    return temp_c;
}

float si7021_read_humidity(i2c_master_dev_handle_t dev)
{
    uint8_t data[3];
    uint8_t cmd = 0xE5;  //hold master
 //   esp_err_t ret = i2c_master_transmit_receive(dev, &cmd, 1, data, 3, -1);
    esp_err_t ret = i2c_master_transmit(dev, &cmd, 1, -1);
    vTaskDelay(pdMS_TO_TICKS(100));  // Esperar 100ms
    ret = i2c_master_receive(dev, data, 3, -1);
    uint16_t raw = (data[0] << 8) | data[1];

    uint8_t crc_loc = si7021_crc(data, 2);
    if (crc_loc != data[2]) {
        ESP_LOGE(TAG, "CRC mismatch! crc should be 0x%02X received 0x%02X", crc_loc, data[2]);
    }
    else{
        ESP_LOGE(TAG, "CRC match");
    }

    float rh = ((125.0f * raw) / 65536.0f) - 6.0f;
    if (rh > 100.0f) rh = 100.0f;
    if (rh < 0.0f) rh = 0.0f;
    return rh;
}

i2c_master_dev_handle_t  si7021_init(i2c_master_bus_handle_t bus_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_DEV_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t si7021;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &si7021));

    return si7021;
}