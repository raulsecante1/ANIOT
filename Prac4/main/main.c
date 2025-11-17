#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "unity.h"  //for unit equalaity test

#include "esp_chip_info.h"  //for switching between esp32 and esp32c3

#include "neo_i2c.h"
#include "neo_si7021.h"

#include "icm42670.h"

#include "blink.h"

static const char *TAG = "Prac4";

static icm42670_handle_t icm42670 = NULL;

int map_gyro_to_rgb(float value)
{
    int rgb = (int)((value + 180.0f) / 360.0f * 255.0f);

    if (rgb < 0) rgb = 0;
    if (rgb > 255) rgb = 255;

    return rgb;
}

static void i2c_sensor_icm42670_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t ret;

    ret = icm42670_create(bus_handle, ICM42670_I2C_ADDRESS, &icm42670);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL_MESSAGE(icm42670, "icm42670 create returned NULL");

    /* Configuration of the accelerometer and gyroscope */
    const icm42670_cfg_t imu_cfg = {
        .acce_fs = ACCE_FS_2G,
        .acce_odr = ACCE_ODR_400HZ,
        .gyro_fs = GYRO_FS_2000DPS,
        .gyro_odr = GYRO_ODR_400HZ,
    };
    ret = icm42670_config(icm42670, &imu_cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void app_main(void)
{
    int GP_SDA;
    int GP_SCL; 

    esp_chip_info_t info;
    esp_chip_info(&info);

    switch(info.model) {
        case CHIP_ESP32:
            ESP_LOGI(TAG, "ESP32 detected");

            GP_SDA = 27;
            GP_SCL = 26;

            i2c_master_bus_handle_t bus_handle_esp32 = i2c_init(GP_SCL, GP_SDA);

            // Añadir el dispositivo Si7021
            i2c_master_dev_handle_t si7021 = si7021_init(bus_handle_esp32);

            // Bucle principal: alternar lecturas cada segundo
            while (1) {
                float temp = si7021_read_temperature(si7021);
                if (temp > -100.0f)
                    ESP_LOGI(TAG, " Temperatura: %.2f °C", temp);
                vTaskDelay(pdMS_TO_TICKS(1000));  // Esperar 1 segundo

                float hum = si7021_read_humidity(si7021);
                if (hum >= 0.0f)
                    ESP_LOGI(TAG, " Humedad: %.2f %%", hum);
                vTaskDelay(pdMS_TO_TICKS(1000));  // Esperar 1 segundo antes del siguiente ciclo
            }

            // Limpieza
            ESP_ERROR_CHECK(i2c_master_bus_rm_device(si7021));

            ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle_esp32));
            
            break;
        case CHIP_ESP32C3:
            ESP_LOGI(TAG, "ESP32-C3 detected");

            GP_SDA = 10;
            GP_SCL = 8;

            i2c_master_bus_handle_t bus_handle_esp32c3 = i2c_init(GP_SCL, GP_SDA);

            esp_err_t ret;
            icm42670_value_t acc, gyro;
            float temperature;


            /* Configure the peripheral according to the LED type */
            led_strip_handle_t led_strip = configure_led();

            i2c_sensor_icm42670_init(bus_handle_esp32c3);

            /* Set accelerometer and gyroscope to ON */
            ret = icm42670_acce_set_pwr(icm42670, ACCE_PWR_LOWNOISE);
            TEST_ASSERT_EQUAL(ESP_OK, ret);
            ret = icm42670_gyro_set_pwr(icm42670, GYRO_PWR_LOWNOISE);
            TEST_ASSERT_EQUAL(ESP_OK, ret);

            for (int i = 0; i < 100; i++) {
                vTaskDelay(pdMS_TO_TICKS(500));
                ret = icm42670_get_acce_value(icm42670, &acc);
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ret = icm42670_get_gyro_value(icm42670, &gyro);
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ret = icm42670_get_temp_value(icm42670, &temperature);
                TEST_ASSERT_EQUAL(ESP_OK, ret);
                ESP_LOGI(TAG, "acc_x:%.2f, acc_y:%.2f, acc_z:%.2f, gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f temp: %.1f",
                        acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, temperature);
                
                blink_led(led_strip, map_gyro_to_rgb(gyro.x), map_gyro_to_rgb(gyro.y), map_gyro_to_rgb(gyro.z), 1);
                
            }

            icm42670_delete(icm42670);

            ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle_esp32c3));

            break;
        default:
            ESP_LOGI(TAG, "Other chip");
    }

}