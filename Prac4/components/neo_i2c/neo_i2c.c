#include "neo_i2c.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "esp_log.h"

static const char *TAG = "i2c_driver";

i2c_master_bus_handle_t  i2c_init(int I2C_SCL, int I2C_SDA){
    
    i2c_master_bus_handle_t i2c_handle = NULL;

    if (i2c_handle) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_handle));

    ESP_LOGI(TAG, "I2C initialized successfully");
    return i2c_handle;
}