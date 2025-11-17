#include "driver/i2c_master.h"

uint8_t si7021_crc(const uint8_t *data, int len);
float si7021_read_temperature(i2c_master_dev_handle_t dev);
float si7021_read_humidity(i2c_master_dev_handle_t dev);
i2c_master_dev_handle_t  si7021_init(i2c_master_bus_handle_t bus_handle);