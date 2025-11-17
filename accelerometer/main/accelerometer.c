#include "driver/i2c.h"
#include "driver/i2c_types.h"
#include "hal/i2c_types.h"
#include "soc/clk_tree_defs.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

#include <driver/gpio.h>

#include <driver/i2c_master.h>

#define DATA_LENGTH 100 
#define TEST_I2C_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_SDA_IO 6
#define SENSOR_BYTES 14 

static const char *TAG = "MPU-6050";

void app_main(void){
    uint8_t data_rd[SENSOR_BYTES];
    i2c_master_bus_config_t i2c_master_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = 0x68,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    // Wake up MPU6050
    uint8_t wakeup[2] = {0x6B, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, wakeup, 2, -1));

    while(1){
        uint8_t reg = 0x3B;
        i2c_master_transmit(dev_handle, &reg, 1, -1);
        i2c_master_receive(dev_handle, data_rd, SENSOR_BYTES, -1);

        int16_t accel_x = (data_rd[0] << 8) | data_rd[1];
        int16_t accel_y = (data_rd[2] << 8) | data_rd[3];
        int16_t accel_z = (data_rd[4] << 8) | data_rd[5];

        int16_t temp = (data_rd[6] << 8) | data_rd[7];

        int16_t gyro_x = (data_rd[8] << 8) | data_rd[9];
        int16_t gyro_y = (data_rd[10] << 8) | data_rd[11];
        int16_t gyro_z = (data_rd[12] << 8) | data_rd[13];

        // Convert to real units
        float ax_g = accel_x / 16384.0f;
        float ay_g = accel_y / 16384.0f;
        float az_g = accel_z / 16384.0f;

        float temp_c = temp / 340.0f + 36.53f;

        float gx_dps = gyro_x / 131.0f;
        float gy_dps = gyro_y / 131.0f;
        float gz_dps = gyro_z / 131.0f;

        ESP_LOGI(TAG, "Accel: %.2f %.2f %.2f g", ax_g, ay_g, az_g);
        ESP_LOGI(TAG, "Gyro: %.2f %.2f %.2f dps", gx_dps, gy_dps, gz_dps);
        ESP_LOGI(TAG, "Temp: %.2f C", temp_c);

        // ESP_LOG_BUFFER_HEX(TAG, data_rd, 14);
        // ESP_LOGI(TAG, data_rd[0], );

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
