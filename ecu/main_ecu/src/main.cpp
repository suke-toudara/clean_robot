#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 400000
#define ROLLERCAN_I2C_ADDR 0x38 // 仮のI2Cアドレス
#define CMD_SET_SPEED 0x01      // 仮の速度設定コマンドID
#define CMD_GET_STATUS 0x02     // 仮のステータス取得コマンドID

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t setMotorSpeed(uint8_t speed) {
    uint8_t data[2] = {CMD_SET_SPEED, speed};
    return i2c_master_write_to_device(ROLLERCAN_I2C_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
}

esp_err_t getMotorStatus(uint8_t *status) {
    uint8_t cmd = CMD_GET_STATUS;
    esp_err_t ret = i2c_master_write_to_device(ROLLERCAN_I2C_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    return i2c_master_read_from_device(ROLLERCAN_I2C_ADDR, status, 1, 1000 / portTICK_PERIOD_MS);
}

// put function declarations here:
int myFunction(int, int);

extern "C" void app_main() {
    i2c_master_init();
    printf("RollerCAN I2C Test Start\n");
    setMotorSpeed(128); // 速度を128に設定（例）
    while (1) {
        uint8_t status = 0;
        esp_err_t ret = getMotorStatus(&status);
        if (ret == ESP_OK) {
            printf("Motor Status: %d\n", status);
        } else {
            printf("I2C Read Error: %d\n", ret);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}