#ifndef SWSPI_PAW3805_H
#define SWSPI_PAW3805_H
#endif

#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"  // 必须包含的基础头文件
#include "freertos/task.h"      // 包含 vTaskDelay 的声明
#include "driver/spi_master.h"  //不底层模拟了
// 定义引脚
#define OPT_CS    19
#define OPT_SCLK  20
#define OPT_SDIO  21
#define REG_RST   8

// 寄存器地址
#define REG_CONFIG         0x06
#define REG_WRITEPROT      0x09
#define REG_ORIENTATION    0x19
#define REG_CPIX           0x0D
#define REG_CPIY           0x0E
#define REG_MOTION_STATUS  0x02
#define REG_XLO            0x03
#define REG_YLO            0x04
#define REG_XHI            0x11
#define REG_YHI            0x12
#define REG_OPMODE         0x05

#define PAW3805_CLK_FREQ 2000000 // 2MHz

// 软件 SPI 写字节（优化后的实现）
void sw_spi_write_byte(uint8_t data);

// 软件 SPI 读字节
uint8_t sw_spi_read_byte();

// 写寄存器
void write_register(uint8_t addr, uint8_t value);

// 读寄存器
uint8_t read_register(uint8_t addr);

// 读取运动数据（示例）
void read_motion_data(int16_t *dx, int16_t *dy);

// 初始化
void init_paw3805ek(spi_host_device_t HOST);

void PAW3805_Function(int16_t* dx, int16_t* dy);