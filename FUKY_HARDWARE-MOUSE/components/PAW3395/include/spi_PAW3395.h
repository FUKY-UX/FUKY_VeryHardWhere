#ifndef SPI_PAW3395_H
#define SPI_PAW3395_H

#include <stdint.h>
#include "driver/spi_master.h"

// 设备参数定义
#define PAW3395_CLK_FREQ         (10*1000*1000)   // 最大SCLK频率
#define PAW3395_INPUT_DELAY_NS   (35)             // MISO数据有效延迟

// 寄存器操作命令
#define CMD_WRITE   0x80
#define CMD_READ    0x00
#define ADDR_MASK   0x7f

//一些会用到的寄存器写入值
#define POWERUP_RESET   0x5A

//操作中用到的寄存器地址
#define REG_POWERUP_RESET  0x3A

// SPI引脚定义
#define PAW_CS      10
#define PAW_NRESET      9
#define PAW_MOTION      8
#define LClick    5//左键
#define RClick    7//右键

// 时钟速度配置
#define SPI_CLK_SPEED 10000000  // 10MHz

typedef enum {
    High_Performance,  // 高性能模式
    LowPower,          // 低功耗模式
    Office,            // 办公模式
    Corded_Gaming      // 有线游戏模式
} ChipMode;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化SPI总线
 */
void PAW3395_init(spi_host_device_t HOST);

/**
 * @brief 为了方便，把手册里一堆写入操作整成了一个函数，会在Power_up_sequence(void)中调用
 */
void Power_Up_Initializaton_Register_Setting(void);

/**
 * @brief 激活光电芯片的BURST
 */
void Active_burst(void);

/**
 * @brief 初始化PAW3395(上电之后写入一遍寄存器)
 */
void Power_up_sequence(void);

/**
 * @brief SPI全双工单字节传输
 * @param data 要发送的数据
 * @return 接收到的数据
 */
uint8_t SPI_SendReceive(uint8_t data);

/**
 * @brief 读取寄存器值
 * @param address 寄存器地址（0x00-0x7F）
 * @return 读取到的寄存器值
 */
uint8_t read_register(uint8_t address);

/**
 * @brief 写入寄存器值
 * @param address 寄存器地址（0x00-0x7F）
 * @param value 要写入的值
 */
void write_register(uint8_t address, uint8_t value);

/**
 * @brief 重设芯片
 */
void ResetChip(void);

/**
 * @brief 配置芯片运行模式
 * @param mode 要设置的模式（High_Performance/LowPower/Office/Corded_Gaming）
 * @return esp_err_t 返回ESP_OK表示成功，ESP_FAIL表示参数错误
 */
esp_err_t set_chip_mode(ChipMode mode);

void GPIO_Init(void);

uint8_t* Motion_Burst();

int8_t GetButtonState();

#ifdef __cplusplus
}
#endif

#endif // SPI_PAW3395_H