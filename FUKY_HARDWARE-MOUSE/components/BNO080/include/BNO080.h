#ifndef BNO080_H
#define BNO080_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"


// 引脚定义
#define BNO_CS          5
#define BNO_INTN        9
#define BNO_RESET       10
#define IMU_PS0         11
#define IMU_PS1         12

// BNO080 SPI配置
#define BNO_SPICLK_SPEED   3000000  // 3MHz，不超过手册规定的最大值
#define SPI_MODE        3        // CPOL=1, CPHA=1


// 定义 HID 报告结构体
typedef struct {
    int16_t lin_accel_x;
    int16_t lin_accel_y;
    int16_t lin_accel_z;
    int16_t quat_i;
    int16_t quat_j;
    int16_t quat_k;
    int16_t quat_w;
} __attribute__((packed)) IMUData_t;

// 全局变量
extern spi_device_handle_t bno_spi;
extern QueueHandle_t gpio_evt_queue;
extern  sh2_SensorValue_t sensorValue;
extern IMUData_t IMUData;

extern sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor
extern bool _reset_occurred;
extern sh2_SensorValue_t *_sensor_value;
extern sh2_Hal_t _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer

// 初始化SPI通讯并添加bno080设备
bool bno080_init(spi_host_device_t HOST);
// 需要循环调用轮询状态的函数
IMUData_t IRAM_ATTR bno080_Function(void);
void Check_SensorInterval();

#endif // BNO080_H