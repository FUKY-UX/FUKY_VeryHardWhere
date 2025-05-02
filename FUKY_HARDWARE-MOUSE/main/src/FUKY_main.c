#include "spi_PAW3395.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "bno080.h"
#include "ble_hidd.h"
#include "esp_timer.h"

//static const char TAG[] = "FUKYMOUSE";

//======================================SPI====================================//
#define MOSI    6
#define MISO    3
#define SCLK    4
#define FUKY_SPI_HOST    SPI2_HOST

int16_t delta_x, delta_y;
IMUData_t IMUDataBuffer;

void Main_Init(void);
void Mouse_Function(void);

void app_main(void)
{
    // 初始化SPI总线
    Main_Init();
    //蓝牙初始化
    BLE_HID_Init();

    // // 设备初始化
    // printf("初始化PAW3395...\n");
    // PAW3395_init(FUKY_SPI_HOST);
    printf("初始化BNO080...\n");
    bno080_init(FUKY_SPI_HOST);
    while (1)
    {
        IMUDataBuffer = bno080_Function();
        SendIMUData(IMUDataBuffer.lin_accel_x,IMUDataBuffer.lin_accel_y,IMUDataBuffer.lin_accel_z,
            IMUDataBuffer.quat_i,IMUDataBuffer.quat_j,IMUDataBuffer.quat_k,IMUDataBuffer.quat_w);
        //Mouse_Function();
    }
}

void IRAM_ATTR Mouse_Function()
{
    uint8_t *received_data = Motion_Burst();
    delta_x = (int16_t)(received_data[2] + (received_data[3] << 8));
    delta_y = (int16_t)(received_data[4] + (received_data[5] << 8));
    int8_t  delta_x_8 = (int8_t)((float)delta_x / 32767 * 127);
    int8_t  delta_y_8 = (int8_t)((float)delta_y / 32767 * 127);
    send_mouse_value(GetButtonState(),delta_x_8,delta_y_8);
    free(received_data);
}

void Main_Init()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = MISO,
        .mosi_io_num = MOSI,
        .sclk_io_num = SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0};
    spi_bus_initialize(FUKY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);// SPI总线初始化
    gpio_install_isr_service(0); // 引脚中断服务
}
