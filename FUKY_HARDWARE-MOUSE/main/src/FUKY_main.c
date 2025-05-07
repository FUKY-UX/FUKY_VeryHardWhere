//#include "spi_PAW3395.h"
#include "swSPI_PAW3805.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "bno080.h"
#include "ble_hidd.h"
#include "esp_timer.h"
//共享数据结构
typedef struct 
{
    bool is_floating;
    SemaphoreHandle_t mutex;  // 互斥锁句柄

        // 鼠标传感器数据
        int8_t raw_x;
        int8_t raw_y;
        
        // IMU传感器数据
        float lin_accel_x;
        float lin_accel_y;
        float lin_accel_z;
} 
SharedState_t;
//static const char TAG[] = "FUKYMOUSE";

//======================================初始化====================================//
#define MOSI    6
#define MISO    3
#define SCLK    4
#define FUKY_SPI_HOST    SPI2_HOST

SharedState_t shared_state = 
{
    .is_floating = false,
    .mutex = NULL,
    .raw_x = 0,
    .raw_y = 0,
    .lin_accel_x = 0.0f,
    .lin_accel_y = 0.0f,
    .lin_accel_z = 0.0f,
};

//int16_t delta_x, delta_y;

bool IsFloating = false;

void Main_Init(void);
void Mouse_Function(void);

void IMUtask_core0(void *pvParameters) 
{
    IMUData_t local_imu_data;
    SharedState_t *state = (SharedState_t *)pvParameters;
    while (1) 
    {
        local_imu_data = bno080_Function();
        // 带锁访问共享状态
        if (xSemaphoreTake(state->mutex, portMAX_DELAY)) 
        {
            state->lin_accel_x = local_imu_data.lin_accel_x;
            state->lin_accel_y = local_imu_data.lin_accel_y;
            state->lin_accel_z = local_imu_data.lin_accel_z;
            if (state->is_floating) 
            {
                SendIMUData(local_imu_data.lin_accel_x, local_imu_data.lin_accel_y, 
                    local_imu_data.lin_accel_z, local_imu_data.quat_i, 
                    local_imu_data.quat_j, local_imu_data.quat_k, 
                    local_imu_data.quat_w);
            }
            xSemaphoreGive(state->mutex);
        }
    }

}

void MOUSEtask_core1(void *pvParameters) 
{
    SharedState_t *state = (SharedState_t *)pvParameters;
    int8_t local_raw_x = 0, local_raw_y = 0;

    while (1)
    {
        PAW3805_Function(&local_raw_x, &local_raw_y);
        // 带锁访问共享状态
        if (xSemaphoreTake(state->mutex, portMAX_DELAY)) 
        {
            state->raw_x = local_raw_x;
            state->raw_y = local_raw_y;
            if (!state->is_floating) 
            {
                send_mouse_value(0, local_raw_x, local_raw_y);
            }
            xSemaphoreGive(state->mutex);
        }
    }
}

void Main_Function(void *pvParameters)
{
    SharedState_t *state = (SharedState_t *)pvParameters;
    while(1)
    {
        if(xSemaphoreTake(state->mutex, portMAX_DELAY))
        {
            bool new_floating =((state->raw_x == 0 && state->raw_y == 0) && (state->lin_accel_z > 0.05 || state->lin_accel_y > 0.05 || state->lin_accel_x > 0.05));
            state->is_floating = new_floating;
            xSemaphoreGive(state->mutex);
        }

    }
}

void app_main(void)
{

        // 初始化互斥锁
    shared_state.mutex = xSemaphoreCreateMutex();
    if (shared_state.mutex == NULL) 
    {
        printf("Failed to create mutex!\n");
        return;
    }
    printf("创建互斥锁\n");
    // 初始化SPI总线
    Main_Init();
    //蓝牙初始化
    BLE_HID_Init();

    // 设备初始化
    printf("初始化PAW3805ek...\n");
    init_paw3805ek(FUKY_SPI_HOST);
    //printf("初始化BNO080...\n");
    bno080_init(FUKY_SPI_HOST);

    // 将task_core0绑定到core0，增加堆栈大小以避免溢出
    xTaskCreatePinnedToCore(IMUtask_core0, "IMUtask_core0", 4096, &shared_state, 5, NULL, 0);
    // 将task_core1绑定到core1
    xTaskCreatePinnedToCore(MOUSEtask_core1, "MOUSEtask_core1", 2048, &shared_state, 5, NULL, 1);
    // 控制蓝牙发送数据类型
    xTaskCreatePinnedToCore(Main_Function, "Main_Function", 2048, &shared_state, 3, NULL, 1);

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
    gpio_install_isr_service(0); //引脚中断服务
}
