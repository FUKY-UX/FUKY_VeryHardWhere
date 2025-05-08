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

    SemaphoreHandle_t mutex;  // 互斥锁句柄
    bool IsMouseStop;
}
SharedState_t;

void delay_ns_Main(uint32_t ns) 
{
    uint32_t cycles = ns * (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ) / 1000;
    for (volatile int i = 0; i < cycles; i++);
}
//static const char TAG[] = "FUKYMOUSE";


//======================================初始化====================================//
#define MOSI    6
#define MISO    3
#define SCLK    4
#define FUKY_SPI_HOST    SPI2_HOST

SharedState_t shared_state = 
{
    .IsMouseStop = false,
    .mutex = NULL,
};

//int16_t delta_x, delta_y;


void Main_Init(void);
//最后决定用一个核心来跑光电
void MouseTask(void *pvParameters) 
{
    int8_t local_raw_x = 0, local_raw_y = 0;
    SharedState_t *state = (SharedState_t *)pvParameters;
    while (1) 
    {
        bool IsStop;
        PAW3805_Function(&local_raw_x, &local_raw_y);
        IsStop = local_raw_x == 0 && local_raw_x == 0;
        state->IsMouseStop = IsStop;
        if(IsStop){continue;}
        send_mouse_value(0,local_raw_x,local_raw_y);
        //state->DataUpdated = true;
        //printf("X: %d, Y: %d\n", local_raw_x, local_raw_y);
        //esp_rom_delay_us(500);
    }

}

void IMUTask(void *pvParameters) 
{
    IMUData_t local_imu_data;
    SharedState_t *state = (SharedState_t *)pvParameters;
    while (1) 
    {
        //带锁访问共享状态
        if (xSemaphoreTake(state->mutex, 0)) 
        {
            if(!state->IsMouseStop)
            {
                xSemaphoreGive(state->mutex);
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            xSemaphoreGive(state->mutex);
        }
        local_imu_data = bno080_Function();
        //带锁访问共享状态
        SendIMUData(local_imu_data.lin_accel_x, local_imu_data.lin_accel_y, local_imu_data.lin_accel_z, local_imu_data.quat_i, local_imu_data.quat_j, local_imu_data.quat_k, local_imu_data.quat_w);
        //ESP_LOGI("IMU","数据");
    }

}

// //尝试在一个核心运行两个任务，但效率更低了，弃用
// void MouseTask_0(void *pvParameters) 
// {
//     int8_t local_raw_x = 0, local_raw_y = 0;
//     SharedState_t *state = (SharedState_t *)pvParameters;
//     while (1) 
//     {
//         PAW3805_Function(&local_raw_x, &local_raw_y);
//         bool Isfloating =(local_raw_x == 0 && local_raw_x == 0);
//         //带锁访问共享状态
//         if (xSemaphoreTake(state->mutex, portMAX_DELAY)) 
//         {
//             state->IsMouseStop = Isfloating;
//             if(Isfloating)
//             {
//                 xSemaphoreGive(state->mutex);
//                 continue;
//             }
//             state->DataUpdated =true;
//             state->raw_x = local_raw_x;
//             state->raw_y = local_raw_y;
//             xSemaphoreGive(state->mutex);
//         }
//     }
//     esp_rom_delay_us(250);
// }
// //尝试在一个核心运行两个任务，但效率更低了，弃用
// void IMUTask_0(void *pvParameters) 
// {
//     IMUData_t local_imu_data;
//     SharedState_t *state = (SharedState_t *)pvParameters;
//     while (1) 
//     {
//         //带锁访问共享状态
//         if (xSemaphoreTake(state->mutex, portMAX_DELAY)) 
//         {
//             if(!state->IsMouseStop)
//             {
//                 xSemaphoreGive(state->mutex);
//                 vTaskDelay(pdMS_TO_TICKS(50));
//                 continue;
//             }
//             xSemaphoreGive(state->mutex);
//         }
//         local_imu_data = bno080_Function();
//         //带锁访问共享状态
//         if (xSemaphoreTake(state->mutex, portMAX_DELAY)) 
//         {
//             state->DataUpdated =true;
//             state->imu_data = local_imu_data;
//             xSemaphoreGive(state->mutex);
//         }
//     }
// }
// //一次性读取两个传感器的数据，但是IMU是低频的，光电是高频的，导致鼠标刷新率被限死在1ms
// void ReadDataTask_core0(void *pvParameters) 
// {
//     IMUData_t local_imu_data;
//     int8_t local_raw_x = 0, local_raw_y = 0;
//     SharedState_t *state = (SharedState_t *)pvParameters;
//     while (1) 
//     {
//         local_imu_data = bno080_Function();
//         PAW3805_Function(&local_raw_x, &local_raw_y);

//         //带锁访问共享状态
//         if (xSemaphoreTake(state->mutex, portMAX_DELAY)) 
//         {
//             state->DataUpdated =true;
//             state->imu_data = local_imu_data;
//             state->raw_x = local_raw_x;
//             state->raw_y = local_raw_y;
//             xSemaphoreGive(state->mutex);
//         }

//     }
//     //esp_rom_delay_us(1000);

// }

// void SendDataTask_core1(void *pvParameters) 
// {
//     SharedState_t *state = (SharedState_t *)pvParameters;
//     IMUData_t local_imu_data;
//     int8_t local_raw_x = 0, local_raw_y = 0;
//     bool local_IsStop = false;
//     while (1)
//     {
//         if(!state->IsMouseStop)
//         {
//             if (xSemaphoreTake(state->mutex, 0)) 
//             {
//                 //local_imu_data=state->imu_data;IMU数据直接在这个核心更新得了
//                 local_raw_x = state->raw_x;
//                 local_raw_y =state->raw_y;
//                 xSemaphoreGive(state->mutex);
//             }
//             //如果鼠标不停下，这个核心就一直发送数据就行
//             send_mouse_value(0, local_raw_x, local_raw_y);
//             ESP_LOGI("鼠标","数据");
//         }
//         else
//         {
//             //如果鼠标停下，这个核心就开始读取IMU的数据并发送
//             local_imu_data = bno080_Function();
//             SendIMUData(local_imu_data.lin_accel_x, local_imu_data.lin_accel_y, local_imu_data.lin_accel_z, local_imu_data.quat_i, local_imu_data.quat_j, local_imu_data.quat_k, local_imu_data.quat_w);
//             ESP_LOGI("IMU","数据");
//         }

//     }
// }


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

    // 尝试提高鼠标的频率
    xTaskCreatePinnedToCore(MouseTask, "MouseTask", 4096, &shared_state, 1, NULL, 0);
    // xTaskCreatePinnedToCore(IMUTask_0, "ReadDataTask", 4096, &shared_state, 0, NULL, 0);
    // 将SendDatatask绑定到core1
    xTaskCreatePinnedToCore(IMUTask, "IMUtask", 4096, &shared_state, 0, NULL, 1);
    //MouseTask(&shared_state);

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
