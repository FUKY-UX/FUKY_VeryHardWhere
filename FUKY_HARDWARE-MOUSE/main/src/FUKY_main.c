//#include "spi_PAW3395.h"
#include "driver/rtc_io.h"
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

//static const char TAG[] = "FUKYMOUSE";


//======================================初始化====================================//
#define MOSI    6
#define MISO    3
#define SCLK    4

#define M_CLICK    18
#define L_CLICK    16
#define R_CLICK    17
#define BTN_HISTORY_SIZE 40 //按键均值消抖
//#define MOVE_HISTORY_SIZE 5  // 移动均值消抖，5个样本够了-------弃用，增加延迟和粘滞手感
#define MOVE_BUFFER_SIZE 3    // 连续相同负值的判定阈值
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
    bool IsStop;//用于判断鼠标是不是停下了
    int8_t local_raw_x = 0, local_raw_y = 0;
    int8_t send_x = 0 , send_y = 0;
    int8_t Repeat_Counter = 0,Last_Value_x = 0,Last_Value_y = 0;
    SharedState_t *state = (SharedState_t *)pvParameters;

   //按钮的均值滤波消抖
    uint8_t btn_history[3][BTN_HISTORY_SIZE] = {0}; // [L, R, M][history]
    uint8_t history_index = 0;
    bool stable_btn[3] = {1, 1, 1}; // 稳定后的状态
    
    //光电的均值滤波压制噪声,压制不住，硬件导致的鼠标在运行时会突然输出一连串的负值，似乎是芯片重启了
    // int8_t x_history[MOVE_HISTORY_SIZE] = {0};
    // int8_t y_history[MOVE_HISTORY_SIZE] = {0};
    // uint8_t move_index = 0;

    // 另一种延后发射数据的方法，应该能更好地滤除异常值
    typedef struct {
        int8_t x;
        int8_t y;
    } MoveData;
    MoveData move_buffer[MOVE_BUFFER_SIZE] = {0};
    uint8_t buffer_count = 0;  // 当前缓冲数据量
    uint8_t buffer_index = 0; // 当前写入位置


    while (1) 
    {
        //0000000000按键状态更新
        bool current_btn[3] = {
            gpio_get_level(L_CLICK),
            gpio_get_level(R_CLICK),
            gpio_get_level(M_CLICK)
        };

        // 更新历史记录（循环缓冲区）
        for(int i=0; i<3; i++) 
        {
            btn_history[i][history_index] = current_btn[i];
        }
        history_index = (history_index + 1) % BTN_HISTORY_SIZE;
        // 计算移动平均
        for(int i=0; i<3; i++) {
            uint16_t sum = 0;
            for(int j=0; j<BTN_HISTORY_SIZE; j++) 
            {
                sum += btn_history[i][j];
            }
            // 判断平均值是否低于阈值（0.5对应按下状态）
            stable_btn[i] = (sum * 10 / BTN_HISTORY_SIZE) >= 5; // 整数运算优化
        }


        //====== 组合按钮状态 ======//
        uint8_t button_state = 0;
        button_state |= (!stable_btn[0]) << 0; // 左键
        button_state |= (!stable_btn[1]) << 1; // 右键
        button_state |= (!stable_btn[2]) << 2; // 中键
        
        //0000000000移动状态更新
        PAW3805_Function(&local_raw_x, &local_raw_y);

        // 将新数据存入缓冲区
        move_buffer[buffer_index].x = local_raw_x;
        move_buffer[buffer_index].y = local_raw_y;
        buffer_index = (buffer_index + 1) % MOVE_BUFFER_SIZE;
        buffer_count = (buffer_count < MOVE_BUFFER_SIZE) ? (buffer_count + 1) : MOVE_BUFFER_SIZE;
        // 当缓冲区填满时进行检查
        if(buffer_count == MOVE_BUFFER_SIZE) 
        {
            // 检查最近5个数据是否相同
            bool all_same = true;
            int8_t first_x = move_buffer[0].x;
            int8_t first_y = move_buffer[0].y;
            
            for(int i=1; i<MOVE_BUFFER_SIZE; i++) 
            {
                if(move_buffer[i].x != first_x || move_buffer[i].y != first_y) 
                {
                    all_same = false;
                    break;
                }
            }

            if(all_same) 
            {
                // 数据异常，清空缓冲区并跳过发送
                buffer_count = 0;
                buffer_index = 0;
                memset(move_buffer, 0, sizeof(move_buffer));
                continue;
            }
            
            // 发送最旧数据（缓冲区第一个元素）
            send_x = move_buffer[0].x;
            send_y = move_buffer[0].y;
            
            // 移动缓冲区数据
            memmove(&move_buffer[0], &move_buffer[1], (MOVE_BUFFER_SIZE-1)*sizeof(MoveData));
            buffer_index = (buffer_index - 1) % MOVE_BUFFER_SIZE;
            buffer_count--;
        }
        else
        {
            continue;
        }
            // 计算移动平均值（带符号处理）-------弃用，引入延迟而且无法解决问题
        // int16_t x_sum = 0, y_sum = 0;
        // for(int i=0; i<MOVE_HISTORY_SIZE; i++) {
        //     x_sum += x_history[i];
        //     y_sum += y_history[i];
        // }
        // int8_t filtered_x = (int8_t)(x_sum / MOVE_HISTORY_SIZE);
        // int8_t filtered_y = (int8_t)(y_sum / MOVE_HISTORY_SIZE);

        IsStop = (send_x == 0) && (send_y == 0);
        state->IsMouseStop = IsStop;

        send_mouse_value(button_state,local_raw_x,local_raw_y);
        esp_rom_delay_us(750);
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
                vTaskDelay(pdMS_TO_TICKS(20));
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
    // 禁用晶振功能
    rtc_gpio_deinit(GPIO_NUM_16);
    // 配置为普通GPIO
    gpio_reset_pin(GPIO_NUM_16);

    spi_bus_config_t buscfg = {
        .miso_io_num = MISO,
        .mosi_io_num = MOSI,
        .sclk_io_num = SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0};
    spi_bus_initialize(FUKY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);// SPI总线初始化

    // 初始化 GPIO（如果还没初始化的话）
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << M_CLICK) | (1ULL << L_CLICK) | (1ULL << R_CLICK),  
        .mode = GPIO_MODE_INPUT,
        //.pull_up_en = GPIO_PULLUP_ENABLE,   // 启用上拉（可选）
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,     // 禁用中断
    };
    gpio_config(&in_conf);
    gpio_install_isr_service(0); //引脚中断服务
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
