//#include "spi_PAW3395.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/rtc_io.h"
#include "swSPI_PAW3805.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "bno080.h"
#include "ble_hidd.h"
#include "esp_timer.h"
#include "FUKY_main.h"
//共享数据结构
typedef struct 
{
    SemaphoreHandle_t mutex;  // 互斥锁句柄
    bool IsMouseStop;
    
    // 压敏电阻历史数据
    int adc_max_value;        // 历史最大值
    int adc_min_value;        // 历史最小值
    int adc_max_delta;        // 最大变化量
}
SharedState_t;

//static const char TAG[] = "FUKYMOUSE";


//======================================初始化====================================//
#define MOSI    6
#define MISO    3
#define SCLK    4
#define PRESS   2

#define M_CLICK    18
#define L_CLICK    16
#define R_CLICK    17
#define BTN_HISTORY_SIZE 20 //按键均值消抖
//#define MOVE_HISTORY_SIZE 5  // 移动均值消抖，5个样本够了-------弃用，增加延迟和粘滞手感
#define MOVE_BUFFER_SIZE 3    // 连续相同负值的判定阈值
#define FUKY_SPI_HOST    SPI2_HOST

SharedState_t shared_state = 
{
    .IsMouseStop = false,
    .mutex = NULL,
    .adc_max_value = 0,
    .adc_min_value = 4095,
    .adc_max_delta = 1,

};

//int16_t delta_x, delta_y;

// ESP32S3 GPIO到ADC通道的映射关系：
// GPIO 1 -> ADC1_CH0
// GPIO 2 -> ADC1_CH1
// GPIO 3 -> ADC1_CH2
// GPIO 4 -> ADC1_CH3
// GPIO 5 -> ADC1_CH4
// GPIO 6 -> ADC1_CH5
// GPIO 7 -> ADC1_CH6
// GPIO 8 -> ADC1_CH7
// GPIO 9 -> ADC1_CH8
// GPIO 10 -> ADC1_CH9
// 可以在ESP-IDF文档中查找：https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/adc.html

// 更新压敏电阻历史值
void update_pressure_history(int adc_raw) {
    // 忽略明显异常的值
    if (adc_raw < 0 || adc_raw > 4095) {
        return;
    }
    
    // 更新最大值
    if (adc_raw > shared_state.adc_max_value) {
        shared_state.adc_max_value = adc_raw;
    }
    
    // 更新最小值
    if (adc_raw < shared_state.adc_min_value) {
        shared_state.adc_min_value = adc_raw;
    }
    
    // 更新最大变化量
    shared_state.adc_max_delta = shared_state.adc_max_value - shared_state.adc_min_value;
    if (shared_state.adc_max_delta < 1) {
        shared_state.adc_max_delta = 1; // 防止除以零
    }
}

// 计算压力百分比
uint32_t calculate_pressure_percentage(int adc_raw) {
    // 如果ADC值超出范围，返回0
    if (adc_raw > shared_state.adc_max_value || 
        adc_raw < shared_state.adc_min_value) {
        return 0;
    }
    
    // 计算相对于最小值的偏移量
    int delta = shared_state.adc_max_value - adc_raw;
    
    // 计算百分比 (0-100%)
    uint32_t percentage = (delta * 100) / shared_state.adc_max_delta;
    
    // 限制最大值为100
    if (percentage > 100) {
        percentage = 100;
    }
    
    return percentage;
}

// 读取压敏电阻的值并返回
uint32_t read_pressure_sensor(void) {
    // 读取ADC原始值 - 使用ADC1_CHANNEL_1，对应GPIO 2
    int adc_raw = adc1_get_raw(ADC1_CHANNEL_1); // PRESS引脚对应的ADC通道
    
    // 更新历史值
    update_pressure_history(adc_raw);
    
    // 计算压力百分比
    uint32_t pressure_percentage = calculate_pressure_percentage(adc_raw);
    
    // 打印原始ADC值和压力百分比
    printf("压敏电阻原始ADC值: %d, 压力百分比: %lu%%\n", adc_raw, pressure_percentage);
    
    return adc_raw;
}

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
    #if DATA_POST_PROCESSING_ENABLED
    typedef struct {
        int8_t x;
        int8_t y;
    } MoveData;
    MoveData move_buffer[MOVE_BUFFER_SIZE] = {0};
    uint8_t buffer_count = 0;  // 当前缓冲数据量
    uint8_t buffer_index = 0; // 当前写入位置
    #endif

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
        
#if DATA_POST_PROCESSING_ENABLED
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
        IsStop = (send_x == 0) && (send_y == 0);
#else
        /* 原始数据直通模式 */
        send_x = local_raw_x;
        send_y = local_raw_y;
        IsStop = (local_raw_x == 0) && (local_raw_y == 0);
#endif
        state->IsMouseStop = IsStop;
        send_mouse_value(button_state,local_raw_x,local_raw_y);
        esp_rom_delay_us(800);//过频繁地访问光电会导致芯片重启和读数异常,因为burst不可用，目前回报率提不上来
    }
        

}
// 压敏电阻读取任务
void PressureTask() 
{
        // 读取压敏电阻值，添加保护措施
        int adc_raw = adc1_get_raw(ADC1_CHANNEL_1);
        
        // 检查ADC值是否在有效范围内（0-4095）
        if (adc_raw >= 0 && adc_raw <= 4095) {
            // 更新历史值
            update_pressure_history(adc_raw);
            
            // 计算压力百分比
            uint32_t pressure_percentage = calculate_pressure_percentage(adc_raw);
            
            // 使用ESP_LOGI代替printf，更安全且支持格式化输出
            ESP_LOGI("PRESSURE", "压敏电阻原始ADC值: %d, 压力百分比: %lu%%, 最大值: %d, 最小值: %d, 变化量: %d", 
                    adc_raw, pressure_percentage, shared_state.adc_max_value, shared_state.adc_min_value, shared_state.adc_max_delta);
            
            // 这里可以添加代码，根据pressure_percentage执行相应的操作
            // 例如调整鼠标DPI、发送特殊按键等
        } else {
            ESP_LOGE("PRESSURE", "读取到无效的ADC值");
        }
}

void IMUTask(void *pvParameters) 
{
    IMUData_t local_imu_data;
     IMUData_t last_sent_data = {0};
    SharedState_t *state = (SharedState_t *)pvParameters;
    bool ShouldSendIMUData;
    
   // 初始化
    state->adc_max_value = adc1_get_raw(ADC1_CHANNEL_1); // PRESS引脚对应的ADC通道
    state->adc_min_value = adc1_get_raw(ADC1_CHANNEL_1);
    while (1) 
    {
        ShouldSendIMUData = false; 
        //带锁访问共享状态
        if (xSemaphoreTake(state->mutex, 0)) 
        {
            ShouldSendIMUData = state->IsMouseStop;
            if(!state->IsMouseStop)
            {
                xSemaphoreGive(state->mutex);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            xSemaphoreGive(state->mutex);
        }
        if(ShouldSendIMUData)
        {
            // 防止发送相同数据,几乎不会出现完全相同的imu和加速度数据
            if(
                local_imu_data.lin_accel_x != last_sent_data.lin_accel_x ||
                local_imu_data.lin_accel_y != last_sent_data.lin_accel_y ||
                local_imu_data.lin_accel_z != last_sent_data.lin_accel_z ||
                local_imu_data.quat_i != last_sent_data.quat_i ||
                local_imu_data.quat_j != last_sent_data.quat_j ||
                local_imu_data.quat_k != last_sent_data.quat_k ||
                local_imu_data.quat_w != last_sent_data.quat_w
            )
            {
                local_imu_data = bno080_Function();
                SendIMUData(local_imu_data.lin_accel_x, local_imu_data.lin_accel_y, local_imu_data.lin_accel_z, local_imu_data.quat_i, local_imu_data.quat_j, local_imu_data.quat_k, local_imu_data.quat_w);    
            }
            // 更新最后发送数据
            PressureTask();
            memcpy(&last_sent_data, &local_imu_data, sizeof(IMUData_t));
            //esp_rom_delay_us(500);//暂时加点延迟，到时候发现回报率还是不够再搞掉
            vTaskDelay(pdMS_TO_TICKS(1));
            //ESP_LOGI("IMU","数据");
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

    // 读取一次压敏电阻的值并打印
    printf("读取压敏电阻...\n");
    uint32_t pressure_value = read_pressure_sensor();
    printf("压敏电阻值: %lu\n", pressure_value);
    

    // 尝试提高鼠标的频率
    xTaskCreatePinnedToCore(MouseTask, "MouseTask", 4096, &shared_state, 1, NULL, 0);
    // xTaskCreatePinnedToCore(IMUTask_0, "ReadDataTask", 4096, &shared_state, 0, NULL, 0);
    // 将SendDatatask绑定到core1
    xTaskCreatePinnedToCore(IMUTask, "IMUtask", 4096, &shared_state, 1, NULL, 1);
    // 创建一个任务来定期读取压敏电阻的值
    //MouseTask(&shared_state);

}



void Main_Init()
{
    // 禁用晶振功能
    rtc_gpio_deinit(GPIO_NUM_16);
    // 配置为普通GPIO
    gpio_reset_pin(GPIO_NUM_16);
    
    // 确保PRESS引脚（GPIO 2）配置为ADC模式
    gpio_reset_pin(PRESS);
    
    // 配置GPIO 2为模拟输入模式，禁用其他功能
    gpio_config_t adc_gpio_config = {
        .pin_bit_mask = (1ULL << PRESS),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // 禁用上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用下拉
        .intr_type = GPIO_INTR_DISABLE,      // 禁用中断
    };
    gpio_config(&adc_gpio_config);
    
    // 初始化ADC
    adc1_config_width(ADC_WIDTH_BIT_12); // 设置ADC分辨率为12位
    
    // 尝试不同的衰减设置，可能需要根据压敏电阻的特性调整
    // ADC_ATTEN_DB_0: 满量程电压 1.1V
    // ADC_ATTEN_DB_2_5: 满量程电压 1.5V
    // ADC_ATTEN_DB_6: 满量程电压 2.2V
    // ADC_ATTEN_DB_11: 满量程电压 3.9V
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
    
    // 等待ADC稳定
    vTaskDelay(pdMS_TO_TICKS(10));

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
