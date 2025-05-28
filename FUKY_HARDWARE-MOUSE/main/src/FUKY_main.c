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
    bool IsMouseFloating;
    // 压敏电阻历史数据
    int adc_max_value;        // 历史最大值
    int adc_min_value;        // 历史最小值
    int adc_max_delta;        // 最大变化量
}
SharedState_t;

//static const char TAG[] = "FUKYMOUSE";


//======================================初始化====================================//
#define MOSI    14 
#define MISO    11
#define SCLK    12
#define PRESS   9 

#define M_CLICK    3                       
#define L_CLICK    5
#define R_CLICK    4
#define BTN_HISTORY_SIZE 5 //按键均值消抖
//#define MOVE_HISTORY_SIZE 5  // 移动均值消抖，5个样本够了-------弃用，增加延迟和粘滞手感
#define MOVE_BUFFER_SIZE 3    // 连续相同负值的判定阈值
#define FUKY_SPI_HOST    SPI2_HOST

SharedState_t shared_state = 
{
    .IsMouseStop = false,
    .IsMouseFloating = false,
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

//该计算方式已经验证过，没有问题，舒适化只要用户按压一下就完事了
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

// 计算压力值并映射到uint16_t全范围
uint16_t calculate_pressure_percentage(int adc_raw) {
    // 如果ADC值超出范围，返回0
    if (adc_raw > shared_state.adc_max_value || 
        adc_raw < shared_state.adc_min_value) {
        return 0;
    }
    // 计算相对于最小值的偏移量
    int delta = shared_state.adc_max_value - adc_raw;
    
    // 计算百分比并映射到uint16_t全范围 (0-65535)
    uint16_t mapped_value = (delta * 65535) / shared_state.adc_max_delta;
    
    return mapped_value;
}

// 读取压敏电阻的值并返回
uint16_t read_pressure_sensor(void) {

    int adc_raw = adc1_get_raw(ADC1_CHANNEL_1);
    
    // 更新历史值
    update_pressure_history(adc_raw);
    
    // 计算压力百分比
    uint16_t pressure_percentage = calculate_pressure_percentage(adc_raw);
    
    // 打印原始ADC值和压力百分比
    //printf("压敏电阻原始ADC值: %d, 压力百分比: %u\n", adc_raw, pressure_percentage);
    
    return pressure_percentage;
}

void Main_Init(void);
//最后决定用一个核心来跑光电
void MouseTask(void *pvParameters) 
{
    bool IsStop;//用于判断鼠标是不是停下了
    bool IsFirstDrop = false;
    int8_t local_raw_x = 0, local_raw_y = 0;
    SharedState_t *state = (SharedState_t *)pvParameters;
    //按钮逻辑
    //按钮的均值滤波消抖
    uint8_t btn_history[3][BTN_HISTORY_SIZE] = {0}; // [L, R, M][history]
    uint8_t history_index = 0;
    bool stable_btn[3] = {1, 1, 1}; // 稳定后的状态
    bool local_IsMouseFloating = false;

    // 压力数据相关变量
    TickType_t lastPressureSendTime = 0;
    const TickType_t pressureSendInterval = pdMS_TO_TICKS(20); // 20ms发送间隔
    uint32_t pressureSum = 0;
    uint16_t pressureSampleCount = 0;

    while (1) 
    {

        if (xSemaphoreTake(state->mutex, 0)) 
        {
             local_IsMouseFloating = state->IsMouseFloating;
            xSemaphoreGive(state->mutex);
        }

        //====== 按钮状态更新👇 ======//
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

        uint8_t button_state = 0;
        button_state |= (!stable_btn[0]) << 0; // 左键
        button_state |= (!stable_btn[1]) << 1; // 右键
        button_state |= (!stable_btn[2]) << 2; // 中键
        //====== 按钮状态更新👆 ======//
        //====== 移动状态更新👇 ======//
        PAW3805_Function(&local_raw_x, &local_raw_y);
        //ESP_LOGI("鼠标", "X=%d,     Y=%d",local_raw_x,local_raw_y);

        IsStop = (local_raw_x == 0) && (local_raw_y == 0);
        state->IsMouseStop = IsStop;
        //====== 移动状态更新👆 ======//
        //====== PRESSURE👇 ======//
        uint16_t currentPressure = read_pressure_sensor();
        pressureSum += currentPressure;
        pressureSampleCount++;
        TickType_t currentTime = xTaskGetTickCount();        // 检查是否达到发送时间
        if (currentTime - lastPressureSendTime >= pressureSendInterval) 
        {
            if (pressureSampleCount > 0) 
            {
                uint16_t avgPressure = pressureSum / pressureSampleCount;
                SendPressureData(avgPressure);
                // 重置累积器
                pressureSum = 0;
                pressureSampleCount = 0;
                lastPressureSendTime = currentTime;
            }
        }
        //====== PRESSURE👆 ======//
        uint16_t pressure = read_pressure_sensor();
        if(local_IsMouseFloating)
        {
            IsFirstDrop = true;
            // 根据 local_IsMouseFloating 设置第四位 (bit3)
            button_state |=  1 << 3;
            //ESP_LOGI("浮起状态", "按钮状态=%d (二进制: %04b)", button_state, button_state); // 添加日志
            SendButtonState(button_state);
            //vTaskDelay(pdMS_TO_TICKS(1));        //过频繁地访问光电会导致芯片重启和读数异常,因为burst不可用，目前回报率提不上来
            esp_rom_delay_us(800);
            continue;
        }

        button_state &= ~(1 << 3); // 如果是悬浮状态的话，清除bit3
        if(IsFirstDrop)
        {
            SendButtonState(0x00);
            IsFirstDrop = false;
        }
        // 判断左键是否按下（检查第0位）
        SendPressureData(pressure);
        //ESP_LOGI("正常时按键按下了", "=%d",button_state);
        send_mouse_value(button_state,local_raw_x,local_raw_y);
        //ESP_LOGI("按键", "=%d",button_state);
        //ESP_LOGI("浮起状态", "按钮状态=%d (二进制: %04b)", button_state, button_state); // 添加日志

        esp_rom_delay_us(800);
        // 正常就只用HID发送按钮
    }
        

}
// 压敏电阻读取任务，返回压力值
uint16_t PressureTask() 
{
        // 读取压敏电阻值，添加保护措施
        int adc_raw = adc1_get_raw(ADC1_CHANNEL_1);
        uint16_t pressure_value = 0;
        
        // 检查ADC值是否在有效范围内（0-4095）
        if (adc_raw >= 0 && adc_raw <= 4095) {
            // 更新历史值
            update_pressure_history(adc_raw);
            
            // 计算压力值
            pressure_value = calculate_pressure_percentage(adc_raw);
            
        } 
        else 
        {
            ESP_LOGE("PRESSURE", "读取到无效的ADC值");
        }
        
        return pressure_value;
}

void IMUTask(void *pvParameters) 
{
    bool ShouldDetect = true;
    //bool IsFloating = false;
    IMUData_t local_imu_data;
    SharedState_t *state = (SharedState_t *)pvParameters;
    
    // 初始化压力传感器参考值，确保最大值和最小值不同
    int initial_value = adc1_get_raw(ADC1_CHANNEL_1);
    state->adc_max_value = initial_value + 100; // 设置一个初始最大值
    state->adc_min_value = initial_value - 100; // 设置一个初始最小值
    state->adc_max_delta = 200; // 初始差值

    while (1) 
    {
        // 读取IMU数据,无论如何都得一直读取，不然数据会挤爆缓存
        local_imu_data = bno080_Function();
        // 移除这里的压力数据发送，只在MouseTask中发送
        // SendPressureData(PressureTask());

        //带锁访问共享状态
        if (xSemaphoreTake(state->mutex, 0)) 
        {
            if(!state->IsMouseStop) //如果光电还有读数，释放锁并延迟20ms再尝试读取,鼠标被判断为正常使用
            {
                //IsFloating =false;
                state->IsMouseFloating = false;
                ShouldDetect = true;//非鼠标停下，就是鼠标正在运动，这时将鼠标的停止检测标志设为未检测到停止
                xSemaphoreGive(state->mutex);//读取完鼠标是否停下状态后就可以释放锁了，以便鼠标核心写入鼠标状态 
                //vTaskDelay(pdMS_TO_TICKS(20));//不要延迟，不然bno080没刷新数据会陈旧
                continue;
            }
            xSemaphoreGive(state->mutex);//当读取状态为停下时时也是同理
        }
        
        if(ShouldDetect)
        {
            // IMU加速度求和，这里测试了静止时的鼠标读数，一般为20以下，少数时候会飙到30，极少数会到50
            //ESP_LOGI("IMU", "加速度 -   X=%d,     Y=%d,     Z=%d ",local_imu_data.lin_accel_x,local_imu_data.lin_accel_y,local_imu_data.lin_accel_z);
            // 如果加速度速度大于设定阈值，就判断鼠标浮起
            if(abs(local_imu_data.lin_accel_z) > 250)
            {
                //IsFloating = true; // Isfloating只会在光电有读数的时候被设为false
                ShouldDetect = false; // 一旦检测通过，后续鼠标在空中停下来也不会改变IsFloating状态,直到光电有读数
                ESP_LOGE("检测", "似乎抬起来了");
                continue;
            }
            continue;
            //ESP_LOGI("检测", "没抬起来");// 如果加速度速度小于设定阈值，就判断鼠标只是正常使用，继续检测
        }

        //ESP_LOGW("鼠标","悬浮模式");
        if (xSemaphoreTake(state->mutex, 0)) 
        {
            state->IsMouseFloating = true;
            xSemaphoreGive(state->mutex);
        }
        SendIMUData(local_imu_data.lin_accel_x, local_imu_data.lin_accel_y, local_imu_data.lin_accel_z, //加速度
            local_imu_data.quat_i, local_imu_data.quat_j, local_imu_data.quat_k, local_imu_data.quat_w);    //旋转

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
    
    xTaskCreatePinnedToCore(MouseTask, "MouseTask", 4096, &shared_state, 1, NULL, 0);

    xTaskCreatePinnedToCore(IMUTask, "IMUtask", 4096, &shared_state, 1, NULL, 1);

}



void Main_Init()
{
    // 禁用晶振功能
    //rtc_gpio_deinit(GPIO_NUM_16);
    // 配置为普通GPIO
    //gpio_reset_pin(GPIO_NUM_16);
    
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
    adc1_config_channel_atten(ADC1_CHANNEL_8, ADC_ATTEN_DB_11);
    
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
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,     // 禁用中断
    };
    gpio_config(&in_conf);
    gpio_install_isr_service(0); //引脚中断服务
}
