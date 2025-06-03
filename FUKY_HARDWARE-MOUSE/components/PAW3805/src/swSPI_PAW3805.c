#include "swSPI_PAW3805.h"

static spi_device_handle_t PAW3805ek_spi;

// =====================👇软件SPI实现，双向通信👇========================

void delay_ns(uint32_t ns) 
{
    uint32_t cycles = ns * (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ) / 1000;
    for (volatile int i = 0; i < cycles; i++);
}

// 软件 SPI 写字节
void sw_spi_write_byte(uint8_t data) {
    gpio_set_direction(OPT_SDIO, GPIO_MODE_OUTPUT);
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(OPT_SDIO, (data >> i) & 0x01);
        gpio_set_level(OPT_SCLK, 0);
        delay_ns(500);
        gpio_set_level(OPT_SCLK, 1);
        delay_ns(500);
    }
}

// 软件 SPI 读字节
uint8_t sw_spi_read_byte() {
    uint8_t data = 0;
    gpio_set_direction(OPT_SDIO, GPIO_MODE_INPUT);
    gpio_set_level(OPT_SCLK, 1);
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(OPT_SCLK, 0);
        delay_ns(500);
        gpio_set_level(OPT_SCLK, 1);
        data |= (gpio_get_level(OPT_SDIO) << i);
        delay_ns(500);
    }
    return data;
}

// 写寄存器
void write_register(uint8_t addr, uint8_t value) {
    gpio_set_level(OPT_CS, 0);
    sw_spi_write_byte(addr | 0x80); // 写操作地址
    sw_spi_write_byte(value);
    gpio_set_level(OPT_CS, 1);
}

// 读寄存器
uint8_t read_register(uint8_t addr) {
    gpio_set_level(OPT_CS, 0);
    sw_spi_write_byte(addr & 0x7F); // 读操作地址 确保 MSB = 0
    esp_rom_delay_us(1);
    uint8_t data = sw_spi_read_byte();
    gpio_set_level(OPT_CS, 1);
    return data;
}

// =====================👆软件SPI实现，双向通信👆========================

// 传感器初始化
void init_paw3805ek(spi_host_device_t HOST) 
{
    // 引脚配置
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OPT_CS) | (1ULL << OPT_SCLK) | (1ULL << REG_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(OPT_CS, 1);
    gpio_set_level(OPT_SCLK, 1);
    gpio_set_level(REG_RST, 0);

    gpio_set_direction(OPT_SDIO, GPIO_MODE_OUTPUT);
    gpio_set_level(OPT_SDIO, 1);
    esp_rom_delay_us(500);
    ESP_LOGI("PAW3805EK", "====== Initialization ======");
    ESP_LOGI("PAW3805EK", "PIN STATES: CS= %d" , gpio_get_level(OPT_CS));
    ESP_LOGI("PAW3805EK", "PIN STATES: CS= %d" , gpio_get_level(OPT_SCLK));
    ESP_LOGI("PAW3805EK", "PIN STATES: CS= %d" , gpio_get_level(REG_RST));

    // 硬件复位
    gpio_set_level(REG_RST, 1);
    esp_rom_delay_us(500);
    gpio_set_level(REG_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 软复位
    write_register(REG_CONFIG, 0x80);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 验证复位状态
    uint8_t post_reset = read_register(REG_CONFIG);
    ESP_LOGI("PAW3805EK","Post-reset REG_CONFIG: 0x%02X" ,post_reset);
    if ((post_reset & 0x80) != 0) 
    {
    ESP_LOGI("PAW3805EK","Error: Reset bit still set!");
    }

    // 解锁写保护
    write_register(REG_WRITEPROT, 0x5A);
    esp_rom_delay_us(10);

    // 配置方向和 CPI
    write_register(REG_ORIENTATION, 0x11);
    write_register(REG_CPIX, 0x4C);  // 2000 CPI X
    write_register(REG_CPIY, 0x54);  // 2000 CPI Y

    // 设置操作模式
    write_register(REG_OPMODE, 0xA1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI("PAW3805EK", "Initialization complete");
}




// 读取运动数据
void read_motion_data(int16_t *dx, int16_t *dy) 
{
    uint8_t status = read_register(REG_MOTION_STATUS);
    if (!(status & 0x80)) 
    {
        *dx = 0;
        *dy = 0;
        return;
    }

    int8_t x_lo = (int8_t)read_register(REG_XLO);
    int8_t y_lo = (int8_t)read_register(REG_YLO);
    int8_t x_hi = (int8_t)read_register(REG_XHI);
    int8_t y_hi = (int8_t)read_register(REG_YHI);

    *dx = (x_hi << 8) | (uint8_t)x_lo;
    *dy = (y_hi << 8) | (uint8_t)y_lo;
    //ESP_LOGI("Motion", "dx=%d, dy=%d", *dx, *dy);

}

// 读取运动数据
void read_motion_data_8bit(int8_t *dx, int8_t *dy) 
{
    uint8_t status = read_register(REG_MOTION_STATUS);
    if (!(status & 0x80)) 
    {
        *dx = 0;
        *dy = 0;
        return;
    }

    int8_t x_lo = (int8_t)read_register(REG_XLO);
    int8_t y_lo = (int8_t)read_register(REG_YLO);

    *dx = x_lo;
    *dy = y_lo;
    //ESP_LOGI("Motion", "dx=%d, dy=%d", *dx, *dy);
}



void PAW3805_Function(int16_t *dx_output, int16_t *dy_output)
{    
    //read_motion_data(dx_output, dy_output);
    read_motion_data_8bit(dx_output,dy_output);
}