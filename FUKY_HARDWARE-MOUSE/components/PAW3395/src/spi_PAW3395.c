#include "spi_PAW3395.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdint.h>

static spi_device_handle_t PAW_spi;
static const char TAG[] = "PAW3395";


void (*LclickFunc)(void);
void (*RclickFunc)(void);
// 初始化
void PAW3395_init(spi_host_device_t HOST)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLK_SPEED,
        .mode = 3,  // SPI mode 0
        .spics_io_num = -1,
        .queue_size = 7,
    };
    spi_bus_add_device(HOST, &devcfg, &PAW_spi);
    
    //CS引脚初始化
    gpio_set_level(PAW_CS, 1);
    gpio_config_t CS_cfg = {
        .pin_bit_mask = BIT64(PAW_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&CS_cfg);
    //NRESET引脚初始化
    gpio_set_level(PAW_NRESET, 1);
    gpio_config_t NRESET_cfg = {
        .pin_bit_mask = BIT64(PAW_NRESET),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&NRESET_cfg);
    //MOTION引脚初始化
    gpio_config_t MOTION_cfg = {
        .pin_bit_mask = BIT64(PAW_MOTION),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&MOTION_cfg);
    //鼠标按键初始化
    gpio_config_t Click_cfg = {
        .pin_bit_mask = (1ULL << LClick) | (1ULL << RClick),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        //.intr_type = GPIO_INTR_POSEDGE  // 按下时上升沿
    };
    gpio_config(&Click_cfg);

    // 重置芯片
    ResetChip();
    esp_rom_delay_us(50000);//等待电压稳定
    Power_up_sequence();
}

// Paw3395上电初始化,按照Datasheet进行操作
void Power_up_sequence(void)
{
    ESP_LOGI(TAG, "开始初始化PAW3395");
    uint8_t reg_it;
    esp_rom_delay_us(50000);//等待电压稳定
    //拉高拉低下CS，确保正常
    gpio_set_level(PAW_CS, 0);
    esp_rom_delay_us(1);
    gpio_set_level(PAW_CS, 1);
    esp_rom_delay_us(1);
    gpio_set_level(PAW_CS, 0);
    
    // Write 0x5A to POWER_UP_Reset register（4.将0x5A写入Power_Up_Reset寄存器）
    write_register(REG_POWERUP_RESET,POWERUP_RESET);
    esp_rom_delay_us(5000);//等待至少5ms

	//Load Power-up initialization register setting（6.加载上电初始化寄存器设置）
    Power_Up_Initializaton_Register_Setting();
    gpio_set_level(PAW_CS, 1);
    esp_rom_delay_us(1);

    // read register from 0x02 to 0x06（7.无论运动位状态如何，都读取寄存器0x02、0x03、0x04、0x05和0x06一次）
	for(reg_it=0x02; reg_it<=0x06; reg_it++)
	{	
		read_register(reg_it);
        esp_rom_delay_us(2);  
	}
	//片选拉高，本次SPI通讯结束
    gpio_set_level(PAW_CS, 1);
    esp_rom_delay_us(1);
}
// SPI发送接收函数
uint8_t SPI_SendReceive(uint8_t data)
{
    uint8_t rx_data = 0;
    spi_transaction_t t = {
        .length = 8,        // 8位数据
        .tx_buffer = &data,
        .rx_buffer = &rx_data
    };
    
    spi_device_transmit(PAW_spi, &t);
    return rx_data;
}

/**
 * @brief 读寄存器
 * @param address 地址，如0x5B等
 * @return 读取值，如0x20等
 */
uint8_t read_register(uint8_t address)
{
    uint8_t temp;
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,  // 8位地址 + 8位数据
        .tx_data = {0x00 | (address & ADDR_MASK)},  // 读命令（地址+0x00）
        .rx_data = {0}
    };
    gpio_set_level(PAW_CS, 0);
    esp_rom_delay_us(1);
    spi_device_polling_transmit(PAW_spi, &t);
    gpio_set_level(PAW_CS, 1);
    esp_rom_delay_us(5);
    temp = t.rx_data[1];
    return temp;
}

/**
 * @brief 写寄存器
 * @param address 地址，如0x5B等
 * @param value 值，如0x20等
 */
void write_register(uint8_t address, uint8_t value)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,  // 8位地址 + 8位数据
        .tx_data = {0x80 | (address & ADDR_MASK) , value}  // 写命令（地址+0x80）
    };
    gpio_set_level(PAW_CS, 0);
    esp_rom_delay_us(1);
    spi_device_polling_transmit(PAW_spi, &t);
    gpio_set_level(PAW_CS, 1);
    esp_rom_delay_us(5);

}
/*
 *上电初始化寄存器设置
 */
void Power_Up_Initializaton_Register_Setting(void)
{
	uint8_t read_tmp;
	uint8_t i ;
	write_register(0x7F ,0x07);
	write_register(0x40 ,0x41);
	write_register(0x7F ,0x00);
	write_register(0x40 ,0x80);
	write_register(0x7F ,0x0E);
	write_register(0x55 ,0x0D);
	write_register(0x56 ,0x1B);
	write_register(0x57 ,0xE8);
	write_register(0x58 ,0xD5);
	write_register(0x7F ,0x14);
	write_register(0x42 ,0xBC);
	write_register(0x43 ,0x74);
	write_register(0x4B ,0x20);
	write_register(0x4D ,0x00);
	write_register(0x53 ,0x0E);
	write_register(0x7F ,0x05);
	write_register(0x44 ,0x04);
	write_register(0x4D ,0x06);
	write_register(0x51 ,0x40);
	write_register(0x53 ,0x40);
	write_register(0x55 ,0xCA);
	write_register(0x5A ,0xE8);
	write_register(0x5B ,0xEA);
	write_register(0x61 ,0x31);
	write_register(0x62 ,0x64);
	write_register(0x6D ,0xB8);
	write_register(0x6E ,0x0F);

	write_register(0x70 ,0x02);
	write_register(0x4A ,0x2A);
	write_register(0x60 ,0x26);
	write_register(0x7F ,0x06);
	write_register(0x6D ,0x70);
	write_register(0x6E ,0x60);
	write_register(0x6F ,0x04);
	write_register(0x53 ,0x02);
	write_register(0x55 ,0x11);
	write_register(0x7A ,0x01);
	write_register(0x7D ,0x51);
	write_register(0x7F ,0x07);
	write_register(0x41 ,0x10);
	write_register(0x42 ,0x32);
	write_register(0x43 ,0x00);
	write_register(0x7F ,0x08);
	write_register(0x71 ,0x4F);
	write_register(0x7F ,0x09);
	write_register(0x62 ,0x1F);
	write_register(0x63 ,0x1F);
	write_register(0x65 ,0x03);
	write_register(0x66 ,0x03);
	write_register(0x67 ,0x1F);
	write_register(0x68 ,0x1F);
	write_register(0x69 ,0x03);
	write_register(0x6A ,0x03);
	write_register(0x6C ,0x1F);

	write_register(0x6D ,0x1F);
	write_register(0x51 ,0x04);
	write_register(0x53 ,0x20);
	write_register(0x54 ,0x20);
	write_register(0x71 ,0x0C);
	write_register(0x72 ,0x07);
	write_register(0x73 ,0x07);
	write_register(0x7F ,0x0A);
	write_register(0x4A ,0x14);
	write_register(0x4C ,0x14);
	write_register(0x55 ,0x19);
	write_register(0x7F ,0x14);
	write_register(0x4B ,0x30);
	write_register(0x4C ,0x03);
	write_register(0x61 ,0x0B);
	write_register(0x62 ,0x0A);
	write_register(0x63 ,0x02);
	write_register(0x7F ,0x15);
	write_register(0x4C ,0x02);
	write_register(0x56 ,0x02);
	write_register(0x41 ,0x91);
	write_register(0x4D ,0x0A);
	write_register(0x7F ,0x0C);
	write_register(0x4A ,0x10);
	write_register(0x4B ,0x0C);
	write_register(0x4C ,0x40);
	write_register(0x41 ,0x25);
	write_register(0x55 ,0x18);
	write_register(0x56 ,0x14);
	write_register(0x49 ,0x0A);
	write_register(0x42 ,0x00);
	write_register(0x43 ,0x2D);
	write_register(0x44 ,0x0C);
	write_register(0x54 ,0x1A);
	write_register(0x5A ,0x0D);
	write_register(0x5F ,0x1E);
	write_register(0x5B ,0x05);
	write_register(0x5E ,0x0F);
	write_register(0x7F ,0x0D);
	write_register(0x48 ,0xDD);
	write_register(0x4F ,0x03);
	write_register(0x52 ,0x49);
		
	write_register(0x51 ,0x00);
	write_register(0x54 ,0x5B);
	write_register(0x53 ,0x00);
		
	write_register(0x56 ,0x64);
	write_register(0x55 ,0x00);
	write_register(0x58 ,0xA5);
	write_register(0x57 ,0x02);
	write_register(0x5A ,0x29);
	write_register(0x5B ,0x47);
	write_register(0x5C ,0x81);
	write_register(0x5D ,0x40);
	write_register(0x71 ,0xDC);
	write_register(0x70 ,0x07);
	write_register(0x73 ,0x00);
	write_register(0x72 ,0x08);
	write_register(0x75 ,0xDC);
	write_register(0x74 ,0x07);
	write_register(0x77 ,0x00);
	write_register(0x76 ,0x08);
	write_register(0x7F ,0x10);
	write_register(0x4C ,0xD0);
	write_register(0x7F ,0x00);
	write_register(0x4F ,0x63);
	write_register(0x4E ,0x00);
	write_register(0x52 ,0x63);
	write_register(0x51 ,0x00);
	write_register(0x54 ,0x54);
	write_register(0x5A ,0x10);
	write_register(0x77 ,0x4F);
	write_register(0x47 ,0x01);
	write_register(0x5B ,0x40);
	write_register(0x64 ,0x60);
	write_register(0x65 ,0x06);
	write_register(0x66 ,0x13);
	write_register(0x67 ,0x0F);
	write_register(0x78 ,0x01);
	write_register(0x79 ,0x9C);
	write_register(0x40 ,0x00);
	write_register(0x55 ,0x02);
	write_register(0x23 ,0x70);
	write_register(0x22 ,0x01);

	//Wait for 1ms
	esp_rom_delay_us(1000);
	
	for( i = 0 ;i < 60 ;i++)
	{
		read_tmp = read_register(0x6C);
		if(read_tmp == 0x80 )
        {
            ESP_LOGI(TAG, "读取到0x80");
            break;
        }
	    esp_rom_delay_us(1000);
	}
	if(i == 60)
	{
		write_register(0x7F ,0x14);
		write_register(0x6C ,0x00);
		write_register(0x7F ,0x00);
	}
	write_register(0x22 ,0x00);
	write_register(0x55 ,0x00);
	write_register(0x7F ,0x07);
	write_register(0x40 ,0x40);
	write_register(0x7F ,0x00);
    ESP_LOGI(TAG, "初始化完成");
}
/**
 * @brief 配置芯片运行模式
 * @param mode 要设置的模式（High_Performance/LowPower/Office/Corded_Gaming）
 * @return esp_err_t 返回ESP_OK表示成功，ESP_FAIL表示参数错误
 */
esp_err_t set_chip_mode(ChipMode mode)
{
    uint8_t reg40_value;
    switch (mode) {
        case High_Performance:
            // TODO: 实现高性能模式寄存器配置
            write_register(0x7F ,0x05);
            write_register(0x51 ,0x40);
            write_register(0x53 ,0x40);
            write_register(0x61 ,0x31);
            write_register(0x6E ,0x0F);
            write_register(0x7F ,0x07);
            write_register(0x42 ,0x32);
            write_register(0x43 ,0x00);
            write_register(0x7F ,0x0D);
            write_register(0x51 ,0x00);
            write_register(0x52 ,0x49);
            write_register(0x53 ,0x00);
            write_register(0x54 ,0x5B);
            write_register(0x55 ,0x00);
            write_register(0x56 ,0x64);
            write_register(0x57 ,0x02);
            write_register(0x58 ,0xA5);
            write_register(0x7F ,0x00);
            write_register(0x54 ,0x54);
            write_register(0x78 ,0x01);
            write_register(0x79 ,0x9C);
            //读取当前寄存器值（避免覆盖保留位）
            reg40_value = read_register(0x40);
            //使用位掩码清除 bit[1:0]
            reg40_value &= 0xFC;  // 0xFC = 11111100（清除最低两位）
            //写回修改后的值
            write_register(0x40, reg40_value);
            break;

        case LowPower:
            write_register(0x7F ,0x05);
            write_register(0x51 ,0x40);
            write_register(0x53 ,0x40);
            write_register(0x61 ,0x3B);
            write_register(0x6E ,0x1F);
            write_register(0x7F ,0x07);
            write_register(0x42 ,0x32);
            write_register(0x43 ,0x00);
            write_register(0x7F ,0x0D);
            write_register(0x51 ,0x00);
            write_register(0x52 ,0x49);
            write_register(0x53 ,0x00);
            write_register(0x54 ,0x5B);
            write_register(0x55 ,0x00);
            write_register(0x56 ,0x64);
            write_register(0x57 ,0x02);
            write_register(0x58 ,0xA5);
            write_register(0x7F ,0x00);
            write_register(0x54 ,0x54);
            write_register(0x7F ,0x00);
            write_register(0x54 ,0x54);
            write_register(0x78 ,0x01);
            write_register(0x79 ,0x9C);
            //读取当前寄存器值（避免覆盖保留位）
            reg40_value = read_register(0x40);
            //使用位掩码清除 bit[1:0]
            reg40_value &= 0xFC;  // 0xFC = 11111100（清除最低两位）
            //写回修改后的值
            write_register(0x40, reg40_value | 0X01);
            break;

        case Office:
            // TODO: 实现办公模式寄存器配置
            write_register(0x7F ,0x05);
            write_register(0x51 ,0x28);
            write_register(0x53 ,0x30);
            write_register(0x61 ,0x3B);
            write_register(0x6E ,0x1F);
            write_register(0x7F ,0x07);
            write_register(0x42 ,0x32);
            write_register(0x43 ,0x00);
            write_register(0x7F ,0x0D);
            write_register(0x51 ,0x00);
            write_register(0x52 ,0x49);
            write_register(0x53 ,0x00);
            write_register(0x54 ,0x5B);
            write_register(0x55 ,0x00);
            write_register(0x56 ,0x64);
            write_register(0x57 ,0x02);
            write_register(0x58 ,0xA5);
            write_register(0x7F ,0x00);
            write_register(0x54 ,0x52);
            write_register(0x7F ,0x00);
            write_register(0x54 ,0x54);
            write_register(0x78 ,0x0A);
            write_register(0x79 ,0x0F);
            //读取当前寄存器值（避免覆盖保留位）
            reg40_value = read_register(0x40);
            //使用位掩码清除 bit[1:0]
            reg40_value &= 0xFC;  // 0xFC = 11111100（清除最低两位）
            //写回修改后的值
            write_register(0x40, reg40_value | 0X02);
            break;

        case Corded_Gaming:
            // TODO: 实现有线游戏模式寄存器配置
            write_register(0x7F ,0x05);
            write_register(0x51 ,0x40);
            write_register(0x53 ,0x40);
            write_register(0x61 ,0x31);
            write_register(0x6E ,0x0F);
            write_register(0x7F ,0x07);
            write_register(0x42 ,0x2F);
            write_register(0x43 ,0x00);
            write_register(0x7F ,0x0D);
            write_register(0x51 ,0x12);
            write_register(0x52 ,0xDB);
            write_register(0x53 ,0x12);
            write_register(0x54 ,0xDC);
            write_register(0x55 ,0x12);
            write_register(0x56 ,0xEA);
            write_register(0x57 ,0x15);
            write_register(0x58 ,0x2D);
            write_register(0x7F ,0x00);
            write_register(0x54 ,0x55);
            reg40_value = read_register(0x40);
            reg40_value = (reg40_value & 0xFC) | 0x83; // bit[1:0]=11
            write_register(0x40, reg40_value);
            break;

        default:
            ESP_LOGE("MODE", "无效的模式参数: %d", mode);
            return ESP_FAIL;
    }
    
    ESP_LOGI("MODE", "模式切换至: %d 完成", mode);
    return ESP_OK;
}

void ResetChip(void)
{
    gpio_set_level(PAW_NRESET, 0);
    esp_rom_delay_us(1);
    gpio_set_level(PAW_NRESET, 1);
    esp_rom_delay_us(1000); // 等待芯片复位完成
}

uint8_t* Motion_Burst()
{
    // 定义要发送的地址字节
    uint8_t tx_address = 0x00 | (0x16 & ADDR_MASK);
    // 动态分配12字节的接收缓冲区
    uint8_t *data_RX = (uint8_t *)malloc(12 * sizeof(uint8_t));
    if (data_RX == NULL)
    {
        printf("Memory allocation failed!\n");
        return NULL;  // 内存分配失败
    }

    // 配置SPI事务
    spi_transaction_t t = {
        .flags = 0,
        .rxlength = 96,               // 接收96位数据
        .length = 8 + 96,             // 发送8位地址 + 接收96位数据
        .tx_buffer  = &tx_address,
        .rx_buffer = data_RX          // 接收缓冲区
    };

    // 拉低CS引脚（开始SPI通信）
    gpio_set_level(PAW_CS, 0);
    esp_rom_delay_us(1);             // 等待t(NCS-SCLK)

    // SPI传输
    esp_err_t ret = spi_device_polling_transmit(PAW_spi, &t);
    if (ret != ESP_OK)
    {
        printf("SPI transaction failed!\n");
        gpio_set_level(PAW_CS, 1);   // 结束SPI通信
        free(data_RX);               // 释放内存
        return NULL;
    }

    // 拉高CS引脚（结束SPI通信）
    gpio_set_level(PAW_CS, 1);
    esp_rom_delay_us(1);

    return data_RX;  // 返回接收数据指针
}

int8_t  GetButtonState()
{
    int8_t mousebutton = 0;
    if (gpio_get_level(LClick) == 1) 
    {
    mousebutton |= 0x01;
    }
    if (gpio_get_level(RClick) == 1) 
    {
    mousebutton |= 0x02;
    }
    //ESP_LOGI("鼠标功能", "按下按钮: %x", mousebutton);
    return mousebutton;
}
