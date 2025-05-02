#include <inttypes.h> //这头文件是为了能打印32位int调试信息
#include <string.h> 
#include "bno080.h"
#include "sh2.h"
#include "esp_log.h"
#include <stdio.h>
#include "esp_timer.h"


spi_device_handle_t PAW_bno080;

sh2_ProductIds_t prodIds;
bool _reset_occurred = false;
sh2_SensorValue_t *_sensor_value = NULL;
sh2_Hal_t _HAL;
/****************************************
***************************************** HAL SPI interface
*****************************************
*****************************************/
uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);  // 转换为毫秒
}

/*!
 *    @brief  将SPI读取到的设备数据存入指定缓冲区
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  sendvalue The 8-bits of data to write when doing the data read,
 * defaults to 0xFF
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */
bool spi_read(uint8_t *buffer, size_t len, uint8_t sendvalue) {
    if (!buffer || len == 0) {
        ESP_LOGE("BNO080初始化", "spi_read: Invalid parameters");
        return false;
    }

    ESP_LOGE("BNO080初始化", "spi_read: Starting read operation, len=%d", len);

    // 准备发送缓冲区
    uint8_t *tx_buffer = heap_caps_malloc(len, MALLOC_CAP_DMA);
    if (!tx_buffer) {
        ESP_LOGE("BNO080初始化", "spi_read: Failed to allocate TX buffer");
        return false;
    }
    memset(tx_buffer, sendvalue, len);

    // 配置SPI事务
    spi_transaction_t t = {
        .length = len * 8,
        .rxlength = len * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = buffer,
        .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST
    };

    // 等待INTN变为低电平
    uint32_t start_time = millis();
    while(gpio_get_level(BNO_INTN)) {
        if(millis() - start_time > 50) { // 增加超时时间到50ms
            ESP_LOGE("BNO080初始化", "spi_read: INTN timeout");
            free(tx_buffer);
            return false;
        }
        vTaskDelay(1);
    }

    // CS拉低前等待
    esp_rom_delay_us(10);
    gpio_set_level(BNO_CS, 0);
    esp_rom_delay_us(10);

    // 执行SPI传输
    esp_err_t ret = spi_device_transmit(PAW_bno080, &t);

    // 等待传输完成
    esp_rom_delay_us(10);

    // CS拉高
    gpio_set_level(BNO_CS, 1);
    esp_rom_delay_us(10);

    // 等待INTN恢复高电平
    start_time = millis();
    while(!gpio_get_level(BNO_INTN)) {
        if(millis() - start_time > 50) {
            ESP_LOGE("BNO080初始化", "spi_read: INTN not restored");
            break;
        }
        vTaskDelay(1);
    }

    // 打印调试信息
    if (ret == ESP_OK) {
        ESP_LOGE("BNO080初始化", "spi_read: Success");
        ESP_LOGE("BNO080初始化", "TX Data (%d bytes):", len);
        ESP_LOG_BUFFER_HEX("BNO080初始化", tx_buffer, len);
        ESP_LOGE("BNO080初始化", "RX Data (%d bytes):", len);
        ESP_LOG_BUFFER_HEX("BNO080初始化", buffer, len);
    } else {
        ESP_LOGE("BNO080初始化", "spi_read: Failed with error %d", ret);
    }

    // 释放发送缓冲区
    free(tx_buffer);

    return (ret == ESP_OK);
}


/*!
 *    @brief  Write a buffer or two to the SPI device, with transaction
 * management.
 *    @param  buffer Pointer to buffer of data to write
 *    @param  len Number of bytes from buffer to write
 *    @param  prefix_buffer Pointer to optional array of data to write before
 * buffer.
 *    @param  prefix_len Number of bytes from prefix buffer to write
 *    @return Always returns true because there's no way to test success of SPI
 * writes
 */static bool spi_write(const uint8_t *buffer, size_t len,
                      const uint8_t *prefix_buffer, size_t prefix_len) 
{
    esp_err_t ret;
    
    // 合并前缀和数据到单个缓冲区(避免多次事务)
    uint8_t *combined_buf = NULL;
    size_t total_len = prefix_len + len;
    
    // **打印写入的完整数据**
    ESP_LOGE("BNO080_w", "SPI Write: Total Length = %d", (int)total_len);
    if (prefix_len > 0) {
        ESP_LOGE("BNO080_w", "Prefix Buffer (%d bytes):", (int)prefix_len);
        ESP_LOG_BUFFER_HEX("BNO080SPI", prefix_buffer, prefix_len);
    }
    if (len > 0) {
        ESP_LOGE("BNO080_w", "Data Buffer (%d bytes):", (int)len);
        ESP_LOG_BUFFER_HEX("BNO080SPI", buffer, len);
    }

    if (total_len > 0) {
        combined_buf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
        if (!combined_buf) return false;
        
        if (prefix_len > 0) {
            memcpy(combined_buf, prefix_buffer, prefix_len);
        }
        if (len > 0) {
            memcpy(combined_buf + prefix_len, buffer, len);
        }
        
        ESP_LOGE("BNO080_w", "Combined Buffer (%d bytes):", (int)total_len);
        ESP_LOG_BUFFER_HEX("BNO080SPI", combined_buf, total_len);
    }

    // 配置 SPI 事务
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = total_len * 8;      // 总比特数
    t.tx_buffer = combined_buf;
    t.rx_buffer = NULL;            // 写操作不需要接收缓冲区
    t.flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX; // 半双工模式,无dummy位

    // 等待INTN变为低电平,最多等待10ms
    uint32_t start_time = millis();
    while(gpio_get_level(BNO_INTN)) {
        if(millis() - start_time > 10) {
            ESP_LOGE("BNO080_w", "spi_write: INTN timeout");
            return false;
        }
        vTaskDelay(1);
    }

    // CS拉低前等待至少5μs确保INTN稳定
    esp_rom_delay_us(5);
    gpio_set_level(BNO_CS, 0);
    esp_rom_delay_us(5);  // CS拉低后等待至少5μs

    // 执行 SPI 传输
    ret = spi_device_transmit(&PAW_bno080, &t);

    // 等待传输完成
    esp_rom_delay_us(5);

    // CS拉高前等待至少5μs
    esp_rom_delay_us(5);
    gpio_set_level(BNO_CS, 1);
    esp_rom_delay_us(5);  // CS拉高后等待至少5μs

    // 等待INTN恢复高电平
    start_time = millis();
    while(!gpio_get_level(BNO_INTN)) {
        if(millis() - start_time > 10) {
            ESP_LOGE("BNO080_w", "spi_write: INTN not restored");
            break;
        }
        vTaskDelay(1);
    }

    // 释放组合缓冲区
    if (combined_buf) {
        free(combined_buf);
    }

    // **检查 SPI 传输是否成功**
    if (ret != ESP_OK) {
        ESP_LOGE("BNO080_w", "SPI transmit failed! Error: %d", ret);
    } else {
        ESP_LOGE("BNO080_w", "SPI transmit successful!");
    }

    return (ret == ESP_OK);
}


//复位芯片
static void BNO_hardwareReset(void) {
    ESP_LOGE("BNO080初始化", "BNO_hardwareReset: starting reset sequence");
    
    // 确保所有控制引脚处于正确状态
    gpio_set_level(BNO_CS, 1);     // CS 高电平
    gpio_set_level(BNO_PS0, 1);    // PS0 高电平 - SPI模式
    gpio_set_level(BNO_PS1, 1);    // PS1 高电平 - SPI模式
    gpio_set_level(BNO_BOOT, 1);   // BOOT 高电平 - 正常启动
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待引脚状态稳定
    
    // 复位序列
    gpio_set_level(BNO_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(20)); // 增加到20ms
    ESP_LOGE("BNO080初始化", "BNO_hardwareReset: RESET -> HIGH");
    
    gpio_set_level(BNO_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20)); // 增加到20ms
    ESP_LOGE("BNO080初始化", "BNO_hardwareReset: RESET -> LOW");
    
    gpio_set_level(BNO_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // 增加到50ms等待芯片完全启动
    ESP_LOGE("BNO080初始化", "BNO_hardwareReset: RESET -> HIGH, reset sequence complete");

    // 记录所有引脚状态
    ESP_LOGE("BNO080初始化", "Pin states after reset:");
    ESP_LOGE("BNO080初始化", "PS1=%d (expected=1)", gpio_get_level(BNO_PS1));
    ESP_LOGE("BNO080初始化", "PS0=%d (expected=1)", gpio_get_level(BNO_PS0));
    ESP_LOGE("BNO080初始化", "BOOT=%d (expected=1)", gpio_get_level(BNO_BOOT));
    ESP_LOGE("BNO080初始化", "CS=%d (expected=1)", gpio_get_level(BNO_CS));
    ESP_LOGE("BNO080初始化", "RESET=%d (expected=1)", gpio_get_level(BNO_RESET));
    ESP_LOGE("BNO080初始化", "INTN=%d", gpio_get_level(BNO_INTN));

    // 等待INTN变为高电平,表示芯片准备好
    uint32_t start_time = millis();
    uint32_t timeout = 100; // 100ms超时
    while(gpio_get_level(BNO_INTN) == 0) {
        if(millis() - start_time > timeout) {
            ESP_LOGE("BNO080初始化", "Warning: INTN did not go high after reset");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = millis() * 1000;
  // Serial.printf("I2C HAL get time: %d\n", t);
  return t;
}

static bool hal_wait_for_int(void) {
  ESP_LOGE("BNO080初始化", "hal_wait_for_int: waiting for interrupt...");
  
  // 首先检查当前INTN状态
  if (!gpio_get_level(BNO_INTN)) {
    ESP_LOGE("BNO080初始化", "hal_wait_for_int: interrupt already active");
    return true;
  }

  // 等待INTN变为低电平
  uint32_t start_time = millis();
  uint32_t timeout = 100; // 100ms超时
  
  while (gpio_get_level(BNO_INTN)) {
    if (millis() - start_time > timeout) {
      ESP_LOGE("BNO080初始化", "hal_wait_for_int: timeout waiting for interrupt");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  ESP_LOGE("BNO080初始化", "hal_wait_for_int: interrupt detected after %lu ms", 
           millis() - start_time);
  return true;
}


static int spihal_open(sh2_Hal_t *self) {
  ESP_LOGE("BNO080初始化", "spihal_open: starting HAL initialization");

  if (!self) {
    ESP_LOGE("BNO080初始化", "spihal_open: NULL HAL pointer");
    return -1;
  }

  ESP_LOGE("BNO080初始化", "spihal_open: waiting for initial interrupt");
  if (!hal_wait_for_int()) {
    ESP_LOGE("BNO080初始化", "spihal_open: initial interrupt timeout");
    return -1;
  }

  ESP_LOGE("BNO080初始化", "spihal_open: HAL initialization complete");
  return 0;
}


static void spihal_close(sh2_Hal_t *self) {
  // Serial.println("SPI HAL close");
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  uint16_t packet_size = 0;

  ESP_LOGE("BNO080初始化", "spihal_read: start, len=%d", len);

  if (!hal_wait_for_int()) {
    ESP_LOGE("BNO080初始化", "spihal_read: wait_for_int timeout");
    return 0;
  }

  if (!spi_read(pBuffer, 4, 0x00)) {
    ESP_LOGE("BNO080初始化", "spihal_read: header read failed");
    return 0;
  }

  // Determine amount to read
  packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
  ESP_LOGE("BNO080初始化", "spihal_read: header=[%02x %02x %02x %02x], packet_size=%d", 
           pBuffer[0], pBuffer[1], pBuffer[2], pBuffer[3], packet_size);

  // Unset the "continue" bit
  packet_size &= ~0x8000;

  if (packet_size > len) {
    ESP_LOGE("BNO080初始化", "spihal_read: packet too large (%d > %d)", packet_size, len);
    return 0;
  }

  if (!hal_wait_for_int()) {
    ESP_LOGE("BNO080初始化", "spihal_read: wait_for_int timeout (2)");
    return 0;
  }

  if (!spi_read(pBuffer, packet_size, 0x00)) {
    ESP_LOGE("BNO080初始化", "spihal_read: payload read failed");
    return 0;
  }

  ESP_LOGE("BNO080初始化", "spihal_read: success, read %d bytes", packet_size);
  return packet_size;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  ESP_LOGE("BNO080初始化", "spihal_write: start, len=%d", len);
  
  if (!hal_wait_for_int()) {
    ESP_LOGE("BNO080初始化", "spihal_write: wait_for_int timeout");
    return 0;
  }

  ESP_LOGE("BNO080初始化", "spihal_write: data to write:");
  ESP_LOG_BUFFER_HEX("BNO080初始化", pBuffer, len);

  if (!spi_write(pBuffer, len, NULL, 0)) {
    ESP_LOGE("BNO080初始化", "spihal_write: write failed");
    return 0;
  }

  ESP_LOGE("BNO080初始化", "spihal_write: success");
  return len;
}


static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    //Serial.println("BNO08x - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}


// 初始化
bool bno080_init(spi_host_device_t HOST) {
    int status;
    _reset_occurred = false;
    _sensor_value = NULL;

    ESP_LOGE("BNO080初始化", "Starting initialization sequence");

    // 1. 配置所有引脚为默认状态
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BNO_PS1) | (1ULL << BNO_PS0) | (1ULL << BNO_BOOT) | (1ULL << BNO_RESET) | (1ULL << BNO_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 2. 配置中断引脚
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << BNO_INTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&int_conf);

    // 3. 初始化引脚状态
    gpio_set_level(BNO_CS, 1);     // CS 高电平
    gpio_set_level(BNO_RESET, 1);  // RESET 高电平
    gpio_set_level(BNO_PS0, 0);    // PS0 初始为低
    gpio_set_level(BNO_PS1, 0);    // PS1 初始为低
    gpio_set_level(BNO_BOOT, 1);   // BOOT 高电平
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待状态稳定

    // 4. 设置为SPI模式 (PS1=1, PS0=1)
    gpio_set_level(BNO_PS1, 1);
    gpio_set_level(BNO_PS0, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // 等待模式切换完成

    // 配置SPI总线
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 100000,    // 降低到100KHz进行测试
        .duty_cycle_pos = 128,        // 50% duty cycle
        .mode = 3,                    // CPOL=1, CPHA=1
        .spics_io_num = -1,          // CS pin handled manually
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST,  // LSB优先
        .pre_cb = NULL,
        .post_cb = NULL,
        .cs_ena_pretrans = 20,        // 增加CS前等待时间
        .cs_ena_posttrans = 20,       // 增加CS后等待时间
        .input_delay_ns = 500         // 增加输入延迟以确保数据稳定
    };
    
    esp_err_t ret = spi_bus_add_device(HOST, &devcfg, &PAW_bno080);
    if (ret != ESP_OK) {
        ESP_LOGE("BNO080初始化", "Failed to add SPI device: %d", ret);
        return false;
    }
    ESP_LOGE("BNO080初始化", "SPI device added successfully");

    _HAL.open = spihal_open;
	_HAL.close = spihal_close;
	_HAL.read = spihal_read;
	_HAL.write = spihal_write;
	_HAL.getTimeUs = hal_getTimeUs;
    int status_HAL = _HAL.open(&_HAL);
    ESP_LOGE("BNO080初始化", "sh2_open: HAL open status=%d", status_HAL);
    
    BNO_hardwareReset();//重设下

    // Open SH2 interface (also registers non-sensor event handler.)
    status = sh2_open(&_HAL, hal_callback, NULL);
    if (status != SH2_OK) {
        return false;
    }
    ESP_LOGE("BNO080初始化", "sh2_open: status=%d", status);

    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        ESP_LOGE("BNO080初始化", "通讯验证错误%d" ,status);
        ESP_LOGE("BNO080初始化", "sh2_getProdIds: numEntries=%d", prodIds.numEntries);
        return false;
    }
    else{
        ESP_LOGI("BNO080初始化", "验证成功，储存的报告数量为:%u",prodIds.numEntries);
        for (uint8_t i = 0; i < prodIds.numEntries; i++) {
            sh2_ProductId_t *p = &prodIds.entry[i];
            ESP_LOGI("BNO080初始化", "|实例%d", i);
            ESP_LOGI("BNO080初始化", "|最近一次复位的原因: %u", p->resetCause);
            ESP_LOGI("BNO080初始化", "|固件主版本号: %u", p->swVersionMajor);
            ESP_LOGI("BNO080初始化", "|固件次版本号: %u", p->swVersionMinor);
            ESP_LOGI("BNO080初始化", "|固件补丁版本号: %u", p->swVersionPatch);
            ESP_LOGI("BNO080初始化", "|软件部件号: %"PRIu32, p->swPartNumber);
            ESP_LOGI("BNO080初始化", "|软件构建号: %"PRIu32, p->swBuildNumber);
            ESP_LOGI("BNO080初始化", "|保留位: %u", p->reserved0);
            ESP_LOGI("BNO080初始化", "|保留位: %u\n", p->reserved1);
        }
    }
    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    return true;
}
