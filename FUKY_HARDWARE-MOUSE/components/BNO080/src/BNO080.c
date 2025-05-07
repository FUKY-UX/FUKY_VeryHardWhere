#include <inttypes.h> 
#include <string.h> 
#include "bno080.h"
#include "sh2.h"
#include "esp_log.h"
#include <stdio.h>
#include "esp_timer.h"

typedef struct BNO080_Calibration
{
  void (*onButtonPressed)(void);
  bool isEnable;
  bool isOnprocessing;
  uint8_t Processing;
}BNO080_Calibration;
//======================================功能逻辑====================================//
IMUData_t IMUData;
//======================================芯片业务====================================//
spi_device_handle_t PAW_bno080;
sh2_ProductIds_t prodIds;
bool _reset_occurred = false;
sh2_SensorValue_t *_sensor_value = NULL;
sh2_SensorValue_t sensorValue;  // 定义全局变量
sh2_Hal_t _HAL;
BNO080_Calibration Calibrate_button;
//======================================前向声明====================================//
void bno080_StartCalibrate(void);
void bno080_EndCalibration(void);
void printResetReasonName(uint8_t resetReasonNumber);
void PrintTest(void);
void SetReport_Test(void);
//======================================回调函数====================================//
static void IRAM_ATTR button_isr_handler(void* arg)
{
    // 使用静态变量记录上一次中断触发时间（单位：微秒）
    static volatile uint32_t last_isr_time = 0;
    // 获取当前时间（微秒）
    uint32_t current_time = esp_timer_get_time();
    // 如果两次中断间隔小于 500ms（500000 微秒），则认为是抖动，直接返回
    if (current_time - last_isr_time < 500000) {
        return;
    }
    // 更新上一次中断触发时间
    last_isr_time = current_time;
    // 当检测到按钮电平为 0 时，调用回调
}

void Calibration_Change(){
  if(Calibrate_button.Processing<=3)
  {
    Calibrate_button.Processing +=1;
    ESP_LOGW("BNO080动态校准", "显示下一个传感器,当前为第%d个",Calibrate_button.Processing );
  }
  return;
}

//======================================基础功能====================================//
void print_binary(uint8_t value) {
    char binary_str[9]; // 8位二进制 + 终止符
    for (int i = 7; i >= 0; i--) {
        binary_str[7 - i] = (value & (1 << i)) ? '1' : '0';
    }
    binary_str[8] = '\0'; // 终止符

    ESP_LOGW("BNO080动态校准", "配置值: %s", binary_str);
}

uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void BNO_hardwareReset(void) {
    ESP_LOGI("BNO080", "执行硬件复位");
    
    gpio_set_level(BNO_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BNO_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BNO_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

}

static bool hal_wait_for_int(void) {
  for (int i = 0; i < 500; i++) {
    if (!gpio_get_level(BNO_INTN))
      return true;
    vTaskDelay(1);
  }
  ESP_LOGE("BNO080", "等待INTN超时");
  BNO_hardwareReset();

  return false;
}

static bool spi_read(uint8_t *buffer, size_t len, uint8_t sendvalue) {
    // 准备发送缓冲区
    uint8_t *tx_buffer = heap_caps_malloc(len, MALLOC_CAP_DMA);
    if (!tx_buffer) {
        //ESP_LOGE("BNO080", "无法分配发送缓冲区");
        return false;
    }
    memset(tx_buffer, sendvalue, len);
    // 配置SPI事务
    spi_transaction_t t = {
        .length = len * 8,      // 发送和接收长度必须相等
        .tx_buffer = tx_buffer, // 发送缓冲区
        .rx_buffer = buffer,    // 接收缓冲区
        .flags = 0              // 使用全双工模式
    };
    // 执行传输
    spi_device_polling_transmit(PAW_bno080, &t);
    // 打印调试信息
    // if (ret == ESP_OK) {
    //     ESP_LOGI("BNO080", "读取成功 (%d bytes)", len);
    //     ESP_LOG_BUFFER_HEX("BNO080-TX", tx_buffer, len);
    //     ESP_LOG_BUFFER_HEX("BNO080-RX", buffer, len);
    // } else {
    //     ESP_LOGE("BNO080", "读取失败: %d", ret);
    // }

    free(tx_buffer);
    return true;
}

static bool spi_write(const uint8_t *buffer, size_t len/*const uint8_t *prefix_buffer, size_t prefix_len*/) {

    // if (len == 0) {
    //     ESP_LOGE("BNO080", "无效的写入长度");
    //     return false;
    // }

    // 分配并准备发送缓冲区
    uint8_t *cptx_buffer = heap_caps_malloc(len, MALLOC_CAP_DMA);
    if (!cptx_buffer) {
        ESP_LOGE("BNO080", "无法分配发送缓冲区");
        return false;
    }

    memcpy(cptx_buffer, buffer, len);

    // 配置SPI事务
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = cptx_buffer,
        .rx_buffer = NULL,
        .flags = 0
    };

    // 执行传输
    esp_err_t ret = spi_device_polling_transmit(PAW_bno080, &t);

    free(cptx_buffer);
    return (ret == ESP_OK);
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    return millis() * 1000;
}

static int spihal_open(sh2_Hal_t *self) {
    //ESP_LOGI("BNO080", "打开SPI HAL");
    if (!hal_wait_for_int()) {
        ESP_LOGE("BNO080", "HAL打开失败");
        return -1;
    }
    return 0;
}

static void spihal_close(sh2_Hal_t *self) {
    ESP_LOGI("BNO080", "关闭SPI HAL");
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,uint32_t *t_us) {
  // Serial.println("SPI HAL read");

  uint16_t packet_size = 0;

  if (!hal_wait_for_int()) {
    return 0;
  }

  if (!spi_read(pBuffer, 4, 0x00)) {
    return 0;
  }

  // Determine amount to read
  packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  if (packet_size > len) {
    return 0;
  }

  if (!hal_wait_for_int()) {
    return 0;
  }

  if (!spi_read(pBuffer, packet_size, 0x00)) {
    return 0;
  }

  return packet_size;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  // Serial.print("SPI HAL write packet size: ");
  // Serial.println(len);

  if (!hal_wait_for_int()) {
    return 0;
  }

  spi_write(pBuffer, len);

  return len;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        //ESP_LOGI("BNO080", "检测到复位事件");
        _reset_occurred = true;
    }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;
  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    //ESP_LOGE("解码", "非有用数据");
    _sensor_value->timestamp = 0;
    return;
  }
}

//======================================传感器数据读取====================================//
/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool getSensorEvent() {

  _sensor_value = &sensorValue;

  _sensor_value->timestamp = 0;

  sh2_service();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

//Return the sensorID
uint8_t getSensorEventID()
{
	return _sensor_value->sensorId;
}

//Given a number between 0 and 5, print the name of the reset reason
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
void printResetReasonName(uint8_t resetReasonNumber)
{
  if(resetReasonNumber == 1)        ESP_LOGW("BNO080", "电源电压/上电初始化中");
  else if(resetReasonNumber == 2)   ESP_LOGW("BNO080", "由设备内部电路自行触发的复位");
  else if(resetReasonNumber == 3)   ESP_LOGW("BNO080", "设备卡死或长时间未响应");
  else if(resetReasonNumber == 4)   ESP_LOGW("BNO080", "外部硬件触发的复位");
  else if(resetReasonNumber == 5)   ESP_LOGW("BNO080", "不明原因的复位");
}

//======================================项目会用到的芯片功能====================================//

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t getResetReason()
{
	return prodIds.entry[0].resetCause;
}

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;
  return x;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @param sensorSpecific config settings specific to sensor/reportID.
 * (e.g. enabling/disabling possible activities in personal activity classifier)
 * @return true: success false: failure
 */
bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us,uint32_t sensorSpecific) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = sensorSpecific;

  config.reportInterval_us = interval_us;

  if(BNO_INTN != -1) {
	if (!hal_wait_for_int()) {
      return 0;
  	}
  }
  
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

//Sends the packet to enable the rotation vector
bool enableGameRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_GAME_ROTATION_VECTOR, timeBetweenReports,0);
}

//Sends the packet to enable the magnetometer
bool enableMagnetometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, timeBetweenReports,0);		
}

//Sends the packet to enable the accelerometer
bool enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	return enableReport(SH2_LINEAR_ACCELERATION, timeBetweenReports,0);	
}

//Sends the packet to enable the rotation vector
bool enableRotationVector(uint16_t timeBetweenReports)
{
	return enableReport(SH2_ROTATION_VECTOR, timeBetweenReports,0);
}

//Return the rotation vector sensor event report status accuracy
uint8_t getAccuracy()
{
	return _sensor_value->status;
}

//Return the game rotation vector quaternion I
float getGameQuatI()
{
	return _sensor_value->un.gameRotationVector.i;
}

//Return the game rotation vector quaternion J
float getGameQuatJ()
{
	return _sensor_value->un.gameRotationVector.j;
}

//Return the game rotation vector quaternion K
float getGameQuatK()
{
	return _sensor_value->un.gameRotationVector.k;
}

//Return the game rotation vector quaternion Real
float getGameQuatReal()
{
	return _sensor_value->un.gameRotationVector.real;
}

//Return the acceleration component
float getLinAccelX()
{
	return _sensor_value->un.linearAcceleration.x;
}

//Return the acceleration component
float getLinAccelY()
{
	return _sensor_value->un.linearAcceleration.y;
}

//Return the acceleration component
float getLinAccelZ()
{
	return _sensor_value->un.linearAcceleration.z;
}

//Return the magnetometer component
float getMagX()
{
	return _sensor_value->un.magneticField.x;
}

//Return the magnetometer component
float getMagY()
{
	return _sensor_value->un.magneticField.y;
}

//Return the magnetometer component
float getMagZ()
{
	return _sensor_value->un.magneticField.z;
}

//Return the rotation vector quaternion I
float getQuatI()
{
	// float quat = qToFloat(rawQuatI, rotationVector_Q1);
	// if (_printDebug == true)
	// {
	// 	if ((quat < -1.0) || (quat > 1.0))
	// 	{
	// 		_debugPort->print(F("getQuatI: quat: ")); // Debug the occasional non-unitary Quat
	// 		_debugPort->print(quat, 2);
	// 		_debugPort->print(F(" rawQuatI: "));
	// 		_debugPort->print(rawQuatI);
	// 		_debugPort->print(F(" rotationVector_Q1: "));
	// 		_debugPort->println(rotationVector_Q1);
	// 	}
	// }
	// return (quat);
	return _sensor_value->un.rotationVector.i;
}

//Return the rotation vector quaternion J
float getQuatJ()
{
	// float quat = qToFloat(rawQuatJ, rotationVector_Q1);
	// if (_printDebug == true)
	// {
	// 	if ((quat < -1.0) || (quat > 1.0)) // Debug the occasional non-unitary Quat
	// 	{
	// 		_debugPort->print(F("getQuatJ: quat: "));
	// 		_debugPort->print(quat, 2);
	// 		_debugPort->print(F(" rawQuatJ: "));
	// 		_debugPort->print(rawQuatJ);
	// 		_debugPort->print(F(" rotationVector_Q1: "));
	// 		_debugPort->println(rotationVector_Q1);
	// 	}
	// }
	//return (quat);
	return _sensor_value->un.rotationVector.j;
}

//Return the rotation vector quaternion K
float getQuatK()
{
	// float quat = qToFloat(rawQuatK, rotationVector_Q1);
	// if (_printDebug == true)
	// {
	// 	if ((quat < -1.0) || (quat > 1.0)) // Debug the occasional non-unitary Quat
	// 	{
	// 		_debugPort->print(F("getQuatK: quat: "));
	// 		_debugPort->print(quat, 2);
	// 		_debugPort->print(F(" rawQuatK: "));
	// 		_debugPort->print(rawQuatK);
	// 		_debugPort->print(F(" rotationVector_Q1: "));
	// 		_debugPort->println(rotationVector_Q1);
	// 	}
	// }
	//return (quat);
	return _sensor_value->un.rotationVector.k;
}

//Return the rotation vector quaternion Real
float getQuatReal()
{
	return _sensor_value->un.rotationVector.real;
}


//======================================动态校准====================================//
//手册说静态校准的数据是只读的，是在出场的时候进行的校准，所以这里不考虑静态校准

//Given a accuracy number, print what it means
void printAccuracyLevel(uint8_t accuracyNumber)
{
  if(accuracyNumber == 0) ESP_LOGE("BNO080动态校准", "数据不可用");
  else if(accuracyNumber == 1) ESP_LOGE("BNO080动态校准", "低精度");
  else if(accuracyNumber == 2) ESP_LOGE("BNO080动态校准", "中等精度");
  else if(accuracyNumber == 3) ESP_LOGE("BNO080动态校准", "高精度");
}

//SH2_CAL_ACCEL || SH2_CAL_GYRO || SH2_CAL_MAG
bool setCalibrationConfig(uint8_t sensors)
{
  int status = sh2_setCalConfig(sensors);
  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

// Here is where you define the sensor outputs you want to receive
void setReports_Calibration(void) {

  ESP_LOGI("BNO080动态校准", "继续设置校准所需报告");
  if (enableMagnetometer(1) == true) {ESP_LOGI("BNO080动态校准", "磁力计报告启用");
  } else {
      ESP_LOGE("BNO080动态校准", "磁力计报告启用失败");
  }
  if (enableGameRotationVector(1) == true) {ESP_LOGI("BNO080动态校准", "游戏用旋转量报告启用");
  } else {
      ESP_LOGE("BNO080动态校准", "游戏用旋转量报告启用失败");
  }
  if (enableLinearAccelerometer(1) == true) {ESP_LOGI("BNO080动态校准", "加速度报告启用");
  } else {
      ESP_LOGE("BNO080动态校准", "加速度报告启用失败");
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

void setReports(void) {
  if (enableRotationVector(6500) == true) {ESP_LOGI("BNO080", "内部融合旋转量报告启用");
  } else {
      ESP_LOGE("BNO080", "内部融合旋转量报告启用失败");
  }
  //vTaskDelay(pdMS_TO_TICKS(100));
  if (enableLinearAccelerometer(2500) == true) {ESP_LOGI("BNO080", "加速度报告启用");
  } else {
      ESP_LOGE("BNO080", "加速度报告启用失败");
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  
  //Check_SensorInterval();

}
//This tells the BNO08x to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
bool saveCalibration()
{
  int status = sh2_saveDcdNow();
  if (status != SH2_OK) {
    return false;
  }
  return true;	
}

void bno080_EndCalibration()
{
  if (saveCalibration() == true) {
  ESP_LOGW("BNO080动态校准", "校准数据已保存");
  Calibrate_button.onButtonPressed=bno080_EndCalibration;
  } else {
    ESP_LOGE("BNO080动态校准", "数据保存失败");
    Calibrate_button.onButtonPressed=bno080_EndCalibration;
  }

}

void bno080_StartCalibrate()
{
  ESP_LOGW("BNO080动态校准", "动态校准启动");
  print_binary(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
  if(setCalibrationConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG) == true){
    ESP_LOGW("BNO080动态校准", "芯片进入动态校准记录模式");
    }
  else {
        ESP_LOGE("BNO080动态校准", "芯片无法进入动态校准记录模式");
    }

  setReports_Calibration();
  ESP_LOGW("BNO080动态校准", "开始校准,顺序为|| 加速度计 || 陀螺仪 || 磁力计 ||");
  ESP_LOGW("BNO080动态校准", "注意精准度，当精准度为标准或高时再次按下按钮结束校准");
  // 这里设置一些状态标志，等待按钮改变 mybutton.Processing 的值,定义静态变量用于2秒打印一次调试信息（单位：微秒）
  Calibrate_button.Processing = 0;
  Calibrate_button.onButtonPressed=Calibration_Change;
  Calibrate_button.isOnprocessing = true;
  vTaskDelay(pdMS_TO_TICKS(100));
}

void OnCalibrate()
{
  static uint32_t last_print_time = 0;
  if(Calibrate_button.isOnprocessing)
  {
    if(wasReset())
    {
        setReports_Calibration();
        printResetReasonName(getResetReason());  
    }
    uint32_t current_time;
    if(getSensorEvent() == true)
    {
      //ESP_LOGI("BNO080动态校准", "当前报告为%d", getSensorEventID());
      if(Calibrate_button.Processing == 0)
      {
        if(getSensorEventID() == SH2_LINEAR_ACCELERATION)
        {
          float x = getLinAccelX();
          float y = getLinAccelY();
          float z = getLinAccelZ();
          uint8_t linAccuracy = getAccuracy();
          ESP_LOGI("BNO080动态校准", "线性加速度: x=%.2f, y=%.2f, z=%.2f, 精度=%d", x, y, z, linAccuracy);
        }
        else
        {
          current_time = esp_timer_get_time();
          if (current_time - last_print_time >= 1000000) 
          {
          ESP_LOGW("BNO080动态校准", "无法获取加速度: 当前报告为%d", getSensorEventID());
                    last_print_time = current_time;
          }
        }
      }
      else if(Calibrate_button.Processing == 1)
      {
        if(getSensorEventID() == SH2_GAME_ROTATION_VECTOR)
        {
          float quatI = getGameQuatI();
          float quatJ = getGameQuatJ();
          float quatK = getGameQuatK();
          float quatReal = getGameQuatReal();
          uint8_t quatAccuracy =getAccuracy();
          current_time = esp_timer_get_time();
          if (current_time - last_print_time >= 1000000) 
          {
            ESP_LOGI("BNO080动态校准", "游戏旋转: I=%.2f, J=%.2f, K=%.2f, R=%.2f, 精度=%d", quatI, quatJ, quatK,quatReal, quatAccuracy);
            last_print_time = current_time;
          }
        }
        else{
            current_time = esp_timer_get_time();
            if (current_time - last_print_time >= 1000000) 
            {
            ESP_LOGW("BNO080动态校准", "无法获取游戏旋转向量: 当前报告为%d", getSensorEventID());
                      last_print_time = current_time;
            }
        }
      }
      else if(Calibrate_button.Processing == 2)
      {
        if(getSensorEventID() == SH2_MAGNETIC_FIELD_CALIBRATED)
        {
          float mx = getMagX();
          float my = getMagY();
          float mz = getMagZ();
          uint8_t MagAccuracy =getAccuracy();
          ESP_LOGI("BNO080动态校准", "磁力计: X=%.2f, Y=%.2f, Z=%.2f, 精度=%d", mx, my, mz, MagAccuracy);
        }
      }
      else if(Calibrate_button.Processing == 3)
        {
          if(getSensorEventID() == SH2_MAGNETIC_FIELD_CALIBRATED)
          {
              ESP_LOGI("BNO080动态校准", "校准结束，再次按下按钮退出校准程序");
              Calibrate_button.onButtonPressed=bno080_EndCalibration;
              Calibrate_button.Processing += 1;
              Calibrate_button.isOnprocessing = false;
          }
        }
    }
    
  }
}
//======================================对外暴露函数====================================//

bool bno080_init(spi_host_device_t HOST) {
    
    ESP_LOGI("BNO080", "开始初始化");
    Calibrate_button.isEnable = false;
    Calibrate_button.Processing = 0;
    Calibrate_button.isOnprocessing = false;
    _reset_occurred = false;
    _sensor_value = malloc(sizeof(sh2_SensorValue_t)); 

    memset(&IMUData, 0, sizeof(IMUData_t));


    // 配置GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BNO_RESET) | (1ULL << BNO_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    gpio_config_t ctrl_conf = 
    {
      .pin_bit_mask = (1ULL << IMU_PS0) | (1ULL << IMU_PS1) ,
      .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    gpio_config(&ctrl_conf);

    gpio_set_level(IMU_PS1,1);
    gpio_set_level(IMU_PS0,1);
    gpio_get_level(IMU_PS1);
    gpio_get_level(IMU_PS0);

    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << BNO_INTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&int_conf);
    //不考虑校准了
    // gpio_config_t btn_conf = {
    // .pin_bit_mask = (1ULL << BNO_CAL),
    // .mode = GPIO_MODE_INPUT,
    // .pull_up_en = GPIO_PULLUP_ENABLE,
    // .pull_down_en = GPIO_PULLDOWN_DISABLE,
    // .intr_type = GPIO_INTR_NEGEDGE  // 按下时下降沿
    // };
    // gpio_config(&btn_conf);
    // gpio_isr_handler_add(BNO_CAL, button_isr_handler, NULL);

    // 初始化引脚状态
    gpio_set_level(BNO_CS, 1);


    // 配置SPI
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 3 * 1000 * 1000,    // 100kHz
        .mode = 3,       // CPOL=1, CPHA=1
        .input_delay_ns = 35,   
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 3,
        .spics_io_num = BNO_CS,          // CS handled manually
        .queue_size = 1,
        .flags = 0,                  // 使用全双工模式
    };
    
    esp_err_t ret = spi_bus_add_device(HOST, &devcfg, &PAW_bno080);
    if (ret != ESP_OK) {
        ESP_LOGE("BNO080", "SPI设备添加失败: %d", ret);
        return false;
    }

    // 配置HAL接口
    _HAL.open = spihal_open;
    _HAL.close = spihal_close;
    _HAL.read = spihal_read;
    _HAL.write = spihal_write;
    _HAL.getTimeUs = hal_getTimeUs;
    
    // 执行硬件复位
    BNO_hardwareReset();

    // 打开SH2接口
    int status = sh2_open(&_HAL, hal_callback, NULL);
    if (status != SH2_OK) {
        ESP_LOGE("BNO080", "SH2接口打开失败: %d", status);
        return false;
    }

    // 获取产品ID
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
    ESP_LOGE("BNO080初始化", "通讯验证错误%d" ,status);
    ESP_LOGE("BNO080初始化", "sh2_getProdIds: numEntries=%d", prodIds.numEntries);
        return false;
    }

    // 打印产品信息
    ESP_LOGI("BNO080初始化", "验证成功，储存的报告数量为:%u",prodIds.numEntries);
    //sh2_ProductId_t *p = &prodIds.entry[0];
    // ESP_LOGI("BNO080初始化", "|第1份报告为");
    // ESP_LOGI("BNO080初始化", "|最近一次复位的原因: %u", p->resetCause);
    // ESP_LOGI("BNO080初始化", "|固件主版本号: %u", p->swVersionMajor);
    // ESP_LOGI("BNO080初始化", "|固件次版本号: %u", p->swVersionMinor);
    // ESP_LOGI("BNO080初始化", "|固件补丁版本号: %u", p->swVersionPatch);
    // ESP_LOGI("BNO080初始化", "|软件部件号: %"PRIu32, p->swPartNumber);
    // ESP_LOGI("BNO080初始化", "|软件构建号: %"PRIu32, p->swBuildNumber);
    // ESP_LOGI("BNO080初始化", "|保留位: %u", p->reserved0);
    // ESP_LOGI("BNO080初始化", "|保留位: %u\n", p->reserved1);
    
    // 注册传感器回调
    sh2_setSensorCallback(sensorHandler, NULL);
    Calibrate_button.onButtonPressed=bno080_StartCalibrate;
    //SetReport_Test();
    ESP_LOGI("BNO080", "初始化完成");
    ESP_LOGI("BNO080", "正在启用传感器报告");
    setReports();
    return true;
}

IMUData_t IRAM_ATTR bno080_Function(void)
{
  //OnCalibrate();//调试
  //进入调试模式
  // if(Calibrate_button.isEnable){
  //   ESP_LOGI("按钮", "调用: %p", Calibrate_button.onButtonPressed);
  //   Calibrate_button.isEnable =false;
  //   Calibrate_button.onButtonPressed();
  // }
  //PrintTest();
  if(wasReset())
  {
      setReports();        
      printResetReasonName(getResetReason());  
  }
  //static uint32_t last_print_time_test = 0;
  if(getSensorEvent() == true)
  {
    if(getSensorEventID() == SH2_LINEAR_ACCELERATION)
    {
      int16_t x = getLinAccelX();
      int16_t y = getLinAccelY();
      int16_t z = getLinAccelZ();
      //printf("加速度: x=%.2f, y=%.2f, z=%.2f, A=%d\n", x, y, z, linAccuracy);
      IMUData.lin_accel_x = x;
      IMUData.lin_accel_y = y;
      IMUData.lin_accel_z = z;
    }
    if(getSensorEventID() == SH2_ROTATION_VECTOR)
    {
      int16_t I = getQuatI();
      int16_t J = getQuatJ();
      int16_t K = getQuatK();
      int16_t W = getQuatReal();
      IMUData.quat_i = I;
      IMUData.quat_j = J;
      IMUData.quat_k = K;
      IMUData.quat_w = W;
      //printf("四元数: i=%.2f, j=%.2f, k=%.2f, w=%.2f\n", I, J, K, W);
    }
  }
  
  return IMUData;
}

void Check_SensorInterval()
{
  sh2_SensorConfig_t GameRotate_SensorReport;
  sh2_SensorConfig_t LinACC_SensorReport;
  
  sh2_getSensorConfig(SH2_ROTATION_VECTOR,&GameRotate_SensorReport);
  sh2_getSensorConfig(SH2_LINEAR_ACCELERATION,&LinACC_SensorReport);
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGW("BNO080","旋转矢量的报告速率:%" PRIu32 "us    线性加速度的报告速率:%" PRIu32 "us",
    GameRotate_SensorReport.reportInterval_us,
    LinACC_SensorReport.reportInterval_us);
}

//=====================================参数读取测试=====================================//

// void SetReport_Test(void)
// {
//     enableLinearAccelerometer(10); 
//     vTaskDelay(pdMS_TO_TICKS(100));
// }

// void PrintTest(void)
// { 
//     if(wasReset())
//     {
//         SetReport_Test();        
//         printResetReasonName(getResetReason());  
//     }
//     static uint32_t last_print_time_test = 0;
//     if(getSensorEvent() == true)
//     {
//       if(getSensorEventID() == SH2_LINEAR_ACCELERATION)
//       {
//         float x = getLinAccelX();
//         float y = getLinAccelY();
//         float z = getLinAccelZ();
//         uint8_t linAccuracy = getAccuracy();
//         ESP_LOGI("BNO080动态校准", "线性加速度: x=%.2f, y=%.2f, z=%.2f, 精度=%d", x, y, z, linAccuracy);
//       }
//     }
// }