#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "esp_camera.h"
#include "camera_pins.h"

bool IsStreaming = false; 
camera_config_t config;

void setup() {
  Serial.begin(926100);
  while(!Serial)

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_SVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  //camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) 
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  // 图像处理参数
  s->set_brightness(s, 0);                    // Brightness 0
  s->set_contrast(s, 0);                      // Contrast 0
  s->set_saturation(s, 0);                    // Saturation 0
  s->set_special_effect(s, 0);                // Special Effect 0
  // 自动控制
  s->set_whitebal(s, 0);                      // AWB off
  s->set_awb_gain(s, 0);                      // AWB Gain off
  s->set_exposure_ctrl(s, 0);                 // AEC Sensor off
  s->set_aec2(s, 0);                          // AEC DSP off
  s->set_ae_level(s, 0);                      // AE Level 0
  s->set_aec_value(s, 1200);                  // Exposure 1200
  s->set_gain_ctrl(s, 0);                     // AGC off
  s->set_agc_gain(s, 1);                      // Gain 1x (需根据实际传感器调整值)
  // 图像校正
  s->set_bpc(s, 0);                           // BPC off
  s->set_wpc(s, 0);                           // WPC off
  s->set_raw_gma(s, 0);                       // Raw GMA off
  s->set_lenc(s, 0);                          // Lens Correction off
  // 镜像/翻转
  s->set_hmirror(s, 1);                       // H-Mirror off
  s->set_vflip(s, 0);                         // V-Flip off
  // 其他功能
  s->set_dcw(s, 0);                           // DCW off
  s->set_colorbar(s, 0);                      // Color Bar off
  //禁用所有自动优化算法
  s->set_denoise(s, 0);       // 降噪关闭
  s->set_sharpness(s, 0);     // 锐化关闭
  s->set_wb_mode(s, 0);       // 白平衡模式0
}

void loop() 
{
  // 处理串口指令
  while(Serial.available() > 0) 
  {
    uint8_t cmd = Serial.read();
    if(cmd == 1)
    {
      IsStreaming = true;
    }
    else if(cmd == 0)
    {
      IsStreaming = false;
    }
    else 
    {
      Serial.write(cmd);
      Serial.flush();
    }
  }


  if(IsStreaming)
  {
    sendingCamData();
  }

  delay(30);
}

void sendingCamData()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return;

    // 发送帧头
    Serial.write(0xAA);
    Serial.write(0x55);
    // 发送图像长度（4字节）
    uint32_t len = fb->len;
    Serial.write((uint8_t*)&len, 4);
    // 发送JPEG数据
    Serial.write(fb->buf, fb->len);
    // 发送帧尾
    Serial.write(0x55);
    Serial.write(0xAA);
    esp_camera_fb_return(fb);
}