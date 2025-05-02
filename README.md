# FUKY 硬件驱动项目

## 项目概述
本项目包含两个嵌入式硬件驱动模块，基于ESP32平台开发：

### 1. FUKY_HARDWARE-LOCATOR
- 摄像头定位模块
- 支持USB数据传输
- PC端控制协议：
  - 发送`1`启动实时画面传输
  - 发送`0`停止传输

### 2. FUKY_HARDWARE-MOUSE
- 智能无线鼠标方案
- 核心功能：
  - 双模运动检测：PAW3395光学传感器 + BNO080惯性测量单元(IMU)
  - 蓝牙HID协议支持
  - 运动数据融合处理

## 硬件架构
### 主要组件
| 部件 | 型号 | 接口 | 功能 |
|------|------|------|------|
| 主控 | ESP32-S3 | SPI/I2C | 系统控制核心 |
| 光学传感器 | PAW3395 | SPI | 平面位移检测 |
| 运动处理器 | BNO080 | SPI | 3D姿态感知 |

### 引脚配置
```c
#define MOSI    6   // SPI主出从入
#define MISO    3   // SPI主入从出 
#define SCLK    4   // 时钟信号
#define FUKY_SPI_HOST SPI2_HOST  // SPI总线选择
```

## 开发环境
- ESP-IDF 5.1+
- Arduino Core for ESP32
- 依赖组件：
  - [TinyUSB](https://github.com/espressif/esp-tinyusb)
  - [BNO080驱动](components/BNO080)
  - [PAW3395驱动](components/PAW3395)

## 编译烧录
```bash
# 设置目标芯片
idf.py set-target esp32s3

# 配置项目
idf.py menuconfig

# 编译并烧录
idf.py build flash monitor
```

## 项目结构
```
FUKY_VeryHardWhere/
├── FUKY_HARDWARE-LOCATOR/    # 摄像头定位模块
│   ├── camera_index.h        # 摄像头配置
│   └── FUKY_HARDWARE-LOCATOR.ino  # 主程序
├── FUKY_HARDWARE-MOUSE/      # 智能鼠标模块
│   ├── components/           # 驱动组件
│   │   ├── BLE/              # 蓝牙HID实现
│   │   ├── BNO080/           # IMU驱动
│   │   └── PAW3395/          # 光学传感器驱动
│   └── main/src/FUKY_main.c  # 主控逻辑
└── README.md                 # 本文档
```

## 使用指南
1. 连接硬件设备至PC
2. 上电后等待蓝牙广播（设备名称：FUKY_MOUSE）
3. 在PC蓝牙设置中配对设备
4. 移动设备测试光标响应
5. 倾斜设备测试姿态控制

## 许可证
[MIT License](LICENSE) © 2024 FUKY Team
