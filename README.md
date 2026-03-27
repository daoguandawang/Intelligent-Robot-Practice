# Intelligent-Robot-Practice | 智能机器人实践项目

本项目是一个结合了 **计算机视觉 (OpenCV)** 与 **嵌入式控制 (Arduino/PlatformIO)** 的智能机器人控制系统。主要功能是通过上位机识别目标（二维码、激光点、特定色块），并通过串口通讯驱动下位机云台实现精确的激光追踪与自动标定。

## 🏗 项目架构

项目由两部分协同工作：
1.  **上位机 (Vision System)**: 运行于 PC 端（Python），负责图像采集、目标识别、坐标转换及任务逻辑调度。
2.  **下位机 (Control System)**: 基于 Arduino 开发板（C++），负责底层步进电机驱动及硬件动作执行。

## 📂 目录结构说明

    ├── final.py                # 上位机核心执行脚本（包含状态机和任务流管理）
    ├── open_test.py             # 上位机调试工具（包含激光检测、PID计算、标定表生成）
    ├── platformio.ini           # PlatformIO 项目配置文件
    ├── src/
    │   └── main.cpp             # 下位机主程序（指令解析、电机定时器驱动、扫射逻辑）
    ├── lib/                     # 下位机私有库
    │   ├── Ctrl/                # 步进电机基类、PID 算法及坐标转换实现
    │   └── Interface/           # 针对 Arduino 硬件（如 Mega 2560）的接口实现
    └── include/                 # 项目相关头文件

## 🛠 开发环境

### 上位机 (Upper Computer)
- **语言**: Python 3.10+
- **关键库**: opencv-python, numpy, pyserial
- **核心功能**: 
  - **视觉定位**: 实时识别红色激光中心点 (RedLaserDetector)。
  - **任务调度**: 通过二维码解析触发不同的打击/追踪任务。
  - **自动标定**: 基于蛇形扫描算法建立像素坐标与电机步进数的映射关系。

### 下位机 (Lower Computer)
- **IDE**: CLion + PlatformIO 插件
- **框架**: Arduino
- **硬件**: Arduino Mega 2560 (或兼容开发板)
- **核心功能**:
  - **电机驱动**: 采用硬件定时器 (500us 周期) 驱动步进电机，保证运动平滑。
  - **通讯协议**: 自定义串口协议，支持 SPEED（速度模式）、DPOS（增量模式）、CAL_START（标定开始）等指令。

## 🚀 核心逻辑：自动标定 (Auto Calibration)

系统最核心的特性是**自动化坐标映射**：
1. 下位机控制云台进行预设的网格扫描。
2. 每一个点位，上位机通过摄像头记录激光点的像素位置。
3. 最终生成 calibration_table.csv。
4. **效果**: 实现“指哪打哪”，上位机只需在画面点击一个像素点，云台即可精准指向物理位置。

## 📦 快速开始

### 1. 下位机部署
1. 使用 CLion 打开项目。
2. 确保 platformio.ini 中的 board 与你实际硬件一致。
3. 点击 PlatformIO: Upload 烧录固件。

### 2. 上位机运行
1. 安装依赖：
    pip install opencv-python numpy pyserial

2. 在 final.py 或 open_test.py 中修改对应的串口号（如 COM8 或 /dev/ttyUSB0）：
    ser = serial.Serial('COM8', 115200, timeout=0.1)

3. 运行程序：
    python final.py

## 🤝 贡献
本项目用于电子科技大学（UESTC）机电学院智能机器人实践课程。
