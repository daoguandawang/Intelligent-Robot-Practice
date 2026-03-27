# 步进电机云台自动标定 (Calibration) 执行流解析

## 1. 模块概述
本模块负责云台激光打靶系统的“二维自动网格标定”。通过控制偏航（Yaw）和俯仰（Pitch）步进电机执行蛇形扫描，并与上位机视觉系统异步交互，最终生成一份 `物理步数 <-> 摄像头像素(X,Y)` 的坐标映射表。

本系统属于典型的无操作系统（裸机）程序。程序的运行轨迹遵循标准模式：**1次初始化（`setup`） + 无数次主循环轮询（`loop`） + 硬件定时器后台中断（ISR）**。

## 2. 核心状态机与交互协议
* **系统主状态 (`g_mode`)**：仅在 `MODE_CALIB` 时执行此标定流程。
* **标定子状态**：
    * `g_calTask.active`：标定总开关。
    * `g_calTask.waitingPoint`：阻塞标志，为 `true` 时暂停电机移动，等待视觉反馈。
* **核心交互指令**：
    * `CAL_START ...` (上位机 -> 单片机)：启动扫描。
    * `REQ_POINT ...` (单片机 -> 上位机)：请求当前物理位置的视觉坐标。
    * `POINT <x> <y>` (上位机 -> 单片机)：下发视觉坐标。

---

## 3. 执行流全景推演

以下是进入自动标定模式后的完整函数调用链路与状态流转步骤。

### 阶段一：开机准备与后台唤醒
系统上电，执行一次 `setup()`，完成硬件初始化并启动隐形的定时器后台。
* **关键动作**：`setupTimer1_500us()` 开启硬件定时器。从这一刻起，硬件每隔 0.5 毫秒强制触发一次中断，在后台持续调用电机运转函数。它是电机平滑转动的“心脏泵”，独立于主程序运行。

### 阶段二：进入主循环 (The Main Engine)
`setup()` 结束后，程序进入 `loop()`。主循环类似于一个高速运转的传送带，不断轮询三大核心任务：
1. `pollSerialLine()`：查串口指令。
2. `updateCalibrationTask()`：推进标定流程。
3. `applyPidUpdate()`：处理 PID 闭环追踪。

### 阶段三：标定流程详细步骤 (收到 CAL_START 后)

#### 📍 步骤 1：接收并解析指令
* `loop()` 轮询至 **`pollSerialLine()`**。
* 识别到标定命令，提取角度范围，调用 **`startCalibration()`**。

#### 📍 步骤 2：初始化标定任务 (状态机转场)
* **`startCalibration()`** 执行初始化：切换总控状态至 `MODE_CALIB`，激活标定任务（`active = true`）。
* 计算网格起点、终点和步长，并命令电机走到**第一个点**。
* 电机走稳后，调用 **`requestPointForCurrentPose()`** 向电脑索要坐标，并置位等待标志（`waitingPoint = true`）。

#### 📍 步骤 3：异步等待与网格扫描
* `loop()` 轮询至 **`updateCalibrationTask()`**。
* 因处于等待状态，主循环在此处快速空转，持续检查是否收到新视觉坐标或是否超时。

#### 📍 步骤 4：收到视觉坐标，走向下一个点
* 某次轮询至 **`pollSerialLine()`**，收到电脑回传：`POINT x y`。解析坐标并更新序号。
* 紧接着轮询至 **`updateCalibrationTask()`**，发现数据更新。
* 调用 **`appendCalibrationSample()`** 记录数据至 `g_calTable`。
* 解除等待状态（`waitingPoint = false`），调用 **`stepCalibrationGrid()`** 走向下一个点。

#### 📍 步骤 5：蛇形走位计算 (`stepCalibrationGrid`)
* **当前行未走完**：Yaw 加一个步长，移动电机，返回 `true`。
* **当前行已走完**：Pitch 换行，Yaw 扫描方向反转（实现蛇形扫描），移动电机，返回 `true`。
* **全部网格扫完**：Pitch 超出终点，返回 `false`。

#### 📍 步骤 6：循环往复，直至结束
* 只要未扫完（返回 `true`），等待电机稳定后，再次索要坐标，回到 **步骤 3**。
* 扫描全部完成（返回 `false`），调用 **`finishCalibration()`**，关闭任务，状态切回 `MODE_IDLE`，标定圆满结束。

---

## 4. 核心时序图 (Sequence Diagram)

```mermaid
sequenceDiagram
    participant PC as 上位机 (电脑)
    participant Loop as 单片机 主循环
    participant Motor as 步进电机底层

    PC->>Loop: 发送 CAL_START 启动指令
    Loop->>Loop: 初始化标定任务 (g_mode = MODE_CALIB)
    Loop->>Motor: moveMotorTo(移动到起点)
    Loop->>PC: 发送 REQ_POINT (索要起点像素)
    Note right of Loop: waitingPoint = true<br/>进入异步等待状态

    loop 网格扫描循环
        PC-->>Loop: 视觉识别完成，发送 POINT x y
        Loop->>Loop: 记录数据到 g_calTable
        Note right of Loop: waitingPoint = false<br/>解除等待
        
        alt 扫描未结束
            Loop->>Loop: 计算下一步，rowForward 蛇形转向
            Loop->>Motor: moveMotorTo(下一个网格点)
            Loop->>PC: 发送 REQ_POINT
            Note right of Loop: 再次进入等待
        else 扫描全部完成
            Loop->>Loop: finishCalibration()
            Loop->>PC: 发送 CAL_DONE
        end
    end