# BMI270

Bosch BMI270 六轴 IMU 传感器驱动模块（BMI270 6-axis IMU Driver for LibXR）

该模块通过 SPI 驱动 Bosch BMI270 IMU，输出机体坐标系下的三轴加速度与三轴角速度，并内置温度控制（加热片 + PID）和陀螺仪零偏标定功能。

---

## 特性概览 / Features

- 支持 **三轴加速度 + 三轴陀螺仪**（6-DoF）
- SPI 接口，使用 **INT1** 中断引脚触发 data-ready
- 加速度 / 陀螺仪 **ODR 可配置**（最高到 1600/3200 Hz）
- 加速度 / 陀螺仪 **量程可配置**
- 加速度 / 陀螺仪 **数字滤波带宽（bwp）可配置**，支持：
  - OSR4 / OSR2 / NORMAL / CIC（加速度）
  - OSR4 / OSR2 / NORMAL（陀螺仪）
- 内置 **温度控制**：通过 PWM 加热片将 IMU 温度稳定在目标值（如 45°C）
- 内置 **陀螺仪静态零偏标定**：
  - 通过 ramfs 命令行触发
  - 采集约 60s 静止数据，计算偏置并存入数据库
- 通过 LibXR `Topic` 发布：
  - `bmi270_gyro`：机体系角速度（rad/s）
  - `bmi270_accl`：机体系加速度（g）
- 采样线程为 **实时优先级**，基于数据就绪中断，acc/gyr 数据天然对齐

---

## 模块依赖 / Required Hardware

LibXR 视角下，本模块依赖以下硬件组件/服务：

- **RamFS**
  - 用于注册命令文件 `bmi270`（CLI：`show` / `cali` / `list_offset`）
- **Database**
  - 键：`bmi270_gyro_bias`，类型为 `Eigen::Matrix<float, 3, 1>`
  - 存储陀螺仪零偏（单位：rad/s）
- **SPI 总线实例**（名称由 `spi_name` 指定）
  - 连接 BMI270 的 SPI 口
- **GPIO CS 引脚**（名称由 `cs_name` 指定）
  - 片选信号，手动拉低/拉高
- **GPIO INT1 引脚**（名称由 `int1_name` 指定）
  - BMI270 INT1，配置为下降沿中断，驱动数据就绪事件
- **PWM 通道**（名称由 `pwm_name` 指定）
  - 驱动加热片，实现闭环温度控制

Manifest 中 `required_hardware: ramfs database` 是指需要对应的硬件容器对象，SPI/GPIO/PWM 则由 name 查找。

---

## Constructor Arguments

```cpp
BMI270(LibXR::HardwareContainer &hw,
       LibXR::ApplicationManager &app,
       DataRateGyro gyro_datarate,
       DataRateAccel accel_datarate,
       AcclRange accl_range,
       GyroRange gyro_range,
       AcclFilterBwp accl_bwp,
       GyroFilterBwp gyro_bwp,
       LibXR::Quaternion<float> &&rotation,
       LibXR::PID<float>::Param pid_param,
       const char *gyro_topic_name,
       const char *accl_topic_name,
       float target_temperature,
       size_t task_stack_depth,
       const char *spi_name,
       const char *cs_name,
       const char *int1_name,
       const char *pwm_name);

## Template Arguments
None

## Depends
None
