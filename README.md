# BMI270

Bosch BMI270 六轴 IMU 驱动模块（BMI270 6-axis IMU Driver）。

模块通过 SPI 驱动 BMI270，发布旋转到应用坐标系后的三轴加速度与三轴角速度，并内置加热片
温控（PID）和陀螺仪零偏标定。

## 行为

- 构造时把 INT1 引脚配置为上升沿中断（下拉），读取 `CHIP_ID` 切换到 SPI 模式，关闭高级省电，
  校验芯片 ID `0x24`，加载 Bosch 官方 8 KB 配置文件，再打开加速度计 / 陀螺仪并按 `Param`
  写入 ODR、量程和滤波带宽（加速度计 filter_perf 高性能；陀螺仪 filter_perf 与 noise_perf
  高性能）。INT1 设为推挽、高电平有效、非锁存，加速度与陀螺仪 data-ready 都映射到 INT1。
  任一步失败会软复位后重试，直到成功。
- 单寄存器写入后会回读，直到读回值与写入值一致。
- 采样线程 `bmi270_thread`（REALTIME 优先级）等待 data-ready 中断；100 ms 内没有中断时主动
  读取 `INT_STATUS_1`，陀螺仪 data-ready 置位则照常读取。每次一个 burst 同时读出加速度、
  陀螺仪和温度，换算后发布。
- 加速度单位 g，乘以 `rotation`；角速度单位 rad/s，先减去零偏再乘以 `rotation`。温度为
  `23 + raw / 512` °C，原始值 `0x8000` 视为无效（NaN）。
- 加热 PWM 以 30 kHz 运行；一个 1 ms 周期的 LibXR 定时器任务用 `pid_param` 计算占空比并
  限制在 0-1，使温度趋近 `target_temperature`。
- 陀螺仪零偏（rad/s）保存在 Database 键 `bmi270_gyro_bias`（`Eigen::Matrix<float, 3, 1>`）中。
- `OnMonitor()` 在数据非有限值时打印 `BMI270: bad data`，并在中断间隔偏离陀螺仪 ODR 理想周期
  超过 150 µs 时打印实际间隔。

## Topic

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `gyro_topic_name`（默认 `bmi270_gyro`） | `Eigen::Matrix<float, 3, 1>` | 角速度，rad/s，已去零偏并旋转 |
| `accl_topic_name`（默认 `bmi270_accl`） | `Eigen::Matrix<float, 3, 1>` | 加速度，g，已旋转 |

## RamFS 命令

模块在 RamFS 中注册 `bmi270` 命令：

- `bmi270`：打印用法。
- `bmi270 show <time_ms> <interval_ms>`：每 `interval_ms`（限制在 2-1000 ms）打印一次
  加速度、角速度和温度，持续 `time_ms`。
- `bmi270 list_offset`：打印当前陀螺仪零偏。
- `bmi270 cali`：陀螺仪零偏标定。设备需保持静止：等待 3 s 后采集 60 s 求平均零偏，再采集
  60 s 打印残差，最后把零偏写入 Database。

## 依赖

无其他模块依赖，仅使用 LibXR。`BMI270.cpp` 中的 `BMI270_CONFIG_FILE` 是 Bosch Sensortec
BMI270 Sensor API 的配置文件镜像，按 BSD-3-Clause 授权，详见 `NOTICE`。

## 构造接口

```cpp
BMI270(LibXR::GPIO& cs,
       LibXR::GPIO& int1,
       LibXR::SPI& spi,
       LibXR::PWM& pwm,
       LibXR::Database& database,
       LibXR::RamFS& ramfs,
       const Param& param = {...});
```

依赖：

- `cs`：片选 GPIO（输出，低有效），由模块手动控制。
- `int1`：BMI270 INT1 引脚 GPIO，模块将其配置为上升沿中断。
- `spi`：连接 BMI270 的 `LibXR::SPI`。
- `pwm`：IMU 加热片的 `LibXR::PWM`。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：注册 `bmi270` 命令的 `LibXR::RamFS`。

配置（`Param` 字段，括号内为默认值）：

- `gyro_datarate`：陀螺仪 ODR（`DATA_RATE_800HZ`），25 Hz 到 3200 Hz。
- `accel_datarate`：加速度计 ODR（`DATA_RATE_800HZ`），0.78 Hz 到 1600 Hz。
- `accl_range`：加速度计量程（`RANGE_8G`），可选 `RANGE_2G` / `RANGE_4G` / `RANGE_8G` /
  `RANGE_16G`。
- `gyro_range`：陀螺仪量程（`DPS_2000`），可选 `DPS_2000` / `DPS_1000` / `DPS_500` /
  `DPS_250` / `DPS_125`。
- `accl_bwp`：加速度计滤波带宽（`NORMAL`），可选 `OSR4` / `OSR2` / `NORMAL` / `CIC`。
- `gyro_bwp`：陀螺仪滤波带宽（`NORMAL`），可选 `OSR4` / `OSR2` / `NORMAL`。
- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`（单位四元数）。
- `pid_param`：温控 PID 参数（`k = 0.2`、`p = 1.0`、`i = 0.1`、`d = 0`、`i_limit = 0.3`、
  `out_limit = 1.0`）。
- `gyro_topic_name` / `accl_topic_name`：Topic 名称（`"bmi270_gyro"` / `"bmi270_accl"`）。
- `target_temperature`：目标温度，°C（45）。
- `task_stack_depth`：采样线程栈深（512）。

## 使用

```sh
xrobot module add xrobot-org/BMI270
xrobot setup
xrobot instance add xrobot-org/BMI270
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：

```yaml
modules:
  - module: xrobot-org/BMI270
    id: bmi270_0
    args:
      - cs: bmi270_cs
      - int1: bmi270_int1
      - spi: spi1
      - pwm: imu_heat_pwm
      - database: database
      - ramfs: ramfs
      - param:
          gyro_datarate: BMI270::DataRateGyro::DATA_RATE_800HZ
          accel_datarate: BMI270::DataRateAccel::DATA_RATE_800HZ
          accl_range: BMI270::AcclRange::RANGE_8G
          gyro_range: BMI270::GyroRange::DPS_2000
          accl_bwp: BMI270::AcclFilterBwp::NORMAL
          gyro_bwp: BMI270::GyroFilterBwp::NORMAL
          rotation:
            - 1.0f
            - 0.0f
            - 0.0f
            - 0.0f
          pid_param:
            k: 0.2f
            p: 1.0f
            i: 0.1f
            d: 0.0f
            i_limit: 0.3f
            out_limit: 1.0f
            cycle: 'false'
          gyro_topic_name: '"bmi270_gyro"'
          accl_topic_name: '"bmi270_accl"'
          target_temperature: 45.0f
          task_stack_depth: '512'
```

BSP 侧：

```cpp
XR_REGISTER(bmi270_cs, LibXR::GPIO);
XR_REGISTER(bmi270_int1, LibXR::GPIO);
XR_REGISTER(spi1, LibXR::SPI);
XR_REGISTER(imu_heat_pwm, LibXR::PWM);
XR_REGISTER(database, LibXR::Database);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/BMI270`
（在 BSP 中）打印 manifest 和当前的构造函数。
