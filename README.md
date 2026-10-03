# BMI270

博世 BMI270 6 轴 IMU（SPI）驱动模块 / Driver Module for the Bosch BMI270 6-axis IMU over SPI

## 1. 模块作用 / Purpose

构造时，BMI270 把 INT1 引脚配置为上升沿中断（下拉），读取 `CHIP_ID` 以切换到 SPI 模式，关闭高级省电，校验芯片 ID `0x24`，加载 Bosch 官方 8 KB 配置文件，再打开加速度计与陀螺仪，并按 `Param` 写入 ODR、量程和滤波带宽（加速度计 filter_perf 为高性能；陀螺仪 filter_perf 与 noise_perf 为高性能）。INT1 设为推挽、高电平有效、非锁存，加速度计与陀螺仪的 data-ready 都映射到 INT1。任一步失败时软复位后重试，直到成功。单个寄存器写入后回读，读回值与写入值不一致时重复写入，最多 10 次；仍不一致则输出 `BMI270: write verify failed` 警告并记为该步失败。

采样线程 `bmi270_thread`（REALTIME 优先级）等待 data-ready 中断；100 ms 内没有中断时读取 `INT_STATUS_1`，陀螺仪 data-ready 置位则照常读取。每次以一个 burst 读出加速度、角速度和温度，换算后发布。加速度单位为 g，乘以 `rotation`；角速度单位为 rad/s，先减去零偏再乘以 `rotation`。温度为 `23 + raw / 512`（°C），原始值 `0x8000` 视为无效（NaN）。

初始化成功后，构造函数先把加热 PWM 配置为 30 kHz、占空比 0 并使能，再创建采样线程与一个周期 1 ms 的 LibXR 定时器任务；该任务用 `pid_param` 计算占空比，限制在 0 到 1，使温度趋近 `target_temperature`。

陀螺仪零偏（rad/s）保存在 Database 的键 `bmi270_gyro_bias`（`Eigen::Matrix<float, 3, 1>`）中。

`OnMonitor()` 在数据不是有限值时输出 `BMI270: bad data`，并在中断间隔偏离陀螺仪 ODR 对应的理想周期超过 150 µs 时输出实际间隔。

模块在 RamFS 中注册命令 `bmi270`：

- `bmi270`：打印用法。
- `bmi270 show <time_ms> <interval_ms>`：在 `time_ms` 内每隔 `interval_ms`（限制在 2 到 1000 ms）打印一次加速度、角速度和温度。
- `bmi270 list_offset`：打印当前陀螺仪零偏。
- `bmi270 cali`：陀螺仪零偏校准，期间设备保持静止。先等待 3 s，再采集 60 s 求平均零偏，然后采集 60 s 打印残差，最后把零偏写入 Database。

Upon construction, BMI270 configures the INT1 pin as a rising-edge interrupt (pull-down), reads `CHIP_ID` to switch to SPI mode, disables advanced power save, checks the chip ID `0x24`, loads the official 8 KB Bosch configuration file, then enables the accelerometer and the gyroscope and writes the ODR, range and filter bandwidth from `Param` (accelerometer filter_perf in high-performance mode; gyroscope filter_perf and noise_perf in high-performance mode). INT1 is set to push-pull, active high, non-latched, and the data-ready signals of both the accelerometer and the gyroscope are mapped to INT1. When any step fails, the sensor is soft-reset and initialization is retried until it succeeds. After each single-register write, the register is read back; on a mismatch the write is repeated, up to 10 attempts in total, after which a `BMI270: write verify failed` warning is logged and the step counts as failed.

The sampling thread `bmi270_thread` (REALTIME priority) waits for the data-ready interrupt; when no interrupt arrives within 100 ms, it reads `INT_STATUS_1`, and a set gyroscope data-ready bit is handled as usual. Each time one burst reads the acceleration, angular velocity and temperature, and the converted values are published. The acceleration unit is g, multiplied by `rotation`; the angular velocity unit is rad/s, with the zero offset subtracted before the multiplication by `rotation`. The temperature is `23 + raw / 512` (°C), and the raw value `0x8000` is treated as invalid (NaN).

After a successful initialization, the constructor first configures the heater PWM to 30 kHz with a duty cycle of 0 and enables it, then creates the sampling thread and a LibXR timer task with a 1 ms period; the task computes the duty cycle with `pid_param`, limits it to 0 to 1, and drives the temperature toward `target_temperature`.

The gyroscope zero offset (rad/s) is stored in the Database under the key `bmi270_gyro_bias` (`Eigen::Matrix<float, 3, 1>`).

`OnMonitor()` logs `BMI270: bad data` when the data is not finite, and logs the actual interval when the interrupt interval deviates from the ideal period of the gyroscope ODR by more than 150 µs.

The Module registers the command `bmi270` in RamFS:

- `bmi270`: print the usage.
- `bmi270 show <time_ms> <interval_ms>`: print the acceleration, angular velocity and temperature every `interval_ms` (limited to 2 to 1000 ms) for `time_ms`.
- `bmi270 list_offset`: print the current gyroscope zero offset.
- `bmi270 cali`: gyroscope zero-offset calibration, with the device held still. It waits 3 s, collects 60 s to average the zero offset, collects another 60 s to print the residual, and finally writes the zero offset to the Database.

## 2. 构造接口 / Constructor

```cpp
BMI270(LibXR::GPIO& cs,
       LibXR::GPIO& int1,
       LibXR::SPI& spi,
       LibXR::PWM& pwm,
       LibXR::Database& database,
       LibXR::RamFS& ramfs,
       const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `cs`：片选 GPIO（输出，低有效），由模块手动控制。
- `int1`：BMI270 INT1 引脚的 GPIO，模块将其配置为上升沿中断。
- `spi`：连接 BMI270 的 `LibXR::SPI`。
- `pwm`：IMU 加热片的 `LibXR::PWM`。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：注册 `bmi270` 命令的 `LibXR::RamFS`。

配置参数（`Param`）：

- `gyro_datarate`：陀螺仪 ODR，默认 `DATA_RATE_800HZ`，范围 25 Hz 到 3200 Hz。
- `accel_datarate`：加速度计 ODR，默认 `DATA_RATE_800HZ`，范围 0.78 Hz 到 1600 Hz。
- `accl_range`：加速度计量程，默认 `RANGE_8G`；可选 `RANGE_2G`、`RANGE_4G`、`RANGE_8G`、`RANGE_16G`。
- `gyro_range`：陀螺仪量程，默认 `DPS_2000`；可选 `DPS_2000`、`DPS_1000`、`DPS_500`、`DPS_250`、`DPS_125`。
- `accl_bwp`：加速度计滤波带宽，默认 `NORMAL`；可选 `OSR4`、`OSR2`、`NORMAL`、`CIC`。
- `gyro_bwp`：陀螺仪滤波带宽，默认 `NORMAL`；可选 `OSR4`、`OSR2`、`NORMAL`。
- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`，默认单位四元数。
- `pid_param`：温控 PID，`LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`，默认 `k = 0.2`、`p = 1.0`、`i = 0.1`、`d = 0`、`i_limit = 0.3`、`out_limit = 1.0`、`cycle = false`。
- `gyro_topic_name`、`accl_topic_name`：发布的 Topic 名称，默认 `"bmi270_gyro"`、`"bmi270_accl"`。
- `target_temperature`：目标温度，单位 °C，默认 45。
- `task_stack_depth`：采样线程栈深，默认 512。

Dependencies:

- `cs`: chip-select GPIO (output, active low), driven manually by the Module.
- `int1`: GPIO of the BMI270 INT1 pin; the Module configures it as a rising-edge interrupt.
- `spi`: the `LibXR::SPI` connected to the BMI270.
- `pwm`: the `LibXR::PWM` of the IMU heating element.
- `database`: the `LibXR::Database` that stores the gyroscope zero offset.
- `ramfs`: the `LibXR::RamFS` that receives the `bmi270` command.

Configuration parameters (`Param`):

- `gyro_datarate`: gyroscope ODR, default `DATA_RATE_800HZ`, range 25 Hz to 3200 Hz.
- `accel_datarate`: accelerometer ODR, default `DATA_RATE_800HZ`, range 0.78 Hz to 1600 Hz.
- `accl_range`: accelerometer range, default `RANGE_8G`; options are `RANGE_2G`, `RANGE_4G`, `RANGE_8G`, `RANGE_16G`.
- `gyro_range`: gyroscope range, default `DPS_2000`; options are `DPS_2000`, `DPS_1000`, `DPS_500`, `DPS_250`, `DPS_125`.
- `accl_bwp`: accelerometer filter bandwidth, default `NORMAL`; options are `OSR4`, `OSR2`, `NORMAL`, `CIC`.
- `gyro_bwp`: gyroscope filter bandwidth, default `NORMAL`; options are `OSR4`, `OSR2`, `NORMAL`.
- `rotation`: quaternion `{w, x, y, z}` from the sensor frame to the application frame, default identity.
- `pid_param`: temperature-control PID, `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`, default `k = 0.2`, `p = 1.0`, `i = 0.1`, `d = 0`, `i_limit = 0.3`, `out_limit = 1.0`, `cycle = false`.
- `gyro_topic_name`, `accl_topic_name`: names of the published Topics, default `"bmi270_gyro"` and `"bmi270_accl"`.
- `target_temperature`: target temperature in °C, default 45.
- `task_stack_depth`: stack depth of the sampling thread, default 512.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `gyro_topic_name`（默认 `bmi270_gyro`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s，已去零偏并旋转 |
| `accl_topic_name`（默认 `bmi270_accl`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 g，已旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `gyro_topic_name` (default `bmi270_gyro`) | Publish | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s, zero offset removed and rotated |
| `accl_topic_name` (default `bmi270_accl`) | Publish | `Eigen::Matrix<float, 3, 1>` | Acceleration in g, rotated |

## 4. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/BMI270` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/BMI270`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

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
          rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
          pid_param:
            k: 0.2f
            p: 1.0f
            i: 0.1f
            d: 0.0f
            i_limit: 0.3f
            out_limit: 1.0f
            cycle: false
          gyro_topic_name: "bmi270_gyro"
          accl_topic_name: "bmi270_accl"
          target_temperature: 45.0f
          task_stack_depth: 512
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。`BMI270.cpp` 中的 `BMI270_CONFIG_FILE` 是 Bosch Sensortec BMI270 Sensor API 的配置文件镜像，按 BSD-3-Clause 授权，详见 `NOTICE`。

硬件：一片通过 SPI 连接的 BMI270，带片选 GPIO 与 INT1 中断 GPIO，另有一路驱动加热片的 PWM；SPI、GPIO、PWM、Database 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR. `BMI270_CONFIG_FILE` in `BMI270.cpp` mirrors the configuration file of the Bosch Sensortec BMI270 Sensor API and is licensed under BSD-3-Clause; see `NOTICE`.

Hardware: one BMI270 connected over SPI, with a chip-select GPIO and an INT1 interrupt GPIO, and one PWM output that drives the heating element; the SPI, GPIOs, PWM, Database and RamFS are registered by the BSP with `XR_REGISTER`.
