#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Bosch BMI270 六轴 IMU 传感器模块 / Bosch BMI270 6-axis IMU Driver
constructor_args:
  - gyro_datarate: BMI270::DataRateGyro::DATA_RATE_800HZ
  - accel_datarate: BMI270::DataRateAccel::DATA_RATE_800HZ
  - accl_range: BMI270::AcclRange::RANGE_8G
  - gyro_range: BMI270::GyroRange::DPS_2000
  - accl_bwp: BMI270::AcclFilterBwp::NORMAL
  - gyro_bwp: BMI270::GyroFilterBwp::NORMAL
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - pid_param:
      k: 0.2
      p: 1.0
      i: 0.1
      d: 0.0
      i_limit: 0.3
      out_limit: 1.0
      cycle: false
  - gyro_topic_name: "bmi270_gyro"
  - accl_topic_name: "bmi270_accl"
  - target_temperature: 45.0
  - task_stack_depth: 512
  - spi_name: "spi_bmi270"
  - cs_name: "bmi270_cs"
  - int1_name: "bmi270_int1"
  - pwm_name: "pwm_bmi270_heat"
template_args: []
required_hardware: ramfs database
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "Eigen/Core"
#include "app_framework.hpp"
#include "database.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "spi.hpp"
#include "transform.hpp"

class BMI270 : public LibXR::Application
{
 public:
  static constexpr uint8_t REG_CHIP_ID = 0x00;
  static constexpr uint8_t REG_INT_STATUS_1 =
      0x1D;  // [7] = 加速度 DRDY, [6] = 陀螺仪 DRDY
  static constexpr uint8_t REG_DATA_START =
      0x0C;  // 从此地址开始：6 字节加速度 + 6 字节陀螺仪
  static constexpr uint8_t REG_TEMP_0 = 0x22;
  static constexpr uint8_t REG_TEMP_1 = 0x23;
  static constexpr uint8_t REG_ACC_CONF = 0x40;
  static constexpr uint8_t REG_ACC_RANGE = 0x41;
  static constexpr uint8_t REG_GYR_CONF = 0x42;
  static constexpr uint8_t REG_GYR_RANGE = 0x43;
  static constexpr uint8_t REG_INT1_IO_CTRL = 0x53;
  static constexpr uint8_t REG_INT_LATCH = 0x55;
  static constexpr uint8_t REG_INT_MAP_DATA = 0x58;
  static constexpr uint8_t REG_PWR_CONF = 0x7C;
  static constexpr uint8_t REG_PWR_CTRL = 0x7D;
  static constexpr uint8_t REG_CMD = 0x7E;

  static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
  static constexpr uint8_t REG_INIT_CTRL = 0x59;
  static constexpr uint8_t REG_INIT_ADDR_0 = 0x5B;
  static constexpr uint8_t REG_INIT_ADDR_1 = 0x5C;
  static constexpr uint8_t REG_INIT_DATA = 0x5E;

  static constexpr uint8_t CMD_SOFTRESET = 0xB6;
  static constexpr float DEG2RAD = 0.01745329251f;

  // 一次 burst 读取的有效寄存器字节数（不含第 1 个 dummy 字节）
  static constexpr size_t SPI_READ_SIZE = REG_TEMP_1 - REG_DATA_START + 1;

  static const uint8_t BMI270_CONFIG_FILE[8192];  // NOLINT

  // 加速度 ODR 枚举，对应 ACC_CONF[3:0]
  enum class DataRateAccel : uint8_t
  {
    DATA_RATE_0_78HZ = 0x01,
    DATA_RATE_1_56HZ = 0x02,
    DATA_RATE_3_12HZ = 0x03,
    DATA_RATE_6_25HZ = 0x04,
    DATA_RATE_12_5HZ = 0x05,
    DATA_RATE_25HZ = 0x06,
    DATA_RATE_50HZ = 0x07,
    DATA_RATE_100HZ = 0x08,
    DATA_RATE_200HZ = 0x09,
    DATA_RATE_400HZ = 0x0A,
    DATA_RATE_800HZ = 0x0B,
    DATA_RATE_1600HZ = 0x0C,
  };

  // 陀螺仪 ODR 枚举，对应 GYR_CONF[3:0]
  enum class DataRateGyro : uint8_t
  {
    DATA_RATE_25HZ = 0x06,
    DATA_RATE_50HZ = 0x07,
    DATA_RATE_100HZ = 0x08,
    DATA_RATE_200HZ = 0x09,
    DATA_RATE_400HZ = 0x0A,
    DATA_RATE_800HZ = 0x0B,
    DATA_RATE_1600HZ = 0x0C,
    DATA_RATE_3200HZ = 0x0D,
  };

  // 陀螺仪量程，对应 GYR_RANGE
  typedef enum : uint8_t
  {
    DPS_2000 = 0x00,
    DPS_1000 = 0x01,
    DPS_500 = 0x02,
    DPS_250 = 0x03,
    DPS_125 = 0x04,
  } GyroRange;

  // 加速度量程，对应 ACC_RANGE
  typedef enum : uint8_t
  {
    RANGE_2G = 0x00,
    RANGE_4G = 0x01,
    RANGE_8G = 0x02,
    RANGE_16G = 0x03,
  } AcclRange;

  // 加速度带宽 / 滤波器配置，对应 ACC_CONF[6:4]（filter_perf=1 高性能模式）
  enum class AcclFilterBwp : uint8_t
  {
    OSR4 = 0x00,    // OSR4，带宽最窄，抗 aliasing 最好，延时最大
    OSR2 = 0x01,    // OSR2
    NORMAL = 0x02,  // Normal 模式，带宽较宽、延时较小（默认）
    CIC = 0x03,     // CIC 模式
  };

  // 陀螺仪带宽 / 滤波器配置，对应 GYR_CONF[5:4]
  enum class GyroFilterBwp : uint8_t
  {
    OSR4 = 0x00,    // OSR4
    OSR2 = 0x01,    // OSR2
    NORMAL = 0x02,  // Normal 模式（默认）
    // 0x03 保留
  };

  BMI270(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
         DataRateGyro gyro_datarate, DataRateAccel accel_datarate, AcclRange accl_range,
         GyroRange gyro_range, AcclFilterBwp accl_bwp, GyroFilterBwp gyro_bwp,
         LibXR::Quaternion<float> &&rotation, LibXR::PID<float>::Param pid_param,
         const char *gyro_topic_name, const char *accl_topic_name,
         float target_temperature, size_t task_stack_depth, const char *spi_name,
         const char *cs_name, const char *int1_name, const char *pwm_name)
      : data_rate_gyro_(gyro_datarate),
        data_rate_accel_(accel_datarate),
        accl_range_(accl_range),
        gyro_range_(gyro_range),
        accl_bwp_(accl_bwp),
        gyro_bwp_(gyro_bwp),
        rotation_(std::move(rotation)),
        pid_heat_(pid_param),
        target_temperature_(target_temperature),
        topic_gyro_(gyro_topic_name, sizeof(gyro_data_)),
        topic_accl_(accl_topic_name, sizeof(accl_data_)),
        cs_(hw.template FindOrExit<LibXR::GPIO>({cs_name})),
        int1_(hw.template FindOrExit<LibXR::GPIO>({int1_name})),
        spi_(hw.template FindOrExit<LibXR::SPI>({spi_name})),
        pwm_(hw.template FindOrExit<LibXR::PWM>({pwm_name})),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("bmi270", CommandFunc, this)),
        gyro_bias_key_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                       "bmi270_gyro_bias", Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f))
  {
    app.Register(*this);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    // 配置数据就绪中断引脚
    int1_->DisableInterrupt();
    int1_->SetConfig({.direction = LibXR::GPIO::Direction::FALL_INTERRUPT,
                      .pull = LibXR::GPIO::Pull::UP});
    auto cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, BMI270 *self)
        {
          auto now = LibXR::Timebase::GetMicroseconds();
          self->dt_ = now - self->last_sample_ts_;
          self->last_sample_ts_ = now;
          self->new_data_.PostFromCallback(in_isr);
        },
        this);
    int1_->RegisterCallback(cb);

    ReadSingle(REG_CHIP_ID);
    LibXR::Thread::Sleep(10);

    // 初始化失败则软复位后重试
    while (!Init())
    {
      WriteSingle(REG_CMD, CMD_SOFTRESET);
      LibXR::Thread::Sleep(100);
      ReadSingle(REG_CHIP_ID);
      LibXR::Thread::Sleep(10);
    }

    // 采样线程：最高优先级实时线程
    thread_.Create(this, ThreadFunc, "bmi270_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);

    // 温控 PID 定时任务（加热 PWM）
    auto temp_ctrl = LibXR::Timer::CreateTask<BMI270 *>(
        [](BMI270 *self)
        {
          float d = self->pid_heat_.Calculate(self->target_temperature_,
                                              self->temperature_, 0.001f);
          self->pwm_->SetDutyCycle(std::clamp(d, 0.0f, 1.0f));
        },
        this, 1);
    LibXR::Timer::Add(temp_ctrl);
    LibXR::Timer::Start(temp_ctrl);
  }

  // SPI 连续写寄存器
  void WriteBurst(uint8_t reg, const uint8_t *data, size_t len)
  {
    cs_->Write(false);
    spi_->MemWrite(reg, LibXR::ConstRawData(data, len), op_spi_);
    cs_->Write(true);
  }

  // 加载官方配置文件到 BMI270 内部配置区域
  bool LoadConfigFile()
  {
    // 1) INIT_CTRL = 0x00，准备加载配置
    WriteSingle(REG_INIT_CTRL, 0x00);
    // 官方建议有一个最小延时，这里给 1ms
    LibXR::Thread::Sleep(1);

    constexpr size_t CHUNK = 32;  // 每次写 32 字节，相比 4 字节效率更高
    constexpr size_t TOTAL = sizeof(BMI270_CONFIG_FILE);
    size_t off = 0;

    while (off < TOTAL)
    {
      size_t n = std::min(CHUNK, TOTAL - off);

      // BMI270 配置文件地址以 16bit half-word 为单位
      // INIT_ADDR_0[3:0] = base[3:0]
      // INIT_ADDR_1[7:0] = base[11:4]
      uint16_t base = static_cast<uint16_t>(off / 2);
      uint8_t addr2[2] = {
          static_cast<uint8_t>(base & 0x0Fu),
          static_cast<uint8_t>((base >> 4) & 0xFFu),
      };

      // 2) 写 INIT_ADDR_0 / INIT_ADDR_1（起始寄存器 0x5B，连续两字节）
      cs_->Write(false);
      uint8_t hdr_addr = static_cast<uint8_t>(REG_INIT_ADDR_0 & 0x7F);  // 确保 MSB=0
      spi_->MemWrite(hdr_addr, LibXR::ConstRawData(addr2, 2), op_spi_);
      cs_->Write(true);

      // 3) 把本块数据写进 INIT_DATA(0x5E)
      WriteBurst(REG_INIT_DATA, &BMI270_CONFIG_FILE[off], n);

      off += n;
    }

    // 4) INIT_CTRL = 0x01，触发配置生效
    WriteSingle(REG_INIT_CTRL, 0x01);

    // 5) 等 INTERNAL_STATUS[3:0] == 0b0001 表示配置加载完成
    auto t0 = LibXR::Timebase::GetMilliseconds();
    while (LibXR::Timebase::GetMilliseconds() - t0 < 20)
    {
      uint8_t st = ReadSingle(REG_INTERNAL_STATUS);
      if ((st & 0x0F) == 0x01)
      {
        return true;
      }
      LibXR::Thread::Sleep(1);
    }

    uint8_t st = ReadSingle(REG_INTERNAL_STATUS);
    XR_LOG_WARN("BMI270: config load timeout, INTERNAL_STATUS=0x%02X", st);
    return false;
  }

  void OnMonitor() override
  {
    if (!std::isfinite(gyro_data_.x()) || !std::isfinite(gyro_data_.y()) ||
        !std::isfinite(gyro_data_.z()) || !std::isfinite(accl_data_.x()) ||
        !std::isfinite(accl_data_.y()) || !std::isfinite(accl_data_.z()))
    {
      XR_LOG_WARN("BMI270: bad data");
    }
    float ideal_dt = IdealDt();
    if (ideal_dt > 0.0f && std::fabs(dt_.ToSecondf() - ideal_dt) > 0.00015f)
    {
      XR_LOG_WARN("BMI270 dt=%d", dt_);
    }
  }

 private:
  bool Init()
  {
    // 1) SPI 模式选择：先读一次寄存器（结果丢弃），切换到 SPI 模式
    (void)ReadSingle(REG_CHIP_ID);

    // 2) 关闭高级省电模式（只改 bit0，避免破坏其他位）
    uint8_t pwr_conf = ReadSingle(REG_PWR_CONF);
    pwr_conf &= ~0x01u;  // adv_power_save = 0
    WriteSingle(REG_PWR_CONF, pwr_conf);

    // 3) 等待 ≥ 450us，这里取 1ms 留裕量
    LibXR::Thread::Sleep(1);

    // 4) 再读一次 CHIP_ID 确认通信正常
    if (ReadSingle(REG_CHIP_ID) != 0x24)
    {
      XR_LOG_WARN("BMI270: bad CHIP_ID");
      return false;
    }

    // 5) 加载配置文件
    if (!LoadConfigFile())
    {
      uint8_t st = ReadSingle(REG_INTERNAL_STATUS);
      XR_LOG_WARN("BMI270: LoadConfigFile_ failed, INTERNAL_STATUS=0x%02X", st);
      return false;
    }

    // 6) 打开 ACC / GYR 电源（bit[3]=GYR_EN, bit[2]=ACC_EN）
    WriteSingle(REG_PWR_CTRL, 0x0E);

    // 高性能模式：滤波性能 / 噪声性能都打开
    static constexpr uint8_t ACC_FILTER_PERF_HP = 0x80;  // ACC_CONF bit7 = 1
    static constexpr uint8_t GYR_FILTER_PERF_HP = 0x80;  // GYR_CONF bit7 = 1
    static constexpr uint8_t GYR_NOISE_PERF_HP = 0x40;   // GYR_CONF bit6 = 1

    // 7) 配置 ACC：高性能 + 指定带宽 + 用户指定 ODR
    uint8_t acc_conf = ACC_FILTER_PERF_HP | (static_cast<uint8_t>(accl_bwp_) << 4) |
                       (static_cast<uint8_t>(data_rate_accel_) & 0x0F);
    WriteSingle(REG_ACC_CONF, acc_conf);
    WriteSingle(REG_ACC_RANGE, static_cast<uint8_t>(accl_range_));

    // 8) 配置 GYR：滤波和噪声都设为高性能 + 指定带宽 + 用户指定 ODR
    uint8_t gyr_conf = GYR_FILTER_PERF_HP | GYR_NOISE_PERF_HP |
                       (static_cast<uint8_t>(gyro_bwp_) << 4) |
                       (static_cast<uint8_t>(data_rate_gyro_) & 0x0F);
    WriteSingle(REG_GYR_CONF, gyr_conf);
    WriteSingle(REG_GYR_RANGE, static_cast<uint8_t>(gyro_range_));

    // 9) 配置 INT1：推挽输出，高电平有效；映射加速度/陀螺仪数据就绪中断
    WriteSingle(REG_INT1_IO_CTRL, 0x0A);  // push-pull, active high
    WriteSingle(REG_INT_MAP_DATA, 0x44);  // gyr/acc drdy -> INT1
    WriteSingle(REG_INT_LATCH, 0x00);     // 非锁存模式（脉冲）

    int1_->EnableInterrupt();

    // 10) 初始化时间戳，用于 dt 监控
    last_sample_ts_ = LibXR::Timebase::GetMicroseconds();

    return true;
  }

  // 采样线程：等待 data ready 事件，读取一帧 IMU 数据并发布
  static void ThreadFunc(BMI270 *self)
  {
    self->pwm_->SetConfig({30000});
    self->pwm_->SetDutyCycle(0);
    self->pwm_->Enable();

    while (true)
    {
      bool got = (self->new_data_.Wait(100) == ErrorCode::OK);
      if (!got)
      {
        // 如果信号丢失，主动读取一次中断状态，避免卡死
        uint8_t ist = self->ReadSingle(REG_INT_STATUS_1);
        if (ist & 0x40)
        {
          got = true;
        }
      }
      if (got)
      {
        // 一次 burst 读取：dummy(1) + ACC(6) + GYR(6) + TEMP(2)
        self->ReadBurst(self->REG_DATA_START, self->buffer_, SPI_READ_SIZE + 1);
        self->Parse();
        self->topic_accl_.Publish(self->accl_data_);
        self->topic_gyro_.Publish(self->gyro_data_);
      }
    }
  }

  // 写单个寄存器，并回读确认写入成功
  void WriteSingle(uint8_t reg, uint8_t data)
  {
    do
    {
      cs_->Write(false);
      spi_->MemWrite(reg, data, op_spi_);
      cs_->Write(true);
    } while (ReadSingle(reg) != data);
  }

  // 读单个寄存器（BMI270 第 1 字节为 dummy，第 2 字节才是数据）
  uint8_t ReadSingle(uint8_t reg)
  {
    uint8_t d[2];
    cs_->Write(false);
    spi_->MemRead(reg, d, op_spi_);
    cs_->Write(true);
    return d[1];
  }

  // 读多个连续寄存器：out[0] 为 dummy，真正数据从 out[1] 开始
  void ReadBurst(uint8_t reg, uint8_t *out, uint8_t len)
  {
    cs_->Write(false);
    spi_->MemRead(reg, {out, len}, op_spi_);
    cs_->Write(true);
  }

  // 解析一帧传感器原始数据，并应用坐标旋转和零偏
  void Parse()
  {
    // 跳过第一个 dummy 字节
    auto raw_data = buffer_ + 1;

    // ---------- 加速度 ----------
    int16_t ax = static_cast<int16_t>(raw_data[1] << 8 | raw_data[0]);
    int16_t ay = static_cast<int16_t>(raw_data[3] << 8 | raw_data[2]);
    int16_t az = static_cast<int16_t>(raw_data[5] << 8 | raw_data[4]);
    float acc_lsb = GetAcclLSB();
    Eigen::Matrix<float, 3, 1> acc(static_cast<float>(ax) * acc_lsb,
                                   static_cast<float>(ay) * acc_lsb,
                                   static_cast<float>(az) * acc_lsb);

    // ---------- 陀螺仪 ----------
    int16_t gx = static_cast<int16_t>(raw_data[7] << 8 | raw_data[6]);
    int16_t gy = static_cast<int16_t>(raw_data[9] << 8 | raw_data[8]);
    int16_t gz = static_cast<int16_t>(raw_data[11] << 8 | raw_data[10]);
    float gyr_lsb = GetGyroLSB();
    Eigen::Matrix<float, 3, 1> gyr(static_cast<float>(gx) * gyr_lsb * DEG2RAD,
                                   static_cast<float>(gy) * gyr_lsb * DEG2RAD,
                                   static_cast<float>(gz) * gyr_lsb * DEG2RAD);

    // 先应用机体坐标旋转，再减去 gyro 零偏
    accl_data_ = rotation_ * acc;
    gyro_data_ = rotation_ * Eigen::Vector3f(gyr - gyro_bias_key_.data_);

    // 若当前处于校准阶段，则累积原始陀螺仪数据
    if (in_cali_)
    {
      gyro_cali_[0] += gx;
      gyro_cali_[1] += gy;
      gyro_cali_[2] += gz;
      cali_counter_++;
    }

    // ---------- 温度 ----------
    // REG_TEMP_0 / 1 在 burst 中的索引
    static constexpr int TEMP0_IDX = REG_TEMP_0 - REG_DATA_START;  // 22
    static constexpr int TEMP1_IDX = REG_TEMP_1 - REG_DATA_START;  // 23

    int16_t traw = static_cast<int16_t>((raw_data[TEMP1_IDX]) << 8 | raw_data[TEMP0_IDX]);
    if (traw == static_cast<int16_t>(0x8000))
    {
      // 特殊值 0x8000 表示温度无效
      temperature_ = NAN;
    }
    else
    {
      // datasheet 中给出的温度转换公式
      temperature_ = 23.0f + static_cast<float>(traw) / 512.0f;
    }
  }

  // 根据 ODR 计算理论采样周期（秒），用于 dt 监控
  float IdealDt() const
  {
    switch (data_rate_gyro_)
    {
      case DataRateGyro::DATA_RATE_25HZ:
        return 0.04f;
      case DataRateGyro::DATA_RATE_50HZ:
        return 0.02f;
      case DataRateGyro::DATA_RATE_100HZ:
        return 0.01f;
      case DataRateGyro::DATA_RATE_200HZ:
        return 0.005f;
      case DataRateGyro::DATA_RATE_400HZ:
        return 0.0025f;
      case DataRateGyro::DATA_RATE_800HZ:
        return 0.00125f;
      case DataRateGyro::DATA_RATE_1600HZ:
        return 0.000625f;
      case DataRateGyro::DATA_RATE_3200HZ:
        return 0.0003125f;
    }
    return 0.0f;
  }

  // 不同加速度量程下，LSB 对应的 g 值
  float GetAcclLSB() const
  {
    switch (accl_range_)
    {
      case RANGE_2G:
        return 1.0f / 16384.0f;
      case RANGE_4G:
        return 1.0f / 8192.0f;
      case RANGE_8G:
        return 1.0f / 4096.0f;
      case RANGE_16G:
        return 1.0f / 2048.0f;
    }
    // 默认按 2g 处理
    return 1.0f / 16384.0f;
  }

  // 不同陀螺仪量程下，LSB 对应的 dps 值
  float GetGyroLSB() const
  {
    switch (gyro_range_)
    {
      case DPS_2000:
        return 1.0f / 16.384f;
      case DPS_1000:
        return 1.0f / 32.768f;
      case DPS_500:
        return 1.0f / 65.536f;
      case DPS_250:
        return 1.0f / 131.072f;
      case DPS_125:
        return 1.0f / 262.144f;
    }
    // 默认按 2000dps 处理
    return 1.0f / 16.384f;
  }

  // 命令行接口：show / list_offset / cali
  static int CommandFunc(BMI270 *self, int argc, char **argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf("  show [time_ms] [interval_ms]\r\n");
      LibXR::STDIO::Printf("  list_offset\r\n");
      LibXR::STDIO::Printf("  cali\r\n");
      return 0;
    }
    else if (argc == 2)
    {
      if (strcmp(argv[1], "list_offset") == 0)
      {
        LibXR::STDIO::Printf("bias: %f %f %f\r\n", self->gyro_bias_key_.data_.x(),
                             self->gyro_bias_key_.data_.y(),
                             self->gyro_bias_key_.data_.z());
        return 0;
      }
      else if (strcmp(argv[1], "cali") == 0)
      {
        // 清零当前陀螺仪偏置和累积数据
        self->gyro_bias_key_.data_.setZero();
        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0, 0, 0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;

        LibXR::STDIO::Printf(
            "Starting BMI270 gyroscope calibration. Please "
            "keep the device steady.\r\n");

        // 给用户一点时间把设备放稳
        LibXR::Thread::Sleep(3000);

        // 采集 60 秒数据
        for (int i = 0; i < 60; i++)
        {
          LibXR::STDIO::Printf("Progress: %d / 60\r", i + 1);
          LibXR::Thread::Sleep(1000);
        }
        LibXR::STDIO::Printf("\r\nProgress: Done\r\n");

        // 停止累积
        self->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        if (self->cali_counter_ == 0)
        {
          LibXR::STDIO::Printf("BMI270 calibration failed: no samples collected.\r\n");
          return -1;
        }

        // 计算平均 raw，转换为 rad/s 偏置
        float s = self->GetGyroLSB();

        // NOLINTBEGIN
        self->gyro_bias_key_.data_.x() = static_cast<double>(self->gyro_cali_.data()[0]) /
                                         static_cast<double>(self->cali_counter_) * s *
                                         DEG2RAD;
        self->gyro_bias_key_.data_.y() = static_cast<double>(self->gyro_cali_.data()[1]) /
                                         static_cast<double>(self->cali_counter_) * s *
                                         DEG2RAD;
        self->gyro_bias_key_.data_.z() = static_cast<double>(self->gyro_cali_.data()[2]) /
                                         static_cast<double>(self->cali_counter_) * s *
                                         DEG2RAD;
        // NOLINTEND

        LibXR::STDIO::Printf("\r\nBMI270 calibration result - x: %f, y: %f, z: %f\r\n",
                             self->gyro_bias_key_.data_.x(),
                             self->gyro_bias_key_.data_.y(),
                             self->gyro_bias_key_.data_.z());

        LibXR::STDIO::Printf("Analyzing calibration quality...\r\n");

        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0, 0, 0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;

        for (int i = 0; i < 60; i++)
        {
          LibXR::STDIO::Printf("Progress: %d / 60\r", i + 1);
          LibXR::Thread::Sleep(1000);
        }
        LibXR::STDIO::Printf("\r\nProgress: Done\r\n");

        self->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        // 再算一次平均 raw，换算为 rad/s（此时应该接近刚刚求出的偏置）
        double avg_x2 = static_cast<double>(self->gyro_cali_.data()[0]) /
                        static_cast<double>(self->cali_counter_) * s * DEG2RAD;
        double avg_y2 = static_cast<double>(self->gyro_cali_.data()[1]) /
                        static_cast<double>(self->cali_counter_) * s * DEG2RAD;
        double avg_z2 = static_cast<double>(self->gyro_cali_.data()[2]) /
                        static_cast<double>(self->cali_counter_) * s * DEG2RAD;

        double err_x = avg_x2 - self->gyro_bias_key_.data_.x();
        double err_y = avg_y2 - self->gyro_bias_key_.data_.y();
        double err_z = avg_z2 - self->gyro_bias_key_.data_.z();

        LibXR::STDIO::Printf("\r\nBMI270 calibration error - x: %f, y: %f, z: %f\r\n",
                             err_x, err_y, err_z);

        // 存进数据库
        self->gyro_bias_key_.Set(self->gyro_bias_key_.data_);
        LibXR::STDIO::Printf("BMI270 calibration data saved.\r\n");
        return 0;
      }
    }
    else if (argc == 4 && strcmp(argv[1], "show") == 0)
    {
      int time = std::atoi(argv[2]);
      int delay = std::atoi(argv[3]);
      delay = std::clamp(delay, 2, 1000);
      while (time > 0)
      {
        LibXR::STDIO::Printf(
            "acc:%+5f %+5f %+5f | gyr:%+5f %+5f %+5f | T:%4.2f\r\n", self->accl_data_.x(),
            self->accl_data_.y(), self->accl_data_.z(), self->gyro_data_.x(),
            self->gyro_data_.y(), self->gyro_data_.z(), self->temperature_);
        LibXR::Thread::Sleep(delay);
        time -= delay;
      }
      return 0;
    }
    LibXR::STDIO::Printf("bad args\r\n");
    return -1;
  }

 private:
  DataRateGyro data_rate_gyro_;
  DataRateAccel data_rate_accel_;
  AcclRange accl_range_;
  GyroRange gyro_range_;

  // 加速度 / 陀螺仪滤波器带宽配置（由构造函数注入）
  AcclFilterBwp accl_bwp_;
  GyroFilterBwp gyro_bwp_;

  uint8_t buffer_[SPI_READ_SIZE + 1];
  Eigen::Matrix<float, 3, 1> gyro_data_{0, 0, 0}, accl_data_{0, 0, 0};

  LibXR::Quaternion<float> rotation_;
  LibXR::PID<float> pid_heat_;
  float temperature_ = 0.0f;
  float target_temperature_ = 25.0f;

  LibXR::Topic topic_gyro_, topic_accl_;

  LibXR::GPIO *cs_ = nullptr;
  LibXR::GPIO *int1_ = nullptr;
  LibXR::SPI *spi_ = nullptr;
  LibXR::PWM *pwm_ = nullptr;

  LibXR::Semaphore sem_spi_, new_data_;
  LibXR::SPI::OperationRW op_spi_;

  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_bias_key_;

  // 陀螺仪零偏校准相关变量
  bool in_cali_ = false;
  uint32_t cali_counter_ = 0;
  Eigen::Matrix<int64_t, 3, 1> gyro_cali_{0, 0, 0};

  // 采样周期监控用时间戳
  LibXR::MicrosecondTimestamp last_sample_ts_ = 0;
  LibXR::MicrosecondTimestamp::Duration dt_ = 0;

  LibXR::Thread thread_;
};
