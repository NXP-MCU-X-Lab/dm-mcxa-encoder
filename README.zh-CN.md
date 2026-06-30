# NXP MCXA344 双轨感应式编码器

> [English](README.md) | 中文

这是一个面向 HW_V2 双轨感应式绝对编码器的参考固件和上位机仪表板项目，目标器件口径为
NXP MCXA34x / MCXA344 系列。固件将 16/15 Vernier 双轨信号解算为单圈绝对角，
在 RAM 中累计多圈位置，并通过 FreeMASTER 暴露实时数据。

![HW_V2 感应式编码器硬件](doc/HW_V2.jpg)

## 功能

- 16/15 Vernier 单圈绝对角解算，输出 16-bit 角度计数
- 每条 `(sin, cos)` 轨道使用 Heydemann 风格椭圆校正
- Type-II 软件 PLL 输出滤波角度和角速度
- 输出层 dead-band / hysteresis：`0.015 deg` 角度门限和 `1` count 门限
- 加载有效 NVM 标定后，运行时监测幅值并做 AGC trim
- RAM-only 有符号多圈计数器，支持上位机命令清零
- FreeMASTER TSA 变量、Web 仪表板和桌面 `.pmpx` 工程
- 使用 DWT CYCCNT 统计 `encoder_process()` 和整个 ADC ISR 回调耗时

## 硬件

| 项目 | 值 |
| --- | --- |
| MCU 系列 | NXP MCXA34x / MCXA344 固件目标 |
| 当前 Keil target 说明 | `ind_encoder.uvprojx` 配置为 `MCXA343VLH` / `NXP.MCXA343_DFP.25.09.00`，源码 define 使用 `CPU_MCXA344VLL` 和 `MCXA344_SERIES` |
| 核心时钟 | Cortex-M33 + FPU，固件时钟配置为 180 MHz |
| 模拟前端 | MCXA OPAMP0/1 加外部 TLV9062 |
| ADC 采样 | LPADC0/1，4 个 raw 通道，10 kHz，CTIMER0 触发，8x 硬件平均 |
| 上位机链路 | FreeMASTER over LPUART0，115200 8N1，与 debug console 共用 |
| 持久化标定 | Flash 末 8 KB 区域，128-byte slot，按 sequence 选择最新有效块，CRC32 校验 |

### 信号接线

| 信号 | 引脚 |
| --- | --- |
| A1 SIN | OPAMP0_OUT -> P2_15 (ADC0_A2) |
| A1 COS | OPAMP1_OUT -> P2_19 (ADC1_A2) |
| A2 SIN | TLV9062 外部运放 -> P2_6 (ADC1_A3) |
| A2 COS | TLV9062 外部运放 -> P2_7 (ADC0_A7) |
| FreeMASTER UART | LPUART0 |
| Heartbeat LED | P3_11，100 ms 翻转一次 |
| ISR probe | P3_0，可选，用示波器观察 ISR 时序 |

## 仓库结构

```text
source/
  app_adc.{h,c}              CTIMER0 触发的 LPADC 采样
  app_encoder.{h,c}          无 OS/HAL 依赖的编码器核心算法
  app_encoder_runtime.{h,c}  ADC 回调、运行时 trim、命令、快照、性能统计
  app_encoder_storage.{h,c}  Flash 标定 slot 存储和 CRC32 校验
  app_encoder_defaults.c     板级默认标定值
  app_freemaster.{h,c}       FreeMASTER 初始化和 TSA 变量表
  hardware_init.{h,c}        OPAMP、引脚、时钟、调试 UART
  main.c                     上电入口和服务循环

freemaster/
  index.html                 Web 仪表板
  digital_encoder.pmpx       FreeMASTER 桌面工程
  simple-jsonrpc-js.js       JSON-RPC over WebSocket 辅助库

doc/
  HW_V2.jpg                  HW_V2 硬件照片
  AN15044.pdf                参考应用笔记
  MCXA34x_based_Inductive_Encoder_v1.1.docx
```

## 编译和烧录

可以直接打开 Keil 工程，也可以用命令行构建：

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

当前期望工具链：

- Keil MDK，ARMCLANG V6.23
- 工程文件当前引用 NXP MCXA343 DFP 25.09.00
- MCXA344 系列 SDK 头文件和源码已随仓库提交

工程输出名为 `ind_encoder`。scatter file 会为工厂标定存储保留 128 KB Flash 镜像末尾的 8 KB。

## 标定和使用

### 首次工厂标定

1. 接好 HW_V2 感应式编码器硬件并上电。
2. 打开 `freemaster/index.html` 或 `freemaster/digital_encoder.pmpx`。
3. FreeMASTER 连接 LPUART0，波特率 115200。
4. 点击 **Factory Cal**。
5. 在采集窗口内至少完整旋转一圈机械角。
6. 停在目标零位。最后一帧采样会作为工厂零位。

固件会采集 `8192` 个降采样后的标定样本，求解两条轨道的椭圆校正，捕获零位，
写入一个 128-byte 标定 slot，并在下次启动时选择 sequence 最大的有效块。

### 运行时命令

| 命令 | 作用 |
| --- | --- |
| **Zero Here** | 将当前角度作为 RAM 零位；复位后丢失 |
| **Reset Turns** | 清零 RAM-only 多圈计数器 |
| **Factory Cal** | 重新执行工厂标定并写入 Flash |
| **MCU Reset** | 触发 NVIC 系统复位 |

## FreeMASTER 仪表板

仪表板读取固件通过 TSA 暴露的变量：

- `encoder_result.angle_deg`、`angle_counts`、`multi_turn_deg` 和 `turn_count`
- `encoder_result.angular_velocity_dps`
- `encoder_result.mag16`、`mag15`、`mag16_raw` 和 `mag15_raw`
- `encoder_result.status`
- `adc_result` raw 通道、采样计数和 overrun 计数
- DWT 性能计数：`encoder_perf_process_cycles`、`encoder_perf_isr_cycles` 和峰值

状态位会由仪表板解码为 warning 和 fault 状态。

| 位 | 名称 | 含义 |
| --- | --- | --- |
| `0x0001` | `NOT_CALIBRATED` | 当前未加载有效标定 |
| `0x0002` | `TRACK16_WEAK` | 16 周期轨幅值低于下限 |
| `0x0004` | `TRACK15_WEAK` | 15 周期轨幅值低于下限 |
| `0x0008` | `ADC_RAIL` | 至少一个 ADC 通道撞导轨 |
| `0x0010` | `TRACK_MISMATCH` | 16/15 双轨残差超出容限 |
| `0x0020` | `CAL_FAILED` | 工厂标定解算失败 |
| `0x0040` | `HOLD_LAST` | 固件保持上一次有效角度 |
| `0x0080` | `CAL_STORAGE_INVALID` | NVM 标定 CRC 或块校验失败 |
| `0x0100` | `FACTORY_CAL_REQUIRED` | 当前使用默认值，需要工厂标定 |

## 配置

主要调参是编译期宏，位于 `source/app_encoder.h` 和 `source/app_adc.h`。

| 宏 | 默认值 | 作用 |
| --- | --- | --- |
| `ADC_SAMPLE_RATE_HZ` | `10000` | 实时 ADC 采样率；修改时要同步 CTIMER0 配置 |
| `ENCODER_CAL_SAMPLE_COUNT` | `8192` | 工厂标定样本数 |
| `ENCODER_TRACKING_BW_HZ` | `100.0f` | Type-II PLL 闭环带宽 |
| `ENCODER_TRACKING_ZETA` | `0.707f` | Type-II PLL 阻尼比 |
| `ENCODER_OUTPUT_DEADBAND_DEG` | `0.015f` | 发布角度 dead-band 门限 |
| `ENCODER_ANGLE_COUNT_HYSTERESIS` | `1` | 发布计数 hysteresis |
| `ENCODER_MAG_WINDOW_BINS` | `32` | 幅值显示窗口的转子角分箱数 |
| `ENCODER_RUNTIME_TRIM_GAIN_STEP_LIMIT` | `0.02f` | 单次 AGC gain trim 限幅 |
| `ENCODER_RUNTIME_TRIM_GAIN_TOTAL_LIMIT` | `0.5f` | AGC gain trim 总限幅 |

## 已知说明和限制

- 多圈位置是 RAM-only。掉电或复位会清零 turn counter。
- **Zero Here** 也是 RAM-only。持久化零位来自工厂标定。
- 低于输出 dead-band 速率的极慢速运动，发布角度路径可能表现为台阶。
- Runtime AGC 只在 NVM 标定作为当前标定源时启用。
- 当前 Keil 工程元数据使用 MCXA343 pack/device 名称，而固件源码目标是 MCXA344 系列；改 pack 或重新生成工程时要注意这一点。
- 当前仓库快照没有顶层 `LICENSE`、`CONTRIBUTING`、`SECURITY` 文件。

## 验证

本 README 描述当前固件接口和常量。文档-only 修改可用以下命令验证：

```powershell
git diff --check
```

固件构建仍通过 Keil MDK 完成：

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

## License

BSD-3-Clause，以源码文件中的 SPDX 头为准。
