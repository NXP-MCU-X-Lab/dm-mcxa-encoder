# NXP MCXA344 双轨感应式绝对编码器

> [English](README.md) | 中文

这是面向 HW_V2 双轨感应式绝对编码器的固件，目标器件为 NXP MCXA344。固件
以 10 kHz 采集 16/15 Vernier 双轨信号，并通过 2.5 Mbit/s T-Format 接口
输出原生 16-bit 单圈位置。

![HW_V2 感应式编码器硬件](doc/HW_V2.jpg)

## 功能

- 16/15 Vernier 单圈绝对角解算
- 原生 16-bit 位置输出，每圈 `0..65535` count
- 每条轨道的偏置、增益、正交和椭圆误差校正
- 运行时中心与增益自适应，并自动持久化
- 分支校验、最后有效位置保持和受控重新锁定
- LPUART2/eDMA 实现标准 T-Format ID0/1/2/3/6/7/8/C/D
- 持久化单圈零位和 128-byte 编码器 EEPROM 镜像
- LPUART0 简化 FreeMASTER 状态与服务页面
- DWT 执行时间统计和双 ADC 样本配对诊断

## 硬件

| 项目 | 配置 |
| --- | --- |
| MCU | NXP MCXA344VLL，Cortex-M33 + FPU，180 MHz |
| 模拟前端 | MCXA OPAMP0/1 加外部 TLV9062 |
| 采样 | LPADC0/1，四通道，CTIMER0 触发，10 kHz |
| 持久化存储 | 两个 8 KB Flash 扇区，256-byte 快照，sequence 与 CRC32 |
| FreeMASTER | LPUART0，115200 baud，8N1 |
| T-Format | LPUART2，2,500,000 baud，8N1 |

### 信号接线

| 信号 | MCU 连接 |
| --- | --- |
| A1 SIN | OPAMP0_OUT -> P2_15 (ADC0_A2) |
| A1 COS | OPAMP1_OUT -> P2_19 (ADC1_A2) |
| A2 SIN | TLV9062 输出 -> P2_6 (ADC1_A3) |
| A2 COS | TLV9062 输出 -> P2_7 (ADC0_A7) |
| T-Format TX | P2_2，LPUART2_TXD |
| T-Format RX | P2_3，LPUART2_RXD |
| RS-485 方向 | P3_12，`DIR_485`；低电平接收，高电平发送 |
| 主循环心跳 | P3_11 |
| ADC ISR 时序探针 | P3_0 |

## 仓库结构

```text
source/
  app_adc.{h,c}              双 LPADC 同步采样
  app_encoder.{h,c}          与硬件无关的编码器算法
  app_encoder_runtime.{h,c}  运行时自适应、命令与持久化
  app_encoder_storage.{h,c}  双扇区快照与 CRC32
  app_encoder_defaults.c     板级默认标定
  app_tformat.{h,c}          LPUART2/eDMA T-Format responder
  app_freemaster.{h,c}       FreeMASTER 初始化与 TSA 表
  hardware_init.{h,c}        时钟、OPAMP、引脚与辅助 GPIO

tools/
  tformat_test.py            标准 T-Format Python 主站

tests/
  encoder_sim.c              算法与动态工况仿真
  tformat_sim.c              协议向量与状态机测试
  storage_sim.c              Flash 快照与掉电注入测试
  stubs/                     最小 MCU 头文件，使 app_tformat.c 可在主机编译

freemaster/
  index.html                 简化操作与服务页面
  digital_encoder.pmpx       FreeMASTER 桌面工程
```

## 编译和烧录

安装 Keil MDK、ARMCLANG 6.23 和 `NXP.MCXA344_DFP.25.06.00`。在 Keil 中
打开 `ind_encoder.uvprojx` 并构建 target `ind_encoder`，或运行：

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

工程目标为 `MCXA344VLL`，下载算法为 `MCXA34X_256.FLM`。应用镜像范围限制为
`0x00000000..0x0003BFFF`，`0x0003C000..0x0003FFFF` 保留给编码器持久化数据。

## T-Format 接口

协议响应器采用 [NXP T-Format 公共定义](https://github.com/nxp-mcuxpresso/mcuxsdk-core/blob/release/26.03.00-pvw2/drivers/flexio/t-format/fsl_flexio_t-format.h)
以及 [TI T-Format 参考设计](https://www.ti.com/lit/ug/tidue74f/tidue74f.pdf)中的字段布局。

| ID | CF | 功能 | 响应长度 |
| --- | --- | --- | --- |
| ID0 | `0x02` | 单圈位置 | 6 bytes |
| ID1 | `0x8A` | 多圈字段，固定为 0 | 6 bytes |
| ID2 | `0x92` | 编码器 ID，`ENID=0x10` | 4 bytes |
| ID3 | `0x1A` | 位置、ENID、ABM 和 ALMC | 11 bytes |
| ID6 | `0x32` | EEPROM 写入 | 4 bytes |
| ID7 | `0xBA` | 错误复位 | 6 bytes |
| ID8 | `0xC2` | 设置并保存单圈零位 | 6 bytes |
| IDC | `0x62` | 多圈与错误复位；ABM 保持为 0 | 6 bytes |
| IDD | `0xEA` | EEPROM 读取 | 4 bytes |

上表每个 CF 字节都是推导出来的，不是抄的：`sink code 2 | (ID << 3) |
(ID 位的奇校验 << 7)`。帧长度、ALMC 位图、SF 编码器错误字段、ADF 地址/busy
掩码以及 CRC 多项式，均与上面链接的 NXP 定义一致。唯一没有规范来源的值是
`ENID` —— `0x10` 只是占位值，必须与驱动器约定，因为驱动器据此推断分辨率。

串口格式为 `2,500,000 baud, 8N1, idle high, LSB first`。`ABS0/ABS1`
承载原生 16-bit 计数，`ABS2=0`，多圈字段始终为 0。CRC 多项式为
`x^8 + 1`，初值为 0。位置无效或过期时保持最后有效值，并通过 SF 和 ALMC
报告 Counting Error。

EEPROM 写入后 ADF busy 位保持有效，直到 128-byte 镜像写入 Flash。配置写入
只在转轴静止时执行。不支持的控制字段会被忽略，协议串口不会发送主动文本。

### Python 主站

安装 pyserial，并将 `COM78` 替换为实际串口：

```powershell
python -m pip install pyserial
python tools\tformat_test.py --self-test
python tools\tformat_test.py --port COM78 --data-id 0 --count 1000 --verbose
python tools\tformat_test.py --port COM78 --data-id 3 --count 1000
python tools\tformat_test.py --port COM78 --reset position
python tools\tformat_test.py --port COM78 --eeprom-write 0x20 0xA5
python tools\tformat_test.py --port COM78 --eeprom-read 0x20
```

## 标定和操作

启动时加载 Flash 中 sequence 最新且 CRC32 有效的快照。Flash 为空时，编码器
先使用板级默认参数，在具备足够旋转覆盖后自动跟踪双轨中心和增益，并在转轴
静止后保存收敛参数。首次自适应锁定前，T-Format 返回 Counting Error。

`Zero & Save` 和 T-Format ID8 要求位置在 0.5 秒内保持于 16 counts 范围。
固件对 64 个样本求平均，设置单圈零位，并在 Flash 校验成功后确认命令完成。

完整椭圆和正交标定保留在 FreeMASTER 的 Service 区域。该功能在转轴完成整周
旋转期间采集 8192 个降采样样本，随后保存求解后的标定参数和零位。

## FreeMASTER

打开 `freemaster/digital_encoder.pmpx`，通过 LPUART0、115200 baud 连接并
打开内嵌网页。主界面仅保留就绪状态、角度、转速、状态和 `Zero & Save`。
折叠的 Service 区域包含原始 ADC、运行计数、T-Format 计数、工厂标定和配置
擦除。页面约以 4 Hz 轮询主状态，不使用 Scope 或 Recorder。

## 配置

| 宏 | 值 | 作用 |
| --- | --- | --- |
| `ADC_SAMPLE_RATE_HZ` | `10000` | 编码器更新率 |
| `ENCODER_CAL_SAMPLE_COUNT` | `8192` | 服务标定样本数 |
| `ENCODER_TRACKING_BW_HZ` | `100.0f` | 观测器带宽 |
| `ENCODER_TRACKING_ZETA` | `0.707f` | 观测器阻尼比 |
| `ENCODER_FILTER_HOLD_RESYNC_SAMPLES` | `50` | 连续无效样本后的重新锁定间隔 |
| `ENCODER_OUTPUT_DEADBAND_DEG` | `0.015f` | 监控角度 dead-band |
| `ENCODER_ANGLE_COUNT_HYSTERESIS` | `3` | 低延迟计数 hysteresis |

## 使用约束

- 产品接口为单圈，ABM 和所有多圈字段固定为 0。
- Flash 擦写期间会暂时停止采样和 T-Format 响应。
- EEPROM 和零位写入在转轴静止并完成新快照校验后结束 busy 状态。
- LPUART0 仅供 FreeMASTER 使用；产品 target 不包含 DebugConsole 和普通串口日志。

## 许可证

项目应用层代码采用 BSD-3-Clause 许可证，见 [LICENSE](LICENSE)。仓库内 SDK、
CMSIS 和 FreeMASTER 组件保留各自的版权与许可证声明。
