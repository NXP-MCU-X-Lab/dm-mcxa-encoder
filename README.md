# NXP MCXA344 双轨感应式编码器

> [English](README.en.md) · 中文

基于 MCXA344 (Cortex-M33 @ 180 MHz) 的双轨感应式绝对编码器参考设计:
**16/15 Vernier 单圈绝对角 + 多圈累计 + Type-II 软件 PLL**,
配套 FreeMASTER 实时监控仪表板。算法层无 OS 依赖,可独立单元测试。

---

## 规格

| 项目 | 值 |
| --- | --- |
| MCU | NXP MCXA344 · Cortex-M33 + FPU · 180 MHz |
| 模拟前端 | OPAMP0/1 + 外部 TLV9062 · LPADC0/1 双 ADC 同步采样 |
| 采样率 | 10 kHz · 4 通道 · 硬件平均 8× · CTIMER0 触发 |
| 角度分辨率 | 16-bit / 转 · 1 LSB ≈ 0.0055° |
| 静态噪声 | ≤ 0.015° (输出层 hysteresis · AS5048 风格 · 2.7 LSB) |
| 跟踪带宽 | 100 Hz · ζ = 0.707 (Type-II PLL,可调) |
| 多圈范围 | INT32 · ±2³¹ 转 · RAM-only |
| 计算占用 | DWT 实测,典型 < 10 % CPU @ 180 MHz / 10 kHz 采样 |
| 持久化标定 | Flash 末 8 KB · CRC32 + 双块冗余 |
| 上位机 | FreeMASTER · LPUART0 @ 115200 8N1 · TSA 自描述 |

---

## 原理

### 信号链

```
LPADC0/1  ─┐
  10 kHz  │ 4 通道同步采样,硬件平均 8×
  CTIMER0 │
触发     ─┘
   │
   ▼
椭圆校正           Heydemann:min/max 取中心 + LSQ 拟椭圆 + Cholesky 解 T
   │
   ▼
atan2  →  φ16 (16 周期轨), φ15 (15 周期轨)
   │
   ▼
Vernier 解算
   coarse = wrap(φ16 − φ15)               全圈过零一次 — 粗角
   angle  = coarse + (φ16 − 16·coarse)/16 高分辨率精修
   │
   ▼
Type-II 跟踪观测器                       BW = 100 Hz, ζ = 0.707
   │  → angular_velocity_dps
   ▼
多圈累计 (±1 跨 0°/360° 边界)
   │
   ▼
输出层 hysteresis (2.7 LSB ≈ 0.015°)
   │
   ▼
encoder_result { angle_deg, angle_counts, multi_turn_deg,
                 angular_velocity_dps, mag16/15, turn_count, status }
```

### Vernier 16/15

两组感应轨道在一圈内分别有 16 和 15 个电气周期。
相位差 `φ16 − φ15` 在一整圈内**恰好变化 360°**,这就是全圈唯一的"粗"角;
再用高分辨率 `φ16` 精修,得到全分辨率绝对角。
游标卡尺(Vernier)原理在角度编码上的经典做法,工业 RDC(AD2S1210)、
磁编码器(iC-MU)、Renishaw 等均采用类似的双轨高低分辨率合成。

### Heydemann 椭圆校正

raw (sin, cos) 因幅值不等、零位偏移、非正交三种偏差,落在一个椭圆上而非单位圆。
两步法校正:

1. **中心**:取 min/max 中点。对采样密度的不均匀鲁棒。
2. **形状**:对中心化样本做代数 LSQ 拟合 `a·x² + b·xy + c·y² = 1`,
   由 `Tᵀ·T = M` 解 Cholesky 下三角 `T`。

`(corr_sin, corr_cos) = T · ((raw_sin, raw_cos) − center)` 后任意转角的
`‖corr‖ ≈ 1.0`,`atan2` 直接给真实相位。

### Type-II 跟踪观测器(软件 PLL)

经典 resolver-to-digital 输出级。闭环特征方程 `s² + Kp·s + Ki`,
`Kp = 2ζωₙ`、`Ki = ωₙ²`。

- 对匀速旋转**零稳态相位滞后**
- 同时输出**滤波角度**和**角速度**(免去后差分)
- 带宽 `ENCODER_TRACKING_BW_HZ` 可调,默认 100 Hz

### 静态抖动抑制

输出层 hysteresis(`ENCODER_OUTPUT_DEADBAND_DEG = 0.015°`,约 2.7 LSB):
新角度与上一次发布相差 < 门限时,保持上一次值。

- AS5048 / iC-MU 等量产编码器的业界做法 —— **不动 PLL 数学,只在发布层加 dead-band**
- 代价:低于 `(threshold / dt)` ≈ 150 °/s 的极慢速被报为静止
- 数字侧另设 `ENCODER_ANGLE_COUNT_HYSTERESIS = 1` LSB 防止 `angle_counts` ±1 抖动

### 分支滑移保护

Vernier 单帧跳变超过半个精细周期时,在 ±8 分支内搜索最匹配点。
预测角用 `last_angle_raw + velocity·dt` —— 取 Vernier raw 而非 PLL 滤波角,
避免跟踪观测器的滞后污染离散分支判别。

### 多圈累计

每帧若发布角发生 |Δ| > 180° 的跨边界跳变,`turn_count ±= 1`;
INT32 饱和保护,RAM-only(掉电丢失,可命令复位为零)。

### 运行时 AGC

NVM 标定加载后,每完整旋转一周观测原始磁矢量幅值均值,以 EMA 方式
调整等效 T 矩阵增益,补偿温度 / OPAMP / 电源带来的幅值漂移。
步长与总量都有 clamp,异常 / 撞导轨样本自动跳过。

---

## 架构

```
source/
  app_adc.{h,c}              CTIMER0 触发的 LPADC 4 通道采样(8× 硬件平均)
  app_encoder.{h,c}          核心算法 — 无 OS / 无硬件依赖,可单元测试
  app_encoder_runtime.{h,c}  ADC 回调、AGC trim、快照发布、命令服务、DWT 性能监测
  app_encoder_storage.{h,c}  标定 Flash 存储(末 8 KB · CRC32 · 双块)
  app_encoder_defaults.c     上电默认标定参数
  app_freemaster.{h,c}       FreeMASTER 初始化与 TSA 变量表
  hardware_init.{h,c}        OPAMP / PIN / 时钟 / 调试 UART
  clock_config.{h,c}         FRO 180 MHz 时钟配置
  main.c                     上电入口与主循环 (WFI 节能)

freemaster/
  index.html                 Web 仪表板 (Monitor + Diagnostics 两 tab)
  digital_encoder.pmpx       FreeMASTER 桌面客户端工程
  simple-jsonrpc-js.js       JSON-RPC over WebSocket(PCM 协议)
```

**数据流**:ADC 中断中调用 `encoder_process` 解算 → 写 volatile 快照
→ 主循环 `EncoderApp_Service` 关中断拷出来发布到 `encoder_result`
→ FreeMASTER TSA 表暴露给上位机 / `index.html` 读取。

---

## 性能监测

ISR 内用 Cortex-M33 **DWT CYCCNT**(1 周期精度)夹测 `encoder_process` 与整个
ADC 回调耗时,峰值持续更新。诊断页 0.4 Hz 低频读取,换算微秒和 CPU 占比。

| TSA 字段 | 含义 |
| --- | --- |
| `encoder_perf_process_cycles` | 最近一帧 `encoder_process` 周期数 |
| `encoder_perf_process_max` | 启动以来峰值 |
| `encoder_perf_isr_cycles` | 最近一帧整个 ADC ISR 回调周期数 |
| `encoder_perf_isr_max` | 启动以来峰值 |
| `encoder_perf_core_clock_hz` | CPU 频率,供上位机换算 µs |

CPU 占比 = (ISR µs) / 100 µs × 100 %(10 kHz 采样 → 每帧 100 µs 预算)。

---

## 上手

### 硬件接线

| 信号 | 引脚 |
| --- | --- |
| A1 SIN | OPAMP0_OUT → P2_15 (ADC0_A2) |
| A1 COS | OPAMP1_OUT → P2_19 (ADC1_A2) |
| A2 SIN | TLV9062 外部运放 → P2_6  (ADC1_A3) |
| A2 COS | TLV9062 外部运放 → P2_7  (ADC0_A7) |
| FreeMASTER UART | LPUART0 (与 debug console 共用) |
| Heartbeat LED | P3_11 (100 ms 翻转,确认 firmware 运行) |
| ISR Probe | P3_0 (可选,示波器看 ADC ISR 时序) |

### 编译烧录

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

工具链:**Keil MDK + ARMCLANG V6.23**。烧写到 MCXA344-EVK。

### 首次工厂标定

1. 接好感应轨道硬件并上电
2. 打开 `freemaster/index.html`(或 `freemaster/digital_encoder.pmpx`)连 LPUART0 @ 115200
3. Monitor 页 → **Factory Cal** 启动
4. 在约 8 秒采集窗口内**完整旋转一圈以上**(尽量匀速)
5. 采集结束时停在目标零位 — 最后一帧自动作为工厂零位
6. 标定数据写入 Flash 末 8 KB 扇区(掉电不丢,CRC32 校验)

标定失败会落回上电默认参数,主页 Alert Ribbon 提示 `CAL_FAILED` / `FACTORY_CAL_REQUIRED`。

### 日常使用

Monitor 页实时显示:

- 绝对角度盘 (针 + 0/90/180/270 刻度)
- 多圈累计角 + 圈数
- 速度仪表(°/s + RPM + 方向条)
- 顶部 Alert Ribbon — 绿/琥珀/红 三态,自动列出有效故障位

Diagnostics 页:

- 状态字 + 标定来源(NVM / DEFAULT / FACTORY_PENDING)
- mag16 / mag15 (mean / raw)
- ADC 采样计数 + overrun 计数
- **Compute Budget**:`encoder_process` 与 ISR 总耗时(µs + cycles + 峰值 + CPU%)

| 按钮 | 作用 |
| --- | --- |
| **Zero Here** | 当前角度记为 RAM 零位(掉电丢失) |
| **Reset Turns** | 多圈计数器清零 |
| **Factory Cal** | 重新工厂标定并写 Flash |
| **MCU Reset** | NVIC 系统复位 |

---

## 状态位

`encoder_result.status` 是一个位掩码,Monitor 页 Alert Ribbon 自动解码。
带 ★ 的位归类为 hard fault(红),其余为 warning(琥珀)。

| 位 | 名称 | 含义 |
| --- | --- | --- |
| `0x0001` | `NOT_CALIBRATED` | 当前未加载有效标定 |
| `0x0002` ★ | `TRACK16_WEAK` | 16 周期轨幅值低于下限 |
| `0x0004` ★ | `TRACK15_WEAK` | 15 周期轨幅值低于下限 |
| `0x0008` ★ | `ADC_RAIL` | ADC 通道撞导轨(0 或满量程) |
| `0x0010` | `TRACK_MISMATCH` | 16/15 双轨残差超出容限 |
| `0x0020` ★ | `CAL_FAILED` | 工厂标定解算失败 |
| `0x0040` | `HOLD_LAST` | 保持上一次有效角度 |
| `0x0080` ★ | `CAL_STORAGE_INVALID` | NVM 块 CRC 校验失败 |
| `0x0100` | `FACTORY_CAL_REQUIRED` | 启动回落到默认值,需要工厂标定 |

---

## 关键调参

源码中以下宏覆盖绝大多数场景,无需重新设计算法即可调:

| 宏 | 默认 | 说明 |
| --- | --- | --- |
| `ADC_SAMPLE_RATE_HZ` | 10000 | ADC 采样率,改时需同步 CTIMER 配置 |
| `ENCODER_TRACKING_BW_HZ` | 100.0 | PLL 闭环带宽,高速应用可上调 |
| `ENCODER_TRACKING_ZETA` | 0.707 | 阻尼比,Butterworth 默认 |
| `ENCODER_OUTPUT_DEADBAND_DEG` | 0.015 | 输出 hysteresis(° 越大越静) |
| `ENCODER_ANGLE_COUNT_HYSTERESIS` | 1 | counts 域 hysteresis(LSB) |
| `ENCODER_MAG_WINDOW_BINS` | 32 | mag 显示均值滑窗的角度分箱 |

---

## License

BSD-3-Clause (见源文件头部)。
