# NXP MCXA344 感应式编码器

双轨感应式编码器示例:16/15 Vernier 单圈绝对角 + 多圈累计,FreeMASTER 实时监控。

---

## 原理

### 信号链

```
ADC × 4  (10 kHz, hw avg 8×)
   │
   ▼
椭圆校正           Heydemann:min/max 取中心,LSQ 拟椭圆,Cholesky 解 T 矩阵
   │
   ▼
atan2  →  phase_a1 (16 周期), phase_a2 (15 周期)
   │
   ▼
Vernier 解算
   coarse = wrap(p16 - p15)              ← 全圈过零一次,粗角
   angle  = coarse + (p16 - 16·coarse)/16 ← p16 精修高分辨率
   │
   ▼
Type-II PLL  (BW=100 Hz, ζ=0.707)        ← 滤波角度 + 角速度
   │
   ▼
多圈累计  (跨 0/360° ±1)
   │
   ▼
输出: angle_deg, multi_turn_deg, angular_velocity_dps
```

### Vernier 16/15

两组感应轨道在一圈内分别有 16 和 15 个电气周期。相位差 `p16 − p15` 在一整圈内
**恰好变化 360°**,这就是全圈唯一的"粗"角;再用高分辨率 `p16` 精修,得到全分辨率
绝对角。游标卡尺(Vernier)原理用到角度编码上的经典做法。

### 椭圆校正(Heydemann)

raw (sin, cos) 因幅值不等、零位偏移、非正交三种偏差,落在一个椭圆上而非单位圆。
校正公式:

```
(corr_sin, corr_cos) = T · ((raw_sin, raw_cos) - center)
```

其中 `T` 是 Cholesky 下三角矩阵,满足 `Tᵀ·T = M`,而 `M` 来自代数最小二乘拟合
`a·x² + b·xy + c·y² = 1`。校正后任意转角的 `‖corr‖ ≈ 1.0`,atan2 直接给出真实相位。

### Type-II 跟踪观测器(软件 PLL)

经典 resolver-to-digital 输出级。闭环传递函数 `s² + Kp·s + Ki`,
`Kp = 2ζωₙ`、`Ki = ωₙ²`。对匀速旋转零稳态相位滞后,同时输出**滤波角度**和**角速度**。
带宽 `ENCODER_TRACKING_BW_HZ` 可调。

**输出层 hysteresis**(`ENCODER_OUTPUT_DEADBAND_DEG ≈ 0.015°` ≈ 2.7 LSB):新角度跟
上一次发布差异 < 门限时,保持上一次值。这是 AS5048 / iC-MU 等磁/感应编码器的业界
标准做法 —— 不动 PLL 数学,只在发布层加 dead-band。代价是低于 `(threshold / dt)`
的极慢速被视作静止。

分支滑移保护使用 `last_angle_raw + velocity·dt` 做预测 — 不取 PLL 滤波后的角度,
避免离散分支判别被跟踪观测器的滞后污染。

---

## 架构

```
source/
  app_adc.{h,c}              CTIMER0 触发的 LPADC 4 通道实时采样(8× 硬件平均)
  app_encoder.{h,c}          编码器算法(无 OS 依赖,可单元测试)
  app_encoder_runtime.{h,c}  ADC 中断回调、快照发布、命令服务、状态机
  app_encoder_storage.{h,c}  工厂标定 Flash 存储(末 8KB 扇区,CRC + 双块)
  app_encoder_defaults.c     上电默认标定参数
  app_freemaster.{h,c}       FreeMASTER TSA 变量表、命令变量
  hardware_init.c            OPAMP / PIN / 时钟
  main.c                     上电流程入口

freemaster/
  index.html                 Web 监控仪表板(主页 + 诊断页)
  digital_encoder.pmpx       FreeMASTER 桌面客户端工程文件
```

**数据流**:ADC 中断中调用 `encoder_process` 解算角度,写入 `s_realtime_*` volatile 快照;
主循环 `EncoderApp_Service` 关中断拷出来发布到 `encoder_result`,经 FreeMASTER
TSA 表暴露给上位机读取。

---

## 上手

### 硬件接线

| 信号 | 引脚 |
| --- | --- |
| A1 SIN | OPAMP0_OUT → P2_15 (ADC0_A2) |
| A1 COS | OPAMP1_OUT → P2_19 (ADC1_A2) |
| A2 SIN | TLV9062 外部运放 → P2_6 (ADC1_A3) |
| A2 COS | TLV9062 外部运放 → P2_7 (ADC0_A7) |
| FreeMASTER UART | LPUART0 |

### 编译烧录

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

用 Keil 烧到 MCXA344 EVK。

### 首次工厂标定

1. 接好感应轨道硬件并上电
2. 打开 `freemaster/digital_encoder.pmpx`(或 `freemaster/index.html`)连 LPUART0
3. 主页 → **Factory Cal** 启动
4. 在约 8 秒采集窗口内**完整旋转一圈以上**
5. 采集结束时停在目标零位 — 最后一帧自动作为工厂零位
6. 标定数据写入 Flash 末 8 KB 扇区(掉电不丢)

### 日常使用

主页实时显示:绝对角度盘 + 多圈计数 + 速度 + 故障告警条。

| 按钮 | 作用 |
| --- | --- |
| **Zero Here** | 当前角度记为 RAM 零位(掉电丢失) |
| **Reset Turns** | 多圈计数器清零 |
| **Factory Cal** | 重新工厂标定并存 Flash |
| **MCU Reset** | NVIC 系统复位 |

故障时主页顶部 Alert Ribbon 自动变 amber / red,列出具体故障名(`LOS_TRACK16`、
`DOS`、`LOT` 等)。

---

## 状态位

| 位 | 名称 | 含义 |
| --- | --- | --- |
| `0x0001` | `NOT_CALIBRATED` | 未加载有效标定 |
| `0x0002` | `TRACK16_WEAK` | 16 周期轨幅值低于下限 |
| `0x0004` | `TRACK15_WEAK` | 15 周期轨幅值低于下限 |
| `0x0008` | `ADC_RAIL` | ADC 通道撞导轨 |
| `0x0010` | `TRACK_MISMATCH` | 16/15 双轨残差超容限 |
| `0x0020` | `CAL_FAILED` | 标定解算失败 |
| `0x0040` | `HOLD_LAST` | 保持上一次有效角度 |
| `0x0080` | `CAL_STORAGE_INVALID` | NVM 块无效 |
| `0x0100` | `FACTORY_CAL_REQUIRED` | 启动回落到默认值 |
