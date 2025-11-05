# NXP MCXA344 Inductive Encoder — 软件架构（单页）

## 架构总览（分层）
- 应用层：`main.c`
- 服务层：`app_encoder`、`app_adc`、`app_sampler`、`app_tformat`、`app_timer`
- 硬件抽象层（SDK）：`fsl_*` 驱动（LPADC、OPAMP、CTIMER、LPUART、EDMA、CLOCK 等）

## 数据流路径（自传感到输出）
- 传感器→OPAMP0/1（差分调理）→`ADC0/ADC1`
- `CTIMER1` 周期中断（1 kHz）→`__SEV` 触发 ADC → 读取 FIFO
- `app_adc.read()` 返回 `adc_sample_result_t`
- `app_encoder.process()`：校准→归一化→`MAU atan2`→相位展开→滤波→量化
- 输出：
  - `app_tformat`（LPUART2+EDMA，T‑Format：ID0/ID3/IDD）
  - 调试串口（LPUART2）与 FreeMASTER（可选）

## 核心模块职责
- `app_adc`：双 ADC 设置与触发；OPAMP OUT 常规采样；温度（ADC0_A26，dual‑VBE 4点采样）；硬件平均（信号16×，温度128×）。
- `app_sampler`：`CTIMER1` 周期 ISR；触发采样、调用 `encoder_process`；提供线程安全拷贝接口。
- `app_encoder`：2×2矩阵校准、幅值门限、`MAU` 加速 `atan2`、电角→机角、Alpha‑Beta 轻滤波、方向/零位、16位量化；ABS/ABM/状态/告警。
- `app_tformat`：从设备响应（2.5 Mbps）；ID0/ID3/IDD；XOR CRC；与 `encoder_get_*` 对接。
- `app_timer`：`CTIMER0` 微秒级计时工具；性能测量辅助。
- `hardware_init`/`board`/`clock_config`：时钟、OPAMP 使能、测试脚、DWT 计数器、串口引脚。
- `mau_atan2`：`MAU_AtanXDivPIFloat` 封装成 `atan2`（硬件加速）。

## 时序与线程模型
- 采样节拍：默认 1 kHz（可配）；温度每 `TEMP_SAMPLE_INTERVAL`（10k 次）触发一次。
- ISR 仅做轻量工作：触发 ADC、读取 FIFO、调用 `encoder_process`、存储结果。
- 主线程：按需读取最新结果、数据流或协议响应；SysTick 用于简单交互超时。

## 关键技术边界
- ADC 与采样：`INPUTMUX` 路由 TXEV；`RESFIFO` 轮询读取；避免在 ISR 中阻塞。
- 校准与滤波：只处理与当前需求相关的 2×2 线性变换与轻滤波
- 协议：T‑Format 从设备最小实现（ID0/ID3/IDD）；CRC 采用 XOR（可切换）。
- 性能：`MAU` 加速三角函数；`DWT`/测试脚衡量 ISR 时长；平均/采样时间参数明确可控。

- 

---

- 