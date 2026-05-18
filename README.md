# NXP MCXA344 Inductive Encoder Demo

本工程当前用于验证 MCXA344 上的双码道电感式编码器。当前主流程基于 FreeMASTER：先确认 A1/A2 四路 OPAMP/ADC 数据，再做 RAM 在线校准，最后输出单圈绝对角度。

![](./img/3.png)

[TOC]

## 当前状态

- 硬件目标：双码道电感式编码器，主验证路径为 A1/A2 16/15 周期绝对角解算。
- 码道配置：当前固件默认 `A1 = 16` 周期、`A2 = 15` 周期，方向为 `dir16 = +1`、`dir15 = +1`。
- 角度算法：每个码道独立计算电角度，再用 16/15 Vernier/Nonius 相位差得到粗机械角，并用 16 周期码道做精细修正。
- 上位机：当前 FreeMASTER 入口为 `freemaster/index.html`；调试优先看 `encoder_result.*`、`encoder_diag.*` 和 `encoder_cal_flat`。
- 校准参数：启动时加载当前样机的板级默认校准值；FreeMASTER 在线校准只覆盖 RAM，复位后恢复固化默认值。

当前阶段不承诺 flash 参数保存、多圈累计、速度输出、温漂补偿或 T-Format 协议输出。

## 硬件与 ADC 映射

使用混合模拟前端：A1 使用 MCU 内部 OPAMP，A2 使用外置 `TLV9062`。

| 信号 | 前端 | MCU/ADC 通道 | FreeMASTER 字段 |
| --- | --- | --- | --- |
| A1 SIN | `OPAMP0_OUT` | `ADC0_A2` / P2_15 | `adc_result.a1_sin_raw` |
| A1 COS | `OPAMP1_OUT` | `ADC1_A2` / P2_19 | `adc_result.a1_cos_raw` |
| A2 SIN | `TLV9062 A2_OPA0_OUT` | `ADC1_A3` / P2_6 | `adc_result.a2_sin_raw` |
| A2 COS | `TLV9062 A2_OPA1_OUT` | `ADC0_A7` / P2_7 | `adc_result.a2_cos_raw` |

当前 TSA 只暴露 A1/A2 四路 raw ADC 字段和双码道解算结果作为验收口径。

## 角度算法

当前实现位于 `source/app_encoder.*`，主配置在 `source/app_encoder.h`：

```c
#define ENCODER_CONFIG_MAPPING ENCODER_MAPPING_A1_16_A2_15
#define ENCODER_CONFIG_DIR16   (1)
#define ENCODER_CONFIG_DIR15   (1)
```

算法流程：

1. 采集 A1/A2 两组 SIN/COS raw ADC。
2. 在线校准采集 8192 组样本，分别计算 A1/A2 的中心点和 2x2 椭圆校正矩阵。
3. 对每个码道执行去中心、幅值归一和 `atan2(SIN, COS)`，得到 A1/A2 电角度。
4. 按配置把 A1/A2 映射成 16/15 周期码道。
5. 计算 `p16` 与 `p15` 的零点修正和方向修正。
6. 先得到粗机械角：

```text
coarse = wrap(p16 - p15)
```

因为 16 周期与 15 周期相差 1，理想情况下 `p16 - p15` 对应一圈机械绝对角。当前主输出会继续用 16 周期码道做精细修正：

```text
angle = wrap(coarse + error(p16, coarse * 16) / 16)
```

一致性检查会比较 `angle * 16` 与 16 周期电角、`angle * 15` 与 15 周期电角。如果幅值过低、ADC 贴边、未校准或双码道相位不一致，`encoder_result.status` 会置位，主角度保持上一次有效值。

## FreeMASTER 使用流程

连接参数：

- UART：`LPUART0`
- 波特率：`115200`
- 格式：`8N1`

推荐 bring-up 顺序：

1. 连接 FreeMASTER，打开 `freemaster/index.html` 控制页。
2. 先观察四路 raw：
   - `adc_result.a1_sin_raw`
   - `adc_result.a1_cos_raw`
   - `adc_result.a2_sin_raw`
   - `adc_result.a2_cos_raw`
3. 缓慢转动转子，确认四路都有周期变化，且没有长时间贴近 0 或 65535。
4. 固件启动后会自动加载板级默认校准值，可直接观察：
   - `encoder_result.angle_deg`
   - `encoder_result.angle_counts`
   - `encoder_result.status`
5. 如果更换线圈、转子或模拟前端，需要重新标定时，再写 `fm_cal_enable = 1`，或点击界面里的 Online Calibration。
6. 在 `fm_cal_progress` 从 0 到 100 期间，慢慢转动转子至少一整圈。
7. `fm_cal_done = 1` 后，重新观察 `encoder_result.*` 和 `encoder_cal_flat`。
8. 如果需要重新设置当前机械零点，校准完成后写 `fm_zero_ctrl = 1`。

在线校准结束时固件会自动捕获当前点作为临时零点；后续也可以通过 `fm_zero_ctrl` 重新捕获零点。在线校准结果不写 flash，复位后会回到程序内固化的板级默认值。

## 关键 FreeMASTER 变量

ADC raw：

- `adc_result.a1_sin_raw`
- `adc_result.a1_cos_raw`
- `adc_result.a2_sin_raw`
- `adc_result.a2_cos_raw`

调试输出：

- `encoder_result.angle_deg`
- `encoder_result.angle_counts`
- `encoder_result.phase16_deg`
- `encoder_result.phase15_deg`
- `encoder_result.coarse_deg`
- `encoder_result.mag16`
- `encoder_result.mag15`
- `encoder_result.status`
- `encoder_diag.mapping`
- `encoder_diag.dir16`
- `encoder_diag.dir15`
- `encoder_diag.phase16_error_deg`
- `encoder_diag.phase15_error_deg`
- `encoder_diag.phase_a1_deg`
- `encoder_diag.phase_a2_deg`

校准参数平铺视图：

- `encoder_cal_flat[0..5]`：A1 `center_sin`、`center_cos`、`T[0,0]`、`T[0,1]`、`T[1,0]`、`T[1,1]`
- `encoder_cal_flat[6..11]`：A2 `center_sin`、`center_cos`、`T[0,0]`、`T[0,1]`、`T[1,0]`、`T[1,1]`
- `encoder_cal_flat[12]`：`phase_a1_zero_deg`
- `encoder_cal_flat[13]`：`phase_a2_zero_deg`

当前固化默认值：

- A1：`center_sin = 12156.5`，`center_cos = 22284.5`，`T = [[0.000151, 0], [0, 0.000079]]`，`zero = 205.701°`
- A2：`center_sin = 21796.5`，`center_cos = 21121.5`，`T = [[0.000091, 0], [0, 0.000091]]`，`zero = 118.153°`

校准控制：

- `fm_cal_enable`：写 1 开始 RAM 在线校准。
- `fm_cal_done`：1 表示本次校准成功。
- `fm_cal_progress`：0..100。
- `fm_cal_state`：0 idle，1 running，2 done，3 failed。
- `fm_cal_status`：校准失败或状态码。
- `fm_zero_ctrl`：写 1 捕获当前机械零点。
- `fm_reset_ctrl`：写 1 软件复位。

速度、多圈累计和方向控制变量当前不作为本阶段验收目标。

## 状态码速查

`encoder_result.status` 和 `fm_encoder_status` 使用位标志：

| Bit | 含义 |
| --- | --- |
| `0x01` | 未校准 |
| `0x02` | 16 周期码道幅值过低 |
| `0x04` | 15 周期码道幅值过低 |
| `0x08` | ADC 贴边 |
| `0x10` | 双码道相位不一致 |
| `0x20` | 校准失败 |
| `0x40` | 保持上一次有效角度 |

正常输出时状态应为 `0x00`。

## 构建与验证

Keil 构建：

```powershell
C:\Keil_v5\UV4\UV4.exe -b ind_encoder.uvprojx -t ind_encoder
```

Host 算法测试：

```powershell
gcc -std=c99 -Wall -Wextra -Isource tests/test_app_encoder.c source/app_encoder.c source/app_encoder_defaults.c -lm -o debug/test_app_encoder.exe
debug/test_app_encoder.exe
```

文档/补丁检查：

```powershell
git diff --check -- README.md
```

## 当前限制

- 在线校准结果不写 flash，复位后恢复 `encoder_calibration_set_board_defaults()` 中的固化默认值。
- 当前只输出单圈绝对角，不做断电保持的多圈计数。
- 当前速度、转数和方向控制变量没有接入主算法。
- 在线校准模型为中心点加 2x2 椭圆校正，不包含谐波补偿。
- 若后续仍有规律性非线性误差，应继续从机械同心度、码道相位关系和谐波补偿入手。
