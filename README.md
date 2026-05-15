# NXP MCXA344 V2 Inductive Encoder Demo

本工程当前用于验证 MCXA344 上的 V2 双码道电感式编码器。当前主流程基于 FreeMASTER：先确认 A1/A2 四路 OPAMP/ADC 数据，再做 RAM 在线校准，最后输出单圈绝对角度。

![](./img/3.png)

[TOC]

## 当前状态

- 硬件目标：V2 双码道编码器，不再按 V1 单码道算法作为主验证路径。
- 码道配置：当前固件默认 `A1 = 15` 周期、`A2 = 16` 周期，方向为 `dir16 = +1`、`dir15 = +1`。
- 角度算法：每个码道独立计算电角度，再用 16/15 Vernier/Nonius 相位差得到单圈机械角。
- 上位机：沿用 V1 FreeMASTER 工程，`encoder_result.angle_deg/counts` 作为兼容输出；V2 调试优先看 `v2_result.*` 和 `v2_diag.*`。
- 校准参数：只保存在 RAM，复位后需要重新校准。

当前阶段不承诺 flash 参数保存、多圈累计、速度输出、温漂补偿或 T-Format 协议输出。

## 硬件与 ADC 映射

V2 使用混合模拟前端：A1 使用 MCU 内部 OPAMP，A2 使用外置 `TLV9062`。

| 信号 | 前端 | MCU/ADC 通道 | FreeMASTER 字段 |
| --- | --- | --- | --- |
| A1 SIN | `OPAMP0_OUT` | `ADC0_A2` / P2_15 | `adc_result.a1_sin_raw` |
| A1 COS | `OPAMP1_OUT` | `ADC1_A2` / P2_19 | `adc_result.a1_cos_raw` |
| A2 SIN | `TLV9062 A2_OPA0_OUT` | `ADC1_A3` / P2_6 | `adc_result.a2_sin_raw` |
| A2 COS | `TLV9062 A2_OPA1_OUT` | `ADC0_A7` / P2_7 | `adc_result.a2_cos_raw` |

兼容旧 FreeMASTER Watch，`adc_result.opamp0_out/opamp1_out` 仍保留为 A1 SIN/COS 的别名，只用于兼容，不代表 V2 全部四路输入。

## V2 角度算法

当前实现位于 `source/app_encoder_v2.*`，主配置在 `source/app_encoder_v2.h`：

```c
#define V2_ENCODER_CONFIG_MAPPING V2_ENCODER_MAPPING_A1_15_A2_16
#define V2_ENCODER_CONFIG_DIR16   (1)
#define V2_ENCODER_CONFIG_DIR15   (1)
```

算法流程：

1. 采集 A1/A2 两组 SIN/COS raw ADC。
2. 在线校准采集 8192 组样本，分别计算 A1/A2 的中心点和 SIN/COS 幅值缩放。
3. 对每个码道执行去中心、幅值归一和 `atan2(SIN, COS)`，得到 A1/A2 电角度。
4. 按配置把 A1/A2 映射成 15/16 周期码道。
5. 计算 `p16` 与 `p15` 的零点修正和方向修正。
6. 输出单圈机械角：

```text
angle = wrap(p16 - p15)
```

因为 16 周期与 15 周期相差 1，理想情况下 `p16 - p15` 正好对应一圈机械绝对角。

一致性检查会比较 `angle * 16` 与 16 周期电角、`angle * 15` 与 15 周期电角。如果幅值过低、ADC 贴边、未校准或双码道相位不一致，`v2_result.status` 会置位，主角度保持上一次有效值。

## FreeMASTER 使用流程

连接参数：

- UART：`LPUART0`
- 波特率：`115200`
- 格式：`8N1`

推荐 bring-up 顺序：

1. 连接 FreeMASTER，加载现有 `freemaster/digital_encoder.pmpx`。
2. 先观察四路 raw：
   - `adc_result.a1_sin_raw`
   - `adc_result.a1_cos_raw`
   - `adc_result.a2_sin_raw`
   - `adc_result.a2_cos_raw`
3. 缓慢转动转子，确认四路都有周期变化，且没有长时间贴近 0 或 65535。
4. 写 `fm_cal_enable = 1`，或点击界面里的 Online Calibration。
5. 在 `fm_cal_progress` 从 0 到 100 期间，慢慢转动转子至少一整圈。
6. `fm_cal_done = 1` 后，观察：
   - `encoder_result.angle_deg`
   - `encoder_result.angle_counts`
   - `v2_result.angle_deg`
   - `v2_result.status`
7. 如果需要重新设置当前机械零点，校准完成后写 `fm_zero_ctrl = 1`。

校准结束时固件会自动捕获当前点作为临时零点；后续也可以通过 `fm_zero_ctrl` 重新捕获零点。

## 关键 FreeMASTER 变量

ADC 与电压：

- `adc_result.a1_sin_raw / a1_sin_voltage`
- `adc_result.a1_cos_raw / a1_cos_voltage`
- `adc_result.a2_sin_raw / a2_sin_voltage`
- `adc_result.a2_cos_raw / a2_cos_voltage`
- `adc_result.temperature`

兼容角度输出：

- `encoder_result.angle_deg`
- `encoder_result.angle_counts`
- `encoder_result.magnitude`
- `encoder_result.elec_angle_deg`

V2 调试输出：

- `v2_result.angle_deg`
- `v2_result.angle_counts`
- `v2_result.phase16_deg`
- `v2_result.phase15_deg`
- `v2_result.coarse_deg`
- `v2_result.mag16`
- `v2_result.mag15`
- `v2_result.status`
- `v2_diag.mapping`
- `v2_diag.dir16`
- `v2_diag.dir15`
- `v2_diag.phase16_error_deg`
- `v2_diag.phase15_error_deg`
- `v2_diag.phase_a1_deg`
- `v2_diag.phase_a2_deg`

校准控制：

- `fm_cal_enable`：写 1 开始 RAM 在线校准。
- `fm_cal_done`：1 表示本次校准成功。
- `fm_cal_progress`：0..100。
- `fm_cal_state`：0 idle，1 running，2 done，3 failed。
- `fm_cal_status`：校准失败或状态码。
- `fm_zero_ctrl`：写 1 捕获当前机械零点。
- `fm_reset_ctrl`：写 1 软件复位。

`fm_direction`、`encoder_result.turns`、`encoder_result.speed_rpm/speed_dps` 当前仅保留兼容入口，不作为 V2 阶段验收目标。

## 状态码速查

`v2_result.status` 和 `fm_encoder_status` 使用位标志：

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
gcc -std=c99 -Wall -Wextra -Isource tests/test_app_encoder_v2.c source/app_encoder_v2.c -lm -o debug/test_app_encoder_v2.exe
debug/test_app_encoder_v2.exe
```

文档/补丁检查：

```powershell
git diff --check -- README.md
```

## 当前限制

- 校准参数不写 flash，复位后重新校准。
- 当前只输出单圈绝对角，不做断电保持的多圈计数。
- 当前速度、转数和方向控制变量没有接入 V2 主算法。
- 校准模型为中心点加 SIN/COS 幅值缩放，不是完整 2x2 椭圆拟合。
- 若后续仍有规律性非线性误差，应继续从机械同心度、码道相位关系和更完整的椭圆/谐波补偿入手。
