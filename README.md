# NXP MCXA344 Inductive Encoder Demo

This project validates a dual-track inductive encoder on MCXA344. The firmware samples four ADC channels at 10 kHz, solves a 16/15-cycle Vernier absolute angle, and exposes the result through FreeMASTER.

## Current Architecture

- `source/app_adc.*`: CTIMER0-triggered LPADC0/1 realtime sampling.
- `source/app_encoder.*`: host-testable encoder math, calibration fitting, zero capture, and RAM-only runtime trim.
- `source/app_encoder_storage.*`: factory calibration block packing, CRC, sequence selection, and Flash storage.
- `source/app_encoder_runtime.*`: ADC callback, snapshot publishing, factory calibration state machine, zero capture, and runtime trim wiring.
- `source/app_freemaster.*`: FreeMASTER TSA variables and commands.

## Calibration Model

Normal users should not need FreeMASTER calibration. Boot order is:

1. Load the latest valid factory calibration block from Flash.
2. If the block is missing or CRC-invalid, fall back to board defaults and set warning status bits.
3. Apply RAM-only runtime center trim when the signal is healthy and enough phase coverage has been observed.

The factory calibration block is stored in the last 8 KB Flash sector reserved by the Keil target:

```text
0x0001E000..0x0001FFFF
```

The linker scatter file limits code placement to `0x00000000..0x0001DFFF`.

Runtime trim only adjusts the RAM overlay for:

- A1 `center_sin`
- A1 `center_cos`
- A2 `center_sin`
- A2 `center_cos`

It never changes stored factory calibration, gain, phase, mechanical zero, or LUT data.

## Factory Service Flow

Use `freemaster/index.html` or write TSA variables directly:

- `fm_factory_cal_ctrl = 1`: start factory calibration.
- `fm_factory_cal_state`: `0 idle`, `1 running`, `2 done`, `3 failed`.
- `fm_factory_cal_progress`: `0..100`.
- `fm_factory_cal_status`: encoder/storage status flags.

Factory calibration collects 8192 samples at an effective 1 kHz rate. Rotate through at least one full mechanical revolution during the capture window. The last captured sample is used as the factory zero reference, so the factory fixture/process must end at the intended zero position.

`Zero Here` remains a service command only. It updates the current RAM calibration zero but does not write Flash.

## FreeMASTER Variables

Core outputs:

- `encoder_result.angle_deg`: published display angle, same value as `angle_deg_filtered`.
- `encoder_result.angle_deg_raw`: fast unfiltered Vernier angle solution.
- `encoder_result.angle_deg_filtered`: display angle from a Type-II tracking observer (software PLL) — the canonical resolver-to-digital loop (AD2S1210 / TI SPRAA94). Tracks constant-velocity rotation with zero steady-state phase lag. Tunable via `ENCODER_TRACKING_BW_HZ` (default 100 Hz) and `ENCODER_TRACKING_ZETA` (default 0.707).
- `encoder_result.angular_velocity_dps`: angular velocity in deg/s, output by the tracking observer's velocity integrator.
- `encoder_result.angle_counts`
- `encoder_result.phase16_deg`
- `encoder_result.phase15_deg`
- `encoder_result.coarse_deg`
- `encoder_result.mag16`: A1 track magnitude, smoothed by a first-order IIR (α = 0.1).
- `encoder_result.mag15`: A2 track magnitude, smoothed by a first-order IIR (α = 0.1).
- `encoder_result.status`

Calibration and storage:

- `encoder_calibration_source`: `0 default`, `1 NVM`, `2 factory pending`, `3 invalid`.
- `encoder_storage_crc_ok`: `1` when boot used a valid NVM block.
- `encoder_cal_flat[0..11]`: A1 track, A2 track, A1 zero phase, A2 zero phase.
- `encoder_runtime_trim_enabled`
- `encoder_runtime_trim_active`
- `encoder_runtime_trim_freeze_reason`
- `encoder_runtime_trim_delta[0..3]`: A1 `dSin`, A1 `dCos`, A2 `dSin`, A2 `dCos` in ADC counts.

ADC health:

- `adc_result.a1_sin_raw`
- `adc_result.a1_cos_raw`
- `adc_result.a2_sin_raw`
- `adc_result.a2_cos_raw`
- `adc_sample_count`
- `adc_overrun_count`
- `encoder_sample_rate_hz`

## Status Bits

| Bit | Meaning |
| --- | --- |
| `0x00000001` | Not calibrated |
| `0x00000002` | 16-cycle track weak |
| `0x00000004` | 15-cycle track weak |
| `0x00000008` | ADC rail |
| `0x00000010` | Track mismatch |
| `0x00000020` | Calibration failed |
| `0x00000040` | Holding last valid angle |
| `0x00000080` | Calibration storage invalid |
| `0x00000100` | Factory calibration required |

## Build and Test

Host tests:

```powershell
gcc -std=c99 -Wall -Wextra -Werror -DENCODER_STORAGE_HOST_TEST -I source tests/test_app_encoder_storage.c source/app_encoder.c source/app_encoder_defaults.c source/app_encoder_storage.c -lm -o debug/test_app_encoder_storage.exe
debug/test_app_encoder_storage.exe

gcc -std=c99 -Wall -Wextra -Werror -I source tests/test_app_encoder_runtime_trim.c source/app_encoder.c source/app_encoder_defaults.c -lm -o debug/test_app_encoder_runtime_trim.exe
debug/test_app_encoder_runtime_trim.exe
```

Keil build:

```powershell
C:\Keil_v5\UV4\UV4.exe -b ind_encoder.uvprojx -t ind_encoder
```

Static checks:

```powershell
rg "adc_read\(|delay_ms\(1\)" source
git diff --check
```
