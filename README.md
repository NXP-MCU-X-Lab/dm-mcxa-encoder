# NXP MCXA344 Dual-Track Inductive Encoder

> English | [中文](README.zh-CN.md)

Reference firmware and host dashboard for an HW_V2 dual-track inductive absolute
encoder built around the NXP MCXA34x/MCXA344 device family. The firmware decodes
16/15 Vernier tracks into a single-turn absolute angle, accumulates multi-turn
motion in RAM, and exposes live data through FreeMASTER.

![HW_V2 inductive encoder hardware](doc/HW_V2.jpg)

## Features

- 16/15 Vernier single-turn absolute angle solver with 16-bit output counts
- Heydemann-style ellipse correction for each `(sin, cos)` track
- Type-II software PLL for filtered angle and angular velocity
- Output-layer dead-band / hysteresis: `0.015 deg` angle threshold and `1` count threshold
- Runtime magnitude monitoring and AGC trim after valid NVM calibration is loaded
- RAM-only signed multi-turn counter with host reset command
- FreeMASTER TSA variables plus a web dashboard and desktop `.pmpx` project
- DWT CYCCNT profiling for `encoder_process()` and total ADC ISR callback time

## Hardware

| Item | Value |
| --- | --- |
| MCU family | NXP MCXA34x / MCXA344 firmware target |
| Current Keil target note | `ind_encoder.uvprojx` is configured with `MCXA343VLH` / `NXP.MCXA343_DFP.25.09.00`, while source defines use `CPU_MCXA344VLL` and `MCXA344_SERIES` |
| Core clock | Cortex-M33 + FPU, 180 MHz firmware clock configuration |
| Analog front-end | MCXA OPAMP0/1 plus external TLV9062 |
| ADC sampling | LPADC0/1, 4 raw channels, 10 kHz, CTIMER0 trigger, 8x hardware averaging |
| Host link | FreeMASTER over LPUART0, 115200 8N1, shared with debug console |
| Persistent calibration | Last 8 KB Flash area, 128-byte slots, monotonically selected sequence, CRC32 validation |

### Signal Wiring

| Signal | Pin |
| --- | --- |
| A1 SIN | OPAMP0_OUT -> P2_15 (ADC0_A2) |
| A1 COS | OPAMP1_OUT -> P2_19 (ADC1_A2) |
| A2 SIN | TLV9062 external op-amp -> P2_6 (ADC1_A3) |
| A2 COS | TLV9062 external op-amp -> P2_7 (ADC0_A7) |
| FreeMASTER UART | LPUART0 |
| Heartbeat LED | P3_11, toggles every 100 ms |
| ISR probe | P3_0, optional scope timing probe |

## Repository Layout

```text
source/
  app_adc.{h,c}              CTIMER0-triggered LPADC sampling
  app_encoder.{h,c}          OS/HAL-independent encoder algorithm
  app_encoder_runtime.{h,c}  ADC callback, runtime trim, commands, snapshots, profiling
  app_encoder_storage.{h,c}  Flash calibration slot storage and CRC32 validation
  app_encoder_defaults.c     Board default calibration values
  app_freemaster.{h,c}       FreeMASTER init and TSA variable table
  hardware_init.{h,c}        OPAMP, pins, clock, debug UART
  main.c                     Boot entry and service loop

freemaster/
  index.html                 Web dashboard
  digital_encoder.pmpx       FreeMASTER desktop project
  simple-jsonrpc-js.js       JSON-RPC over WebSocket helper

doc/
  HW_V2.jpg                  HW_V2 hardware photo
  AN15044.pdf                Reference application note
  MCXA34x_based_Inductive_Encoder_v1.1.docx
```

## Build and Flash

Open the Keil project directly or run the command-line build:

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

Expected toolchain:

- Keil MDK with ARMCLANG V6.23
- NXP MCXA343 DFP 25.09.00 as currently referenced by the project file
- MCXA344-series SDK headers and source files already vendored in this repository

The project output name is `ind_encoder`. The scatter file reserves the final
8 KB of the 128 KB Flash image for factory calibration storage.

## Calibration and Operation

### First Factory Calibration

1. Connect the HW_V2 inductive encoder hardware and power the board.
2. Open `freemaster/index.html` or `freemaster/digital_encoder.pmpx`.
3. Connect FreeMASTER to LPUART0 at 115200 baud.
4. Press **Factory Cal**.
5. Rotate at least one complete mechanical revolution during the capture window.
6. Stop at the desired zero position. The last captured frame becomes factory zero.

The firmware captures `8192` decimated calibration samples, solves both track
ellipse corrections, captures zero, writes one 128-byte calibration slot, and
selects the highest valid sequence number on the next boot.

### Runtime Commands

| Command | Effect |
| --- | --- |
| **Zero Here** | Capture the current angle as a RAM zero offset; lost after reset |
| **Reset Turns** | Clear the RAM-only multi-turn counter |
| **Factory Cal** | Re-run factory calibration and persist it to Flash |
| **MCU Reset** | Trigger an NVIC system reset |

## FreeMASTER Dashboard

The dashboard reads TSA-exposed variables from the firmware:

- `encoder_result.angle_deg`, `angle_counts`, `multi_turn_deg`, and `turn_count`
- `encoder_result.angular_velocity_dps`
- `encoder_result.mag16`, `mag15`, `mag16_raw`, and `mag15_raw`
- `encoder_result.status`
- `adc_result` raw channels, sample count, and overrun count
- DWT profiler counters: `encoder_perf_process_cycles`, `encoder_perf_isr_cycles`, and peak values

Status bits are decoded by the dashboard into warning and fault states.

| Bit | Name | Meaning |
| --- | --- | --- |
| `0x0001` | `NOT_CALIBRATED` | No valid calibration is loaded |
| `0x0002` | `TRACK16_WEAK` | 16-cycle track magnitude below the floor |
| `0x0004` | `TRACK15_WEAK` | 15-cycle track magnitude below the floor |
| `0x0008` | `ADC_RAIL` | At least one ADC channel is clipped |
| `0x0010` | `TRACK_MISMATCH` | 16/15 residual exceeds tolerance |
| `0x0020` | `CAL_FAILED` | Factory calibration solve failed |
| `0x0040` | `HOLD_LAST` | Firmware is holding the previous valid angle |
| `0x0080` | `CAL_STORAGE_INVALID` | NVM calibration CRC or block validation failed |
| `0x0100` | `FACTORY_CAL_REQUIRED` | Defaults are active and factory calibration is required |

## Configuration

Most tuning is compile-time and lives in `source/app_encoder.h` and
`source/app_adc.h`.

| Macro | Default | Purpose |
| --- | --- | --- |
| `ADC_SAMPLE_RATE_HZ` | `10000` | Realtime ADC sample rate; keep CTIMER0 setup in sync |
| `ENCODER_CAL_SAMPLE_COUNT` | `8192` | Factory calibration sample count |
| `ENCODER_TRACKING_BW_HZ` | `100.0f` | Type-II PLL closed-loop bandwidth |
| `ENCODER_TRACKING_ZETA` | `0.707f` | Type-II PLL damping ratio |
| `ENCODER_OUTPUT_DEADBAND_DEG` | `0.015f` | Published angle dead-band threshold |
| `ENCODER_ANGLE_COUNT_HYSTERESIS` | `1` | Published count-domain hysteresis |
| `ENCODER_MAG_WINDOW_BINS` | `32` | Rotor-angle bins for magnitude display window |
| `ENCODER_RUNTIME_TRIM_GAIN_STEP_LIMIT` | `0.02f` | Per-update AGC gain trim limit |
| `ENCODER_RUNTIME_TRIM_GAIN_TOTAL_LIMIT` | `0.5f` | Total AGC gain trim limit |

## Known Notes and Limitations

- Multi-turn position is RAM-only. Power loss or reset clears the turn counter.
- **Zero Here** is also RAM-only. Factory calibration is the persistent zero source.
- Very slow motion below the output dead-band rate can be reported in steps by the published angle path.
- Runtime AGC is enabled only when NVM calibration is the active calibration source.
- The current Keil project metadata names an MCXA343 pack/device while the firmware sources target the MCXA344 series. Keep this in mind when changing device packs or regenerating the project.
- No top-level `LICENSE`, `CONTRIBUTING`, or `SECURITY` file is included in this repository snapshot.

## Verification

This README describes the current firmware interfaces and constants. For a
documentation-only change, verify with:

```powershell
git diff --check
```

Firmware builds are still performed through Keil MDK:

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

## License

BSD-3-Clause, as declared by the source file SPDX headers.
