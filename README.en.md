# NXP MCXA344 Dual-Track Inductive Encoder

> English · [中文](README.md)

Reference design for a dual-track inductive absolute encoder on the
MCXA344 (Cortex-M33 @ 180 MHz): **16/15 Vernier single-turn absolute angle
+ multi-turn accumulation + Type-II software PLL**, with a FreeMASTER
real-time monitoring dashboard. The algorithm layer has no OS dependency
and is unit-testable in isolation.

---

## Specifications

| Item | Value |
| --- | --- |
| MCU | NXP MCXA344 · Cortex-M33 + FPU · 180 MHz |
| Analog front-end | OPAMP0/1 + external TLV9062 · synchronous LPADC0/1 |
| Sample rate | 10 kHz · 4 channels · 8× hardware averaging · CTIMER0 triggered |
| Angle resolution | 16-bit / revolution · 1 LSB ≈ 0.0055° |
| Static noise | ≤ 0.015° (output hysteresis · AS5048-style · ~2.7 LSB) |
| Tracking bandwidth | 100 Hz · ζ = 0.707 (Type-II PLL, tunable) |
| Multi-turn range | INT32 · ±2³¹ revolutions · RAM-only |
| Compute load | DWT-measured, typically < 10 % CPU @ 180 MHz / 10 kHz |
| Persistent calibration | Last 8 KB of Flash · CRC32 + dual-block redundancy |
| Host tool | FreeMASTER · LPUART0 @ 115200 8N1 · TSA self-describing |

---

## Principle

### Signal chain

```
LPADC0/1  ─┐
  10 kHz  │ 4 channels synchronous, 8× hardware averaging
  CTIMER0 │
trigger  ─┘
   │
   ▼
Ellipse correction  Heydemann: min/max centre + LSQ shape + Cholesky T
   │
   ▼
atan2  →  φ16 (16-cycle track), φ15 (15-cycle track)
   │
   ▼
Vernier solve
   coarse = wrap(φ16 − φ15)                one zero-cross per revolution — coarse
   angle  = coarse + (φ16 − 16·coarse)/16  fine refinement
   │
   ▼
Type-II tracking observer               BW = 100 Hz, ζ = 0.707
   │  → angular_velocity_dps
   ▼
Multi-turn accumulator (±1 across 0°/360° wrap)
   │
   ▼
Output hysteresis (2.7 LSB ≈ 0.015°)
   │
   ▼
encoder_result { angle_deg, angle_counts, multi_turn_deg,
                 angular_velocity_dps, mag16/15, turn_count, status }
```

### Vernier 16/15

Two inductive tracks generate 16 and 15 electrical cycles per mechanical
revolution. The phase difference `φ16 − φ15` traverses **exactly 360°
over one full revolution** — that is the unique coarse angle. The
high-resolution `φ16` then refines it inside the chosen 16-cycle period.
This is the classic Vernier-caliper trick applied to angle encoding;
industrial RDCs (AD2S1210), magnetic encoders (iC-MU) and inductive
encoders from Renishaw all use a similar high/low-resolution split.

### Heydemann ellipse correction

Raw `(sin, cos)` samples fall on an ellipse instead of the unit circle
because of amplitude mismatch, offset, and non-orthogonality. A two-step
fit yields the correction matrix:

1. **Centre** — midpoint of (min, max). Robust against uneven sample
   density along the rotor angle.
2. **Shape** — algebraic LSQ fit of `a·x² + b·xy + c·y² = 1` on centred
   samples; the Cholesky lower-triangular `T` of `Tᵀ·T = M` is then
   computed.

Applying `(corr_sin, corr_cos) = T · ((raw_sin, raw_cos) − centre)`
yields `‖corr‖ ≈ 1.0` at every rotor angle, so `atan2` produces the true
electrical phase.

### Type-II tracking observer (software PLL)

The canonical resolver-to-digital output stage. The closed-loop
characteristic equation is `s² + Kp·s + Ki` with `Kp = 2ζωₙ`,
`Ki = ωₙ²`.

- **Zero steady-state phase lag** at constant velocity
- Outputs both **filtered angle and angular velocity** (no
  post-differentiation needed)
- Bandwidth `ENCODER_TRACKING_BW_HZ` is tunable (default 100 Hz)

### Static jitter suppression

Output-layer hysteresis (`ENCODER_OUTPUT_DEADBAND_DEG = 0.015°`, ~2.7 LSB):
when the new angle differs from the last published value by less than
this threshold, the previous value is held.

- The same approach used by AS5048 / iC-MU and similar production
  encoders — **leave the PLL math untouched; apply the dead-band only at
  the publish stage**
- Trade-off: motion slower than `(threshold / dt)` ≈ 150 °/s is reported
  as stationary
- A separate `ENCODER_ANGLE_COUNT_HYSTERESIS = 1` LSB keeps the integer
  `angle_counts` from oscillating by ±1

### Branch-slip guard

If a single ADC frame yields a Vernier jump larger than half a fine
period, ±8 candidate branches are searched and the one closest to the
predicted angle wins. The prediction is `last_angle_raw + velocity·dt`
— the **raw** Vernier output, not the PLL-filtered angle, so that the
tracking observer's lag cannot bias the discrete branch decision.

### Multi-turn accumulation

If the published angle exhibits a |Δ| > 180° wrap between consecutive
frames, `turn_count ±= 1`. INT32-saturating, RAM-only (lost on power
cycle; can be cleared on demand).

### Runtime AGC

After NVM calibration is loaded, the raw magnitude mean is observed over
one full revolution and the effective T-matrix gain is nudged by EMA to
compensate for amplitude drift caused by temperature, OPAMP gain, or
supply variation. Both per-step and total deltas are clamped; abnormal
or rail-clipped samples are skipped.

---

## Architecture

```
source/
  app_adc.{h,c}              CTIMER0-triggered LPADC 4-channel sampling (8× hw avg)
  app_encoder.{h,c}          Core algorithm — no OS / no HAL dependency, unit-testable
  app_encoder_runtime.{h,c}  ADC callback, AGC trim, snapshot publish, commands, DWT profiler
  app_encoder_storage.{h,c}  Calibration Flash storage (last 8 KB · CRC32 · dual block)
  app_encoder_defaults.c     Power-on default calibration parameters
  app_freemaster.{h,c}       FreeMASTER init and TSA variable table
  hardware_init.{h,c}        OPAMP / pin / clock / debug UART setup
  clock_config.{h,c}         FRO 180 MHz clock tree
  main.c                     Boot entry and main loop (WFI low-power)

freemaster/
  index.html                 Web dashboard (Monitor + Diagnostics tabs)
  digital_encoder.pmpx       FreeMASTER desktop client project
  simple-jsonrpc-js.js       JSON-RPC over WebSocket (PCM protocol)
```

**Data flow**: `encoder_process` runs inside the ADC ISR, writes a
volatile snapshot. The main loop `EncoderApp_Service` copies it out
under interrupt-mask to the published `encoder_result`, which the
FreeMASTER TSA table exposes to the host and `index.html`.

---

## Performance monitoring

The ISR brackets `encoder_process` and the full ADC callback with the
Cortex-M33 **DWT CYCCNT** counter (1-cycle precision) and maintains
running peaks. The Diagnostics tab polls them at ~0.4 Hz and converts
cycles to microseconds and CPU utilisation.

| TSA field | Meaning |
| --- | --- |
| `encoder_perf_process_cycles` | Most recent `encoder_process` duration (cycles) |
| `encoder_perf_process_max` | Peak since boot |
| `encoder_perf_isr_cycles` | Most recent full ADC ISR callback (cycles) |
| `encoder_perf_isr_max` | Peak since boot |
| `encoder_perf_core_clock_hz` | CPU frequency, used by the host to convert to µs |

CPU load = (ISR µs) / 100 µs × 100 % (10 kHz sample rate → 100 µs budget per frame).

---

## Getting started

### Hardware wiring

| Signal | Pin |
| --- | --- |
| A1 SIN | OPAMP0_OUT → P2_15 (ADC0_A2) |
| A1 COS | OPAMP1_OUT → P2_19 (ADC1_A2) |
| A2 SIN | TLV9062 external op-amp → P2_6  (ADC1_A3) |
| A2 COS | TLV9062 external op-amp → P2_7  (ADC0_A7) |
| FreeMASTER UART | LPUART0 (shared with debug console) |
| Heartbeat LED | P3_11 (toggles every 100 ms — firmware-alive indicator) |
| ISR probe | P3_0 (optional, for scope timing checks) |

### Build & flash

```powershell
C:\Keil_v5\UV4\uVision.com -b ind_encoder.uvprojx -t ind_encoder -j0
```

Toolchain: **Keil MDK + ARMCLANG V6.23**. Programs the MCXA344-EVK.

### First-time factory calibration

1. Power on the board with the inductive track hardware connected
2. Open `freemaster/index.html` (or `freemaster/digital_encoder.pmpx`)
   and connect to LPUART0 @ 115200
3. From the Monitor tab, click **Factory Cal**
4. **Rotate at least one full revolution** during the ~8 s capture
   window (smooth motion preferred)
5. Stop at the desired zero position — the last captured frame becomes
   the factory zero
6. Calibration is written to the last 8 KB Flash sector (CRC32 protected,
   survives power cycles)

If calibration fails, the firmware falls back to the on-chip defaults
and the Alert Ribbon raises `CAL_FAILED` / `FACTORY_CAL_REQUIRED`.

### Daily operation

The **Monitor** tab shows in real time:

- Absolute-angle dial with needle and 0/90/180/270 ticks
- Multi-turn cumulative angle and revolution counter
- Velocity panel (°/s, RPM, direction bar)
- Top Alert Ribbon — three states (green / amber / red) with decoded
  active fault names

The **Diagnostics** tab shows:

- Status word and calibration source (NVM / DEFAULT / FACTORY_PENDING)
- mag16 / mag15 (mean / raw)
- ADC sample and overrun counts
- **Compute Budget** — `encoder_process` and ISR-total timing
  (µs + cycles + peak + CPU%)

| Button | Action |
| --- | --- |
| **Zero Here** | Record the current angle as the RAM zero (lost on power cycle) |
| **Reset Turns** | Clear the multi-turn counter |
| **Factory Cal** | Re-run factory calibration and persist to Flash |
| **MCU Reset** | NVIC system reset |

---

## Status bits

`encoder_result.status` is a bitmask; the Monitor tab Alert Ribbon
decodes it automatically. Bits marked ★ are classified as hard faults
(red); the others are warnings (amber).

| Bit | Name | Meaning |
| --- | --- | --- |
| `0x0001` | `NOT_CALIBRATED` | No valid calibration is currently loaded |
| `0x0002` ★ | `TRACK16_WEAK` | 16-cycle track amplitude below the floor |
| `0x0004` ★ | `TRACK15_WEAK` | 15-cycle track amplitude below the floor |
| `0x0008` ★ | `ADC_RAIL` | ADC channel clipping (zero or full scale) |
| `0x0010` | `TRACK_MISMATCH` | 16/15 dual-track residual exceeds tolerance |
| `0x0020` ★ | `CAL_FAILED` | Factory calibration solver failed |
| `0x0040` | `HOLD_LAST` | Holding the previous valid angle |
| `0x0080` ★ | `CAL_STORAGE_INVALID` | NVM block CRC mismatch |
| `0x0100` | `FACTORY_CAL_REQUIRED` | Booted with defaults, needs factory cal |

---

## Key tunables

The macros below cover almost every use case without re-architecting the
algorithm:

| Macro | Default | Note |
| --- | --- | --- |
| `ADC_SAMPLE_RATE_HZ` | 10000 | ADC sample rate (keep CTIMER config in sync) |
| `ENCODER_TRACKING_BW_HZ` | 100.0 | PLL closed-loop bandwidth; raise for high-speed apps |
| `ENCODER_TRACKING_ZETA` | 0.707 | Damping ratio (Butterworth default) |
| `ENCODER_OUTPUT_DEADBAND_DEG` | 0.015 | Output hysteresis (° — higher = quieter at rest) |
| `ENCODER_ANGLE_COUNT_HYSTERESIS` | 1 | Counts-domain hysteresis (LSB) |
| `ENCODER_MAG_WINDOW_BINS` | 32 | Bins of the rotor-angle window for mag mean / AGC |

---

## License

BSD-3-Clause (see source file headers).
