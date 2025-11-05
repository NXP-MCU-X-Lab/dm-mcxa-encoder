# NXP MCXA344 Inductive Position Encoder

A high-precision inductive position encoder implementation for NXP MCXA344 microcontroller, featuring advanced signal processing, automatic calibration, and real-time angle computation.

## Table of Contents
- [Overview](#overview)
- [Hardware Architecture](#hardware-architecture)
- [Software Architecture](#software-architecture)
- [Key Features](#key-features)
- [Implementation Details](#implementation-details)
- [Getting Started](#getting-started)
- [Usage Guide](#usage-guide)
- [Development Tools](#development-tools)
- [Performance Specifications](#performance-specifications)
- [Troubleshooting](#troubleshooting)

## Overview

This project implements a complete inductive encoder system using the NXP MCXA34x ARM Cortex-M33 microcontroller. The system processes differential sin/cos signals from inductive sensors to provide high-resolution angular position measurements with multi-turn capability.

### System Components
- **MCXA344 Microcontroller**: ARM Cortex-M33 with dual 16-bit ADCs
- **Inductive Sensor Interface**: Dual OPAMP channels for differential signal conditioning
- **Signal Processing**: Advanced calibration and angle computation algorithms
- **Real-time Output**: UART streaming with configurable data formats

## Hardware Architecture

### Pin Configuration
```
OPAMP0 (Sin Channel):
- OPAMP0_INP  → P2_12 (ADC0_A5)
- OPAMP0_INN  → P2_13 (ADC1_A5)  
- OPAMP0_OUT  → P2_15 (ADC0_A2)

OPAMP1 (Cos Channel):
- OPAMP1_INP  → P2_16 (ADC0_A6)
- OPAMP1_INN  → P2_17 (ADC1_A6)
- OPAMP1_OUT  → P2_19 (ADC1_A2)

Temperature Sensor:
- Temperature → ADC0_A26

Debug Interface:
- UART Debug  → LPUART2 (2 Mbps)
```

### Hardware Features
- **Dual 16-bit ADCs**: Independent ADC0 and ADC1 for simultaneous sampling
- **OPAMP Integration**: On-chip operational amplifiers for signal conditioning

## Software Architecture

### Module Structure

```
┌─────────────────────────────────────────────────────────────┐
│                        Application Layer                     │
├─────────────────────────────────────────────────────────────┤
│  main.c      │  Calibration  │  User Interface │  Debug    │
├─────────────────────────────────────────────────────────────┤
│                    Application Services                      │
├─────────────────────────────────────────────────────────────┤
│  app_encoder │  app_adc       │  app_sampler    │  app_timer│
├─────────────────────────────────────────────────────────────┤
│                    Hardware Abstraction Layer(SDK)          │
├─────────────────────────────────────────────────────────────┤
│  fsl_lpadc   │  fsl_opamp     │  fsl_ctimer     │  fsl_clock│
└─────────────────────────────────────────────────────────────┘
```

### Core Modules

#### 1. ADC Driver (`app_adc.c/h`)
- **Dual ADC Management**: Coordinates ADC0 and ADC1 for synchronized sampling
- **Multiple Sampling Modes**: Output-only, full debug, and calibration modes
- **Temperature Integration**: Automatic temperature sampling every 10,000 readings
- **Signal Conditioning**: Hardware averaging (16x for signals, 128x for temperature)

#### 2. Encoder Processing (`app_encoder.c/h`)
- **Advanced Calibration**: 2×2 matrix transformation for ellipse correction
- **Signal Quality Monitoring**: Magnitude-based gating with 0.6 threshold
- **Phase Unwrapping**: Multi-turn capability with electrical cycle tracking
- **Adaptive Filtering**: Velocity-dependent jitter suppression
- **High Resolution**: 16-bit angle quantization (65,536 counts per revolution)

#### 3. Sampling Engine (`app_sampler.c/h`)
- **CTIMER1-Based ISR**: Precise 1kHz sampling interrupt
- **Thread-Safe Operation**: Atomic data copying with interrupt protection
- **Performance Monitoring**: ISR execution time tracking
- **Non-Intrusive Design**: Independent from system timing utilities

#### 4. Calibration System
- **Auto-Calibration**: Complete ellipse fitting algorithm
- **Manual Calibration**: Min/max based calibration support
- **Zero Position Setting**: Flexible zero reference configuration
- **Direction Control**: Reversible rotation direction

## Key Features

### Signal Processing Pipeline
1. **Raw ADC Sampling**: Dual-channel synchronized acquisition
2. **Calibration Transformation**: 2×2 matrix correction for sensor imperfections
3. **Signal Quality Assessment**: Magnitude-based validity checking
4. **Angle Computation**: High-precision arctangent calculation
5. **Phase Unwrapping**: Multi-turn tracking with drift compensation
6. **Output Filtering**: Adaptive jitter suppression based on velocity

### Advanced Calibration
- **Ellipse Fitting**: Statistical analysis of sensor characteristics
- **Temperature Compensation**: Thermal drift correction
- **Amplitude Normalization**: Consistent signal scaling
- **Center Point Correction**: DC offset elimination

## Implementation Details

### Encoder Configuration
```c
#define ENCODER_ELEC_CYCLES_PER_REV    4    // Electrical cycles per revolution
#define ENCODER_RESOLUTION_BITS       16    // 16-bit angle resolution
#define ENCODER_MIN_MAG_THRESHOLD     0.6f  // Signal quality threshold
```

### Sampling Configuration
```c
#define SAMPLE_FRQ          (10*1000)        // 1kHz sampling rate
#define TEMP_SAMPLE_INTERVAL (10*1000)     // Temperature every 10k samples
#define ADC_HW_AVG_SIGNAL   kLPADC_HardwareAverageCount16
#define ADC_HW_AVG_TEMP     kLPADC_HardwareAverageCount128
```

### Clock Configuration
- **Core Clock**: 96MHz from FRO_HF
- **ADC Clock**: 32MHz (divided by 3)
- **CTIMER Clock**: 1MHz (divided by 4)
- **UART Baud**: 2Mbps for high-speed data streaming

## Getting Started

### Prerequisites
- NXP MCXA344 development board
- Inductive position sensor with differential outputs
- Keil MDK-ARM development environment
- Serial terminal software (TeraTerm, PuTTY, etc.)

### Hardware Setup
1. Connect inductive sensor to designated pins (P2_12-P2_19)
2. Ensure proper power supply (3.3V)
3. Connect debug UART to host computer
4. Install jumpers for OPAMP configuration if needed

### Software Build
1. Open `ind_encoder.uvprojx` in Keil MDK-ARM
2. Select appropriate target configuration
3. Build the project (F7)
4. Download to target device

## Usage Guide

### Serial Interface

#### Connection Parameters
- **Baud Rate**: 2,000,000 bps
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

#### Startup Sequence
```
=== System Clocks ===
Core Clock:    96000000 Hz (96 MHz)
FRO_HF:        96000000 Hz (96 MHz)
FRO_HF_DIV:    48000000 Hz (48 MHz)

=== Select ADC Sampling Mode (3s timeout) ===
1: Normal Mode (OPAMP outputs only, 2ch)
2: Debug Mode (All 6 channels: INP, INN, OUT)
3: Auto Calibration (OPAMP outputs only, 2ch)
```

#### Operating Modes

**Mode 1: Normal Operation**
- Samples OPAMP0_OUT and OPAMP1_OUT only
- Optimal performance for position tracking
- Temperature compensation included

**Mode 2: Debug Mode**
- Samples all 6 channels (INP, INN, OUT for both OPAMPs)
- Comprehensive signal analysis
- Useful for sensor characterization

**Mode 3: Auto Calibration**

- Automatic sensor calibration routine
- Ellipse fitting algorithm
- Zero position setting

#### Data Output Format
```
Angle:   123.45 deg | Counts:  54321 | Turns:    -1 | Sin: 34567 | Cos: 45678
```

### Calibration Procedure

#### Automatic Calibration
1. Select Mode 3 at startup
2. Rotate encoder slowly for one full revolution
3. Press any key to start calibration
4. System will collect 300,000 samples over 2048 points
5. Position mechanical zero and press any key
6. Calibration completes automatically

#### Manual Calibration
Use the encoder API functions for programmatic calibration:
```c
encoder_calibrate(sin_min, sin_max, cos_min, cos_max);
encoder_apply_calibration(&custom_calibration);
encoder_set_zero_deg(zero_angle);
```

### FreeMASTER Integration

The project includes FreeMASTER support for real-time debugging and visualization:

#### FreeMASTER Setup
1. Open `freemaster\index.html` in web browser
2. Configure serial connection to match UART settings
3. Load the project map file
4. Enable real-time data monitoring

#### Available Variables
- `encoder_result.angle_deg`: Mechanical angle (degrees)
- `encoder_result.angle_counts`: Quantized angle (counts)
- `encoder_result.turns`: Turn counter
- `encoder_result.magnitude`: Signal magnitude
- `adc_result.temperature`: Temperature reading

#### Oscilloscope Features
- Real-time angle plotting
- Signal waveform visualization
- Calibration parameter monitoring
- Performance metrics display

## Development Tools

### Keil MDK-ARM
- Project file: `ind_encoder.uvprojx`
- Target device: MCXA343VLH
- Compiler: ARM Compiler 6.23
- Optimization: Speed optimized

### Debugging Features
- **Test Pin**: GPIO toggle for ISR timing measurement
- **Performance Metrics**: ISR execution time tracking
- **Error Handling**: Comprehensive fault detection
- **Serial Debug**: Extensive diagnostic output

### Code Conventions
- **Naming**: snake_case for C functions and variables
- **Prefixes**: Module-specific prefixes (`adc_*`, `encoder_*`, `sampler_*`)
- **Globals**: Static scope preferred, minimal global state
- **Documentation**: Comprehensive API documentation

## Performance Specifications

### Sampling Performance
- **Update Rate**: 10kHz
- **ADC Resolution**: 16-bit
- **Angle Resolution**: 16-bit (0.0055° per count)
- **Multi-turn Range**: ±2,147,483,647 turns


## Support

For technical support, please refer to:
- NXP MCXA344 documentation
- Project source code and comments
- FreeMASTER user guide
- Keil MDK-ARM documentation
