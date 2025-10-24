# mcxa_encoder
MCXA3xx Inductive Position Encoder

## Coding Conventions
- Naming style: snake_case for C functions and variables (rt-thread style).
- Module prefixes:
  - `adc_*` for ADC sampling (`app_adc_sample.*`).
  - `encoder_*` for angle processing (`app_encoder.*`).
  - `sampler_*` for CTIMER-driven ISR sampling (`app_sampler.*`).
- Public API examples:
  - `adc_init(mode)`, `adc_read()`, `adc_get_mode()`
  - `encoder_init()`, `encoder_calibrate()`, `encoder_process()`
  - `sampler_init(freq_hz)`, `sampler_start()`, `sampler_stop()`
- Globals: avoid unless necessary; use `static` for internal state.
- Readability: keep functions short, clear responsibilities; prefer descriptive names.
- Constants/macros: UPPER_CASE for compile-time config (e.g., `ENCODER_RESOLUTION_BITS`).
