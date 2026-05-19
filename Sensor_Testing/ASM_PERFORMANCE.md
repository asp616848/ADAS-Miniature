# Assembly Primitives — Performance Notes

Repository: ADAS-Miniature / Sensor_Testing

## Environment
- MCU: STM32F103RB (Cortex-M3)
- SYSCLK (tested): 8 MHz (project uses HSI/2 × PLL×2 per SystemClock_Config)
- DWT cycle counter used for measurements; calibration: 8 cycles/us (CYCLES_PER_US = 8)
- UART: USART2 @ 115200 (printf retargeted to `huart2`)
- Test harness: `perf_test()` in `Core/Src/main.c` — runs at boot, prints per-iteration values and min/avg/max.
- Iterations: N = 8 by default in `perf_test()`.

## Assembly file
- Path: Core/Src/sensor_asm.s
- Exported functions (callable from C via `extern` in `main.c`):
  - `asm_dwt_init()` — enable DWT cycle counter
  - `asm_delay_us(uint32_t us)` — busy-wait microsecond delay
  - `asm_ultrasonic_ticks(GPIO_TypeDef *tp, uint16_t t_pin, GPIO_TypeDef *ep, uint16_t e_pin)` → returns 32-bit echo pulse width in DWT cycles (0 → timeout)
  - `asm_adc1_read()` → returns 12-bit ADC sample or 0xFFFF on error
  - `asm_gpio_set(uint32_t port_base, uint32_t pin_mask)` — write BSRR
  - `asm_gpio_reset(uint32_t port_base, uint32_t pin_mask)` — write BRR
  - `asm_tim_set_ccr(uint32_t tim_base, uint32_t ccr_off, uint32_t val)` — write CCR register

## Where the perf harness lives
- `Core/Src/main.c`
  - `dwt_get_cycles()` reads DWT->CYCCNT (`*(volatile uint32_t *)0xE0001004U`).
  - `perf_test()` runs a small benchmark for the listed assembly primitives and prints per-iteration numbers and min/avg/max.
  - `perf_test()` is called once at boot (after `asm_dwt_init()` and `HAL_ADC_Start(&hadc1)`).

## Observed performance (measured on your board)
Notes: cycles → microseconds = cycles / 8 (since DWT counts at 8 MHz). Results shown are from the N=8 run printed by `perf_test()`.

- asm_ultrasonic_ticks (call duration):
  - min = 19040 cycles (2380 µs)
  - avg = 19048 cycles (2381 µs)
  - max = 19055 cycles (2381 µs)
  - per-iteration samples (example): 19053,19055,19040,...
  - Notes: this is the total call duration (function execution + sensor wait). The returned echo pulse width (the function return value) may be much smaller; the call duration reflects waiting for the echo and processing.
  - Failure/timeout path observed earlier: 200185 cycles (~25023 µs, ≈25 ms) when no echo was detected (function returns 0 in that case).

- asm_adc1_read:
  - min = 119 cycles (14 µs)
  - avg = 119 cycles (14 µs)
  - max = 119 cycles (14 µs)
  - Earlier (before starting ADC) an error return showed 48 cycles (~6 µs) — this is the error/early-exit path in assembly which returns 0xFFFF quickly when ADC is not powered/ready.

- asm_gpio_set:
  - min = 26 cycles (3.25 µs)
  - avg = 26 cycles (3.25 µs)
  - max = 26 cycles (3.25 µs)
  - Very fast memory-mapped BSRR write.

- asm_gpio_reset:
  - min = 26 cycles (3.25 µs)
  - avg = 26 cycles (3.25 µs)
  - max = 26 cycles (3.25 µs)

- asm_tim_set_ccr:
  - min = 28 cycles (3.5 µs)
  - avg = 28 cycles (3.5 µs)
  - max = 28 cycles (3.5 µs)
  - Single CCR register write + function overhead.

## Test notes and recommendations
- The ultrasonic call latency is dominated by the physical echo time — to benchmark pure function overhead, use a short-circuit test or mock echo (or measure when echo is present at a known distance). For example, place a reflector at 10 cm to see a deterministic echo and much smaller call durations than the timeout.
- The `asm_adc1_read()` assembly routine returns quickly with `0xFFFF` when ADC not configured or timed out — ensure `HAL_ADC_Start(&hadc1)` is called before benchmarks (the `main.c` test harness now starts ADC prior to `perf_test()`).
- For stable statistics, increase `N` in `perf_test()` or run multiple boots and average results.
- When converting assembly to C, compare the same min/avg/max numbers printed by `perf_test()` to quantify performance differences.

## Quick pointers to files
- Perf harness and changes: `Core/Src/main.c`
- Assembly primitives: `Core/Src/sensor_asm.s`

---
If you want, I can:
- Add the C versions of specific primitives (start with `asm_adc1_read()`), run the same N-iteration test, and append C vs ASM comparisons to this document.
- Increase `N` or add CSV output to make importing results into tools easier.
