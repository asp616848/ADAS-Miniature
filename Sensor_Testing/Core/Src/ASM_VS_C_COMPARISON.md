**ASM vs C Performance Comparison**

- **Project:** ADAS Miniature — STM32F103RB (Cortex-M3)
- **Test harness:** DWT_CYCCNT cycle counter (memory-mapped) measured at boot-time
- **SYSCLK assumed:** 8 MHz (1 cycle = 0.125 µs → 8 cycles = 1 µs)
- **Iterations:** N = 8 (min / avg / max reported)

**Measurement method**

- We use the Cortex-M DWT cycle counter (DWT->CYCCNT) to measure wall-clock CPU cycles for each primitive. Each measured section wraps the primitive call with a timestamp read before and after, then records the difference (t1 - t0) in cycles.
- Convert cycles → microseconds using: microseconds = cycles / 8 (since SYSCLK = 8 MHz in this project).
- For I/O primitives that interact with external hardware (ultrasonic echo), results depend on physical conditions; timeouts are reported when echo is absent.

**Test parameters & notes**

- N = 8 iterations per primitive, printed per-iteration with min / avg / max.
- ADC: `asm_adc1_read()` vs `c_adc1_read()` — both start conversion by writing ADC1 registers; hardware ADC must be started (`HAL_ADC_Start(&hadc1)`) before running the test.
- GPIO: `asm_gpio_set` / `asm_gpio_reset` vs `c_gpio_set` / `c_gpio_reset` — measure single BSRR/BRR writes.
- TIM CCR write: `asm_tim_set_ccr` vs `c_tim_set_ccr` — measure single TIMx->CCR write.
- Ultrasonic: `asm_ultrasonic_ticks` vs `c_ultrasonic_ticks` — measures trigger + wait-for-echo flow; results depend on echo being present (reflector/hand needed). Timeouts return 0 and inflate measured durations for the wrapper call.

**Observed results (from serial PERF log)**

Measured values are cycles; µs column is cycles/8. These are the numbers captured from the test run you pasted.

| Primitive | Implementation | min (cycles) | avg (cycles) | max (cycles) | min (µs) | avg (µs) | max (µs) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ultrasonic (US1) | asm_ultrasonic_ticks | 19041 | 19345 | 19656 | 2380 | 2418 | 2457 |
| Ultrasonic (US1) | c_ultrasonic_ticks (observed run — unreliable echoes/timeouts) | 20060 | 110148 | 200236 | 2507 | 13768 | 25029 |
| ADC1 read | asm_adc1_read | 119 | 119 | 119 | 14.9 | 14.9 | 14.9 |
| ADC1 read | c_adc1_read | 138 | 138 | 138 | 17.3 | 17.3 | 17.3 |
| GPIO set | asm_gpio_set | 27 | 27 | 27 | 3.4 | 3.4 | 3.4 |
| GPIO set | c_gpio_set | 49 | 49 | 49 | 6.1 | 6.1 | 6.1 |
| GPIO reset | asm_gpio_reset | 27 | 27 | 27 | 3.4 | 3.4 | 3.4 |
| GPIO reset | c_gpio_reset | 48 | 48 | 48 | 6.0 | 6.0 | 6.0 |
| TIM CCR write | asm_tim_set_ccr | 29 | 29 | 29 | 3.6 | 3.6 | 3.6 |
| TIM CCR write | c_tim_set_ccr | 52 | 52 | 52 | 6.5 | 6.5 | 6.5 |

**Analysis**

- Small register-only operations (ADC trigger/read, GPIO BSRR/BRR, TIM CCR writes) show a clear ASM advantage in this project. Observed deltas:
  - `asm_adc1_read` = 119 cycles vs `c_adc1_read` = 138 cycles → ~16% faster.
  - `asm_gpio_set/reset` = 27 cycles vs `c_gpio_set/reset` ≈ 48–49 cycles → ~80% faster for asm.
  - `asm_tim_set_ccr` = 29 cycles vs `c_tim_set_ccr` = 52 cycles → ~79% faster for asm.

- Reasons: Assembly versions are compact, use fewer instructions and avoid volatile pointer derefencing overhead or extra C compiler-generated loads/stores. The C shims use straightforward memory writes, but compiler ABI, volatile access, and function call overhead raise cycle counts.

- ADC: The measured ADC routines trigger conversion via ADC1 registers and poll the EOC flag. The time measured here only covers the register write + poll loop overhead until EOC — if EOC is set immediately (short sampling), the difference is small (single-digit cycles). The assembly shim wins by a small margin.

- Ultrasonic: The measured times mainly reflect the echo travel time (physical distance) plus the trigger/wait code path. Where the C ultrasonic test showed very large avg and max values, that run included timeouts (no echo) which inflated averages. After ensuring both asm and C measure the same sensor and echoes are present, the comparison should reflect pure code overhead (asm win expected by small constant cycles) plus the echo-dependent component.

**Caveats & reproducibility**

- Ultrasonic measurements are not pure CPU-bound timings — they include real-world echo delays. To compare code overhead you must either capture an echo (use a close reflector) or mock the echo line (toggle input pin) so the measured time is dominated by the code path rather than the distance.
- The current C ultrasonic results in this file came from a run where echoes were inconsistent → observed large avg and max due to timeouts. We patched the code to measure the same sensor for both asm and C; please re-run `perf_test()` after placing a reflector and paste the new PERF output to update the table.

**Recommendations**

- For minimal, latency-critical register writes (GPIO, TIM CCR), keep the hotspot in assembly if you need the absolute lowest cycles and the code is stable. Assembly gives ~2× speed advantage for single-register writes in these measurements.
- For maintainability and portability, prefer C for most code. Use inline assembly or small asm helpers only for micro-optimised primitives when the measured benefit justifies the maintenance cost.
- For ADC reads, the benefit is modest; using register-level C (as implemented) is acceptable unless the extra ~20 cycles matters for a real-time deadline.
- For ultrasonic timing, isolate hardware-dependent delays from code overhead when profiling: use a local loopback or reflector to get stable echo times.

**How to reproduce**

1. Build & flash in STM32CubeIDE (project default settings).
2. Connect serial console at 115200 8N1 (USART2) to capture output.
3. Power the board; ensure `HAL_ADC_Start(&hadc1)` is executed before `perf_test()` (current `main.c` does this).
4. Place a reflector ~20–50 cm in front of the ultrasonic sensor under test (US1) or toggle the echo input pin with a scope/logic tool for deterministic timing.
5. Observe perf output printed at boot; capture lines starting with `PERF:` and the per-iteration prints.

**Next steps**

- Re-run `perf_test()` after ensuring a stable ultrasonic echo and paste the output; I will update the table with the corrected C ultrasonic timings and produce a short summary with final recommendations on whether to keep assembly hotspots or convert them to C.

---

Generated by the ADAS perf harness — file: `Core/Src/main.c` perf_test output.
