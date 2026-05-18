Benchmarking the C equivalents vs assembly
-----------------------------------------

Files added:
- Core/Inc/sensor_c.h
- Core/Src/sensor_c.c
- Core/Src/sensor_bench.c

How to use
1. Build the project as usual (ensure the new files are added to the build):

   - Add `sensor_c.c` and `sensor_bench.c` to your STM32 project sources.
   - Keep the existing `sensor_asm.s` assembled and linked so `asm_*` symbols are available.

2. In your application, call `bench_run(results)` where `results` is a `uint32_t[8]` array.
   - After the call, inspect/print the `results` array. Indices:
     - 0: cycles for `c_dwt_init`
     - 1: cycles for `asm_dwt_init`
     - 2: cycles for `c_delay_us(1)`
     - 3: cycles for `asm_delay_us(1)`
     - 4: cycles for `c_gpio_set(dummy)`
     - 5: cycles for `asm_gpio_set(dummy)`
     - 6: placeholder for ultrasonic (run manually with sensors)
     - 7: placeholder for ADC (run manually with ADC configured)

3. To compare ultrasonic and ADC precisely, call `c_ultrasonic_ticks(...)` and
   `asm_ultrasonic_ticks(...)` manually when the hardware is wired and store cycle
   results (the functions return cycle counts or 0 for timeout).

Build notes
- Use `-O3` when compiling C to let the compiler optimize away overhead and
  produce code comparable to hand-written assembly.
- If using HAL, disable heavy wrapper calls for the tested paths; use register-level
  access for fair comparison.

Example (STM32CubeIDE):
- Add the .c files to `Core/Src` and the header to `Core/Inc` in the project tree.
- Rebuild and call `bench_run` from `main()` after peripherals and UART (for printing)
  are initialized.
