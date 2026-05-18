/* Simple DWT-based microbench harness to compare assembly vs C implementations.

   Usage: call `bench_run()` from your project (or compile as a test binary).
   It prints cycle counts if you have a serial/stdout mechanism wired in.
   Alternatively, read results via a debugger watch of the results array.
 */

#include "sensor_c.h"
#include <stdint.h>

/* extern assembly functions (present in sensor_asm.s) */
extern void asm_dwt_init(void);
extern void asm_delay_us(uint32_t us);
extern void asm_gpio_set(uint32_t port_base, uint32_t pin_mask);
extern void asm_gpio_reset(uint32_t port_base, uint32_t pin_mask);
extern void asm_tim_set_ccr(uint32_t tim_base, uint32_t ccr_off, uint32_t val);
extern uint32_t asm_ultrasonic_ticks(uint32_t tp, uint32_t t_pin, uint32_t ep, uint32_t e_pin);
extern uint16_t asm_adc1_read(void);

/* DWT register */
#define DWT_CYCCNT (*(volatile uint32_t*)0xE0001004u)

/* small helper to measure a function (single call) */
static inline uint32_t measure_call(void (*fn)(void))
{
    uint32_t t0 = DWT_CYCCNT;
    fn();
    return DWT_CYCCNT - t0;
}

/* Bench runner — fills results in the provided array. Caller may print them. */
void bench_run(uint32_t results[8])
{
    /* Ensure DWT running */
    asm_dwt_init();

    /* 0: c_dwt_init vs asm_dwt_init */
    results[0] = measure_call((void(*)(void))c_dwt_init);
    results[1] = measure_call((void(*)(void))asm_dwt_init);

    /* 1: delay_us minimal call (measure small us value) */
    uint32_t t0 = DWT_CYCCNT; c_delay_us(1); results[2] = DWT_CYCCNT - t0;
    t0 = DWT_CYCCNT; asm_delay_us(1); results[3] = DWT_CYCCNT - t0;

    /* 2: gpio set/reset (dummy base/pin — use real port when running) */
    uint32_t dummy_port = 0x40010800u; uint32_t dummy_pin = 0x0010u;
    t0 = DWT_CYCCNT; c_gpio_set(dummy_port, dummy_pin); results[4] = DWT_CYCCNT - t0;
    t0 = DWT_CYCCNT; asm_gpio_set(dummy_port, dummy_pin); results[5] = DWT_CYCCNT - t0;

    /* 3: ultrasonic and ADC are hardware-bound; measure only if hardware connected.
       provide placeholders (0) otherwise. */
    results[6] = 0u; /* ultrasonic: run manually when sensor attached */
    results[7] = 0u; /* adc: run manually when ADC configured */
}
