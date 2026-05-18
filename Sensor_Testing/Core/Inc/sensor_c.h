/* C equivalents to sensor_asm.s routines (register-level, inline-friendly)
 * Use these to compare performance with the assembly implementations.
 */
#ifndef SENSOR_C_H
#define SENSOR_C_H

#include <stdint.h>

/* Expose same signatures as assembly but prefixed with c_ */
void c_dwt_init(void);
void c_delay_us(uint32_t us);
void c_gpio_set(uint32_t port_base, uint32_t pin_mask);
void c_gpio_reset(uint32_t port_base, uint32_t pin_mask);
void c_tim_set_ccr(uint32_t tim_base, uint32_t ccr_off, uint32_t val);
uint32_t c_ultrasonic_ticks(uint32_t tp, uint32_t t_pin, uint32_t ep, uint32_t e_pin);
uint16_t c_adc1_read(void);

/* Timing constants (match assembly) */
#define CYCLES_PER_US      8u
#define TRIG_LOW_CYCLES    16u
#define TRIG_HIGH_CYCLES   80u
#define ECHO_TIMEOUT       200000u
#define ADC_TIMEOUT_ITER   50000u

#endif /* SENSOR_C_H */
