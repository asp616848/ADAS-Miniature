/* Register-level C equivalents for sensor_asm.s
 * Keep semantics identical to assembly; designed for benchmarking.
 */

#include "../../Inc/sensor_c.h"
#include <stdint.h>

/* Core / DWT registers */
#define DEMCR       (*(volatile uint32_t*)0xE000EDFCu)
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000u)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004u)

/* GPIO offsets */
#define GPIO_IDR_OFF    0x08u
#define GPIO_BSRR_OFF   0x10u
#define GPIO_BRR_OFF    0x14u

/* ADC1 */
#define ADC1_BASE       0x40012400u
#define ADC_SR_OFF      0x00u
#define ADC_CR2_OFF     0x08u
#define ADC_DR_OFF      0x4Cu

void c_dwt_init(void)
{
    /* enable trace */
    DEMCR |= (1u << 24);
    DWT_CTRL |= 1u; /* enable cycle counter */
    DWT_CYCCNT = 0u;
}

void c_delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t total = us * CYCLES_PER_US;
    while ((DWT_CYCCNT - start) < total) { }
}

void c_gpio_set(uint32_t port_base, uint32_t pin_mask)
{
    volatile uint32_t *bsrr = (volatile uint32_t*)(port_base + GPIO_BSRR_OFF);
    *bsrr = pin_mask;
}

void c_gpio_reset(uint32_t port_base, uint32_t pin_mask)
{
    volatile uint32_t *brr = (volatile uint32_t*)(port_base + GPIO_BRR_OFF);
    *brr = pin_mask;
}

void c_tim_set_ccr(uint32_t tim_base, uint32_t ccr_off, uint32_t val)
{
    volatile uint32_t *ccr = (volatile uint32_t*)(tim_base + ccr_off);
    *ccr = val;
}

uint32_t c_ultrasonic_ticks(uint32_t tp, uint32_t t_pin, uint32_t ep, uint32_t e_pin)
{
    volatile uint32_t *trig_brr = (volatile uint32_t*)(tp + GPIO_BRR_OFF);
    volatile uint32_t *trig_bsrr = (volatile uint32_t*)(tp + GPIO_BSRR_OFF);
    volatile uint32_t *echo_idr = (volatile uint32_t*)(ep + GPIO_IDR_OFF);
    uint32_t t0, timeout_start, elapsed;

    /* TRIG low 2us */
    *trig_brr = t_pin;
    timeout_start = DWT_CYCCNT;
    while ((DWT_CYCCNT - timeout_start) < TRIG_LOW_CYCLES) { }

    /* TRIG high 10us */
    *trig_bsrr = t_pin;
    timeout_start = DWT_CYCCNT;
    while ((DWT_CYCCNT - timeout_start) < TRIG_HIGH_CYCLES) { }

    /* TRIG low */
    *trig_brr = t_pin;

    /* wait for echo high */
    timeout_start = DWT_CYCCNT;
    while (((*echo_idr & e_pin) == 0u)) {
        if ((DWT_CYCCNT - timeout_start) >= ECHO_TIMEOUT) return 0u;
    }
    t0 = DWT_CYCCNT;

    /* wait for echo low */
    timeout_start = DWT_CYCCNT;
    while (((*echo_idr & e_pin) != 0u)) {
        if ((DWT_CYCCNT - t0) >= ECHO_TIMEOUT) return 0u;
    }

    elapsed = DWT_CYCCNT - t0;
    return elapsed;
}

uint16_t c_adc1_read(void)
{
    volatile uint32_t *adc_base = (volatile uint32_t*)ADC1_BASE;
    uint32_t sr = adc_base[ADC_SR_OFF/4];

    /* check ADON */
    uint32_t cr2 = adc_base[ADC_CR2_OFF/4];
    if ((cr2 & 1u) == 0u) return 0xFFFFu;

    /* clear EOC */
    adc_base[ADC_SR_OFF/4] = (sr & ~2u);

    /* start conversion by setting EXTTRIG + SWSTART bits */
    cr2 = adc_base[ADC_CR2_OFF/4];
    cr2 |= (1u << 20);
    cr2 |= (1u << 22);
    adc_base[ADC_CR2_OFF/4] = cr2;

    uint32_t iter = ADC_TIMEOUT_ITER;
    while (((adc_base[ADC_SR_OFF/4] & 2u) == 0u) && (iter--)) { }
    if ((adc_base[ADC_SR_OFF/4] & 2u) == 0u) return 0xFFFFu;

    uint16_t val = *((volatile uint16_t*)((uint8_t*)adc_base + ADC_DR_OFF));
    return val;
}
