/**
 * sensor_asm.s  —  ARM Cortex-M3 assembly drivers for STM32F103
 *
 * Exports (callable from C via extern declarations):
 *   asm_dwt_init()
 *   asm_delay_us(uint32_t us)
 *   asm_ultrasonic_ticks(GPIO_TypeDef *tp, uint16_t t_pin,
 *                         GPIO_TypeDef *ep, uint16_t e_pin)  → uint32_t
 *   asm_adc1_read()                                          → uint16_t
 *   asm_gpio_set(uint32_t port_base, uint32_t pin_mask)
 *   asm_gpio_reset(uint32_t port_base, uint32_t pin_mask)
 *   asm_tim_set_ccr(uint32_t tim_base, uint32_t ccr_off, uint32_t val)
 *
 * Hardware assumption: SYSCLK = 8 MHz  (HSI/2 × PLL×2)
 * Change CYCLES_PER_US if the clock is ever raised.
 */

.syntax unified
.cpu    cortex-m3
.thumb

/* ---- Cortex-M3 debug / DWT ------------------------------------------- */
.equ DEMCR,              0xE000EDFC   /* CoreDebug: enables trace unit      */
.equ DWT_CTRL,           0xE0001000   /* DWT control – bit0 = CYCCNTENA     */
.equ DWT_CYCCNT,         0xE0001004   /* Free-running 32-bit cycle counter  */

/* ---- GPIO register offsets (same for all GPIO ports) ----------------- */
.equ GPIO_IDR_OFF,       0x08         /* Input  data register               */
.equ GPIO_BSRR_OFF,      0x10         /* Bit set/reset (bits[15:0] = set)   */
.equ GPIO_BRR_OFF,       0x14         /* Bit reset register                 */

/* ---- ADC1 (base 0x40012400) ------------------------------------------ */
.equ ADC1_BASE,          0x40012400
.equ ADC_SR_OFF,         0x00         /* status:  bit1 = EOC                */
.equ ADC_CR2_OFF,        0x08         /* ctrl2:   bit0=ADON                 */
                                       /*          bit20=EXTTRIG             */
                                       /*          bit22=SWSTART             */
.equ ADC_DR_OFF,         0x4C         /* data register (lower 12 bits)      */

/* ---- Timing constants ------------------------------------------------- */
.equ CYCLES_PER_US,      8            /* 8 MHz → 8 cycles per µs            */
.equ TRIG_LOW_CYCLES,    16           /* 2 µs  × 8                          */
.equ TRIG_HIGH_CYCLES,   80           /* 10 µs × 8                          */
.equ ECHO_TIMEOUT,       200000       /* ~25 ms max wait at 8 MHz           */
.equ ADC_TIMEOUT_ITER,   50000        /* poll iterations before giving up   */


/* ========================================================================
   asm_dwt_init — enable DWT cycle counter (call once at startup)
   No arguments.  Preserves all registers.
   ======================================================================== */
.global asm_dwt_init
.thumb_func
asm_dwt_init:
    push    {r0, r1, lr}

    /* CoreDebug->DEMCR |= TRCENA (bit 24) */
    ldr     r0, =DEMCR
    ldr     r1, [r0]
    orr     r1, r1, #(1 << 24)
    str     r1, [r0]

    /* DWT->CTRL |= CYCCNTENA (bit 0) */
    ldr     r0, =DWT_CTRL
    ldr     r1, [r0]
    orr     r1, r1, #1
    str     r1, [r0]

    /* DWT->CYCCNT = 0 */
    ldr     r0, =DWT_CYCCNT
    mov     r1, #0
    str     r1, [r0]

    pop     {r0, r1, pc}


/* ========================================================================
   asm_delay_us — busy-wait microsecond delay using DWT cycle counter
   r0 = microseconds to wait (must be > 0)
   ======================================================================== */
.global asm_delay_us
.thumb_func
asm_delay_us:
    push    {r1, r2, r3, lr}

    ldr     r1, =DWT_CYCCNT
    ldr     r2, [r1]                   /* r2 = start cycle count            */
    ldr     r3, =CYCLES_PER_US
    mul     r0, r0, r3                 /* r0 = total cycles to wait         */

.du_spin:
    ldr     r3, [r1]                   /* current CYCCNT                    */
    sub     r3, r3, r2                 /* elapsed cycles                    */
    cmp     r3, r0
    blo     .du_spin

    pop     {r1, r2, r3, pc}


/* ========================================================================
   asm_gpio_set — write pin mask to BSRR (sets pin HIGH)
   r0 = port base address (e.g. 0x40010800 for GPIOA)
   r1 = pin mask         (e.g. 0x0010 for GPIO_PIN_4)
   ======================================================================== */
.global asm_gpio_set
.thumb_func
asm_gpio_set:
    str     r1, [r0, #GPIO_BSRR_OFF]
    bx      lr


/* ========================================================================
   asm_gpio_reset — write pin mask to BRR (sets pin LOW)
   r0 = port base address
   r1 = pin mask
   ======================================================================== */
.global asm_gpio_reset
.thumb_func
asm_gpio_reset:
    str     r1, [r0, #GPIO_BRR_OFF]
    bx      lr


/* ========================================================================
   asm_tim_set_ccr — write a value to a timer capture/compare register
   r0 = timer base address  (e.g. (uint32_t)TIM3 = 0x40000400)
   r1 = CCR register offset (0x34=CCR1, 0x38=CCR2, 0x3C=CCR3, 0x40=CCR4)
   r2 = compare value
   ======================================================================== */
.global asm_tim_set_ccr
.thumb_func
asm_tim_set_ccr:
    str     r2, [r0, r1]
    bx      lr


/* ========================================================================
   asm_ultrasonic_ticks — drive TRIG pulse, time the ECHO pulse width.
   Uses direct register I/O for sub-microsecond accuracy.

   r0 = tp    : TRIG port base address
   r1 = t_pin : TRIG pin mask
   r2 = ep    : ECHO port base address
   r3 = e_pin : ECHO pin mask

   Returns r0 = DWT cycle count of echo high pulse.
                0 means timeout (no echo / out of range).
   ======================================================================== */
.global asm_ultrasonic_ticks
.thumb_func
asm_ultrasonic_ticks:
    push    {r4, r5, r6, r7, r8, r9, lr}

    /* Save arguments into callee-saved registers */
    mov     r4, r0                     /* r4 = TRIG port                    */
    mov     r5, r1                     /* r5 = TRIG pin                     */
    mov     r6, r2                     /* r6 = ECHO port                    */
    mov     r7, r3                     /* r7 = ECHO pin                     */
    ldr     r8, =DWT_CYCCNT            /* r8 = &DWT_CYCCNT (constant)       */

    /* ---- Step 1: TRIG LOW for 2 µs ------------------------------------ */
    str     r5, [r4, #GPIO_BRR_OFF]    /* pull TRIG low                     */
    ldr     r9, [r8]                   /* snapshot cycle counter            */
    ldr     r0, =TRIG_LOW_CYCLES
.trig_lo:
    ldr     r1, [r8]
    sub     r1, r1, r9
    cmp     r1, r0
    blo     .trig_lo

    /* ---- Step 2: TRIG HIGH for 10 µs ---------------------------------- */
    str     r5, [r4, #GPIO_BSRR_OFF]   /* drive TRIG high                   */
    ldr     r9, [r8]
    ldr     r0, =TRIG_HIGH_CYCLES
.trig_hi:
    ldr     r1, [r8]
    sub     r1, r1, r9
    cmp     r1, r0
    blo     .trig_hi

    /* ---- Step 3: TRIG LOW -------------------------------------------- */
    str     r5, [r4, #GPIO_BRR_OFF]

    /* ---- Step 4: wait for ECHO to go HIGH (with cycle timeout) -------- */
    ldr     r9, [r8]                   /* r9 = timeout start                */
    ldr     r0, =ECHO_TIMEOUT
.wait_hi:
    ldr     r1, [r6, #GPIO_IDR_OFF]    /* read ECHO port IDR                */
    tst     r1, r7                     /* test echo pin bit                 */
    bne     .echo_rose
    ldr     r1, [r8]
    sub     r1, r1, r9                 /* elapsed                           */
    cmp     r1, r0
    bhs     .us_tout
    b       .wait_hi

    /* ---- Step 5: record echo-rise timestamp --------------------------- */
.echo_rose:
    ldr     r9, [r8]                   /* r9 = echo-rise CYCCNT             */

    /* ---- Step 6: wait for ECHO to go LOW (with cycle timeout) --------- */
    ldr     r0, =ECHO_TIMEOUT
.wait_lo:
    ldr     r1, [r6, #GPIO_IDR_OFF]
    tst     r1, r7
    beq     .echo_fell
    ldr     r1, [r8]
    sub     r1, r1, r9                 /* elapsed since echo rose           */
    cmp     r1, r0
    bhs     .us_tout
    b       .wait_lo

    /* ---- Step 7: compute pulse width ---------------------------------- */
.echo_fell:
    ldr     r0, [r8]
    sub     r0, r0, r9                 /* r0 = echo pulse in DWT cycles     */
    pop     {r4, r5, r6, r7, r8, r9, pc}

.us_tout:
    mov     r0, #0                     /* 0 → timeout                       */
    pop     {r4, r5, r6, r7, r8, r9, pc}


/* ========================================================================
   asm_adc1_read — software-trigger ADC1, return 12-bit result.
   ADC1 must be pre-initialised (HAL_ADC_Init) with ch14 / PC4 selected.
   No arguments.
   Returns r0 = 12-bit sample value, or 0xFFFF on error/timeout.
   ======================================================================== */
.global asm_adc1_read
.thumb_func
asm_adc1_read:
    push    {r1, r2, r3, r4, lr}

    ldr     r4, =ADC1_BASE

    /* Verify ADC is powered (ADON bit 0 in CR2) */
    ldr     r1, [r4, #ADC_CR2_OFF]
    tst     r1, #1
    beq     .adc_fail

    /* Clear EOC flag in SR before starting */
    ldr     r1, [r4, #ADC_SR_OFF]
    bic     r1, r1, #2                 /* clear bit1 = EOC                  */
    str     r1, [r4, #ADC_SR_OFF]

    /* Set EXTTRIG (bit 20) + SWSTART (bit 22) to start conversion */
    ldr     r1, [r4, #ADC_CR2_OFF]
    orr     r1, r1, #(1 << 20)         /* EXTTRIG                           */
    orr     r1, r1, #(1 << 22)         /* SWSTART                           */
    str     r1, [r4, #ADC_CR2_OFF]

    /* Poll EOC with iteration timeout */
    ldr     r3, =ADC_TIMEOUT_ITER
.adc_wait:
    ldr     r1, [r4, #ADC_SR_OFF]
    tst     r1, #2                     /* EOC bit?                          */
    bne     .adc_ok
    subs    r3, r3, #1
    bne     .adc_wait

.adc_fail:
    ldr     r0, =0xFFFF
    pop     {r1, r2, r3, r4, pc}

.adc_ok:
    ldrh    r0, [r4, #ADC_DR_OFF]      /* 16-bit read — lower 12 valid      */
    pop     {r1, r2, r3, r4, pc}

.end
