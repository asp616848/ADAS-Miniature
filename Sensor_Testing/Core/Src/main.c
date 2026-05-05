/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   ADAS Miniature — always-moving obstacle avoidance, STM32F103RB
  *
  * Sensor layout (bird's-eye, FWD = up):
  *
  *          US3(L) ←─── [BOT] ───→ US1(R)
  *          US4(FL)↗               ↖US2(FR)
  *                      [FWD ↑]
  *          US6(RL)↙               ↘US5(RR)
  *
  * Motor 1 = RIGHT wheel  (TIM3_CH1/PA6 speed, PB14/IN1, PB15/IN2)
  * Motor 2 = LEFT  wheel  (PB13/ENB on-off,    PC7/IN3,  PC8/IN4 )
  *
  * Key constraints
  *   • Robot NEVER fully stops — always in motion or turning.
  *   • Robot NEVER collides   — multi-layer threshold logic.
  *
  * Manoeuvre vocabulary
  *   PIVOT_LEFT  : right-fwd  + left-back  (CCW spin, turns vehicle left)
  *   PIVOT_RIGHT : right-back + left-fwd   (CW  spin, turns vehicle right)
  *   REVERSE     : both wheels back; transitions to a big pivot (U-turn)
  *   FORWARD     : variable speed; gentle steering near side walls
  *
  * Servo  : sweeps freely when open; freezes toward obstacle / turn dir.
  * Alerts : LED1/LED2/buzzer encode current state non-blocking.
  *
  * Assembly drivers (sensor_asm.s):
  *   asm_dwt_init, asm_delay_us, asm_ultrasonic_ticks, asm_adc1_read,
  *   asm_gpio_set, asm_gpio_reset, asm_tim_set_ccr
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* =====================================================================
   ASSEMBLY FUNCTION DECLARATIONS
   ===================================================================== */
extern void     asm_dwt_init(void);
extern void     asm_delay_us(uint32_t us);
extern uint32_t asm_ultrasonic_ticks(GPIO_TypeDef *tp, uint16_t t_pin,
                                      GPIO_TypeDef *ep, uint16_t e_pin);
extern uint16_t asm_adc1_read(void);
extern void     asm_gpio_set  (uint32_t port_base, uint32_t pin_mask);
extern void     asm_gpio_reset(uint32_t port_base, uint32_t pin_mask);
extern void     asm_tim_set_ccr(uint32_t tim_base, uint32_t ccr_off,
                                 uint32_t val);

/* =====================================================================
   DEFINES — sensor ranges
   ===================================================================== */
#define US_MIN_CM            2.0f
#define US_MAX_CM          300.0f
#define US_GAP_MS           25U      /* inter-sensor crosstalk guard        */
#define ADC_TIMEOUT_MS      10U
#define IR_ALERT_TH       1000U      /* ADC counts below this → object near */

/* Servo (TIM1 CH4) --------------------------------------------------- */
#define SERVO_LEFT_US      1000U
#define SERVO_CENTER_US    1500U
#define SERVO_RIGHT_US     2000U
#define SERVO_HALF_MS      2500U     /* half-sweep period (left→right 2.5 s)*/

/* TIM CCR register offsets ------------------------------------------- */
#define TIM_CCR1_OFF        0x34U
#define TIM_CCR2_OFF        0x38U
#define TIM_CCR4_OFF        0x40U

/* Motor pin masks / port addresses ----------------------------------- */
/* Motor 1 (RIGHT) */
#define M1_IN1_BASE  ((uint32_t)GPIOB)
#define M1_IN1_PIN   GPIO_PIN_14
#define M1_IN2_BASE  ((uint32_t)GPIOB)
#define M1_IN2_PIN   GPIO_PIN_15

/* Motor 2 (LEFT) */
#define M2_ENB_BASE  ((uint32_t)GPIOB)
#define M2_ENB_PIN   GPIO_PIN_13
#define M2_IN3_BASE  ((uint32_t)GPIOC)
#define M2_IN3_PIN   GPIO_PIN_7
#define M2_IN4_BASE  ((uint32_t)GPIOC)
#define M2_IN4_PIN   GPIO_PIN_8

/* Motor 1 duty cycle limits (0 – 999) */
#define DUTY_FULL    900U
#define DUTY_HALF    600U
#define DUTY_SLOW    320U

/* Navigation thresholds (cm) */
#define NAV_CRITICAL_CM      8.0f   /* immediate emergency pivot            */
#define NAV_FRONT_STOP_CM   30.0f   /* must turn                            */
#define NAV_FRONT_WARN_CM   45.0f   /* slow down / servo freeze             */
#define NAV_SIDE_WARN_CM    15.0f   /* gentle wall steer                    */
#define NAV_REAR_STOP_CM    20.0f   /* abort reverse                        */

/* Navigation timing (ms) — tune these four to match your robot's speed */
#define PIVOT_90_MS        1100U    /* ~90°  in-place pivot                 */
#define PIVOT_180_MS       2200U    /* ~180° U-turn pivot                   */
#define PIVOT_DODGE_MS      500U    /* short IR-dodge pivot                 */
#define PIVOT_MIN_MS        350U    /* earliest permissible early exit      */
#define BACKUP_MS           900U    /* reverse duration before big pivot    */

/* Ultrasonic sensor port/pin groups ---------------------------------- */
#define US1_TRIG_PORT  GPIOC
#define US1_TRIG_PIN   GPIO_PIN_0
#define US1_ECHO_PORT  GPIOA
#define US1_ECHO_PIN   GPIO_PIN_4

#define US2_TRIG_PORT  GPIOC
#define US2_TRIG_PIN   GPIO_PIN_1
#define US2_ECHO_PORT  GPIOC
#define US2_ECHO_PIN   GPIO_PIN_2

#define US3_TRIG_PORT  GPIOC
#define US3_TRIG_PIN   GPIO_PIN_3
#define US3_ECHO_PORT  GPIOC
#define US3_ECHO_PIN   GPIO_PIN_6

#define US4_TRIG_PORT  GPIOC
#define US4_TRIG_PIN   GPIO_PIN_5
#define US4_ECHO_PORT  GPIOA
#define US4_ECHO_PIN   GPIO_PIN_5

#define US5_TRIG_PORT  GPIOB
#define US5_TRIG_PIN   GPIO_PIN_0
#define US5_ECHO_PORT  GPIOB
#define US5_ECHO_PIN   GPIO_PIN_1

#define US6_TRIG_PORT  GPIOB
#define US6_TRIG_PIN   GPIO_PIN_10
#define US6_ECHO_PORT  GPIOB
#define US6_ECHO_PIN   GPIO_PIN_11

/* Output pins */
#define BUZZER_BASE      ((uint32_t)GPIOB)
#define BUZZER_PIN       GPIO_PIN_8
#define LED1_BASE        ((uint32_t)GPIOB)
#define LED1_PIN         GPIO_PIN_9
#define LED2_BASE        ((uint32_t)GPIOB)
#define LED2_PIN         GPIO_PIN_12

/* =====================================================================
   HAL PERIPHERAL HANDLES
   ===================================================================== */
ADC_HandleTypeDef   hadc1;
I2C_HandleTypeDef   hi2c1;
TIM_HandleTypeDef   htim1;
TIM_HandleTypeDef   htim3;
UART_HandleTypeDef  huart2;

/* =====================================================================
   NAVIGATION STATE
   ===================================================================== */
typedef enum {
    NAV_FORWARD     = 0,
    NAV_PIVOT_LEFT,          /* CCW spin: right-fwd, left-back             */
    NAV_PIVOT_RIGHT,         /* CW  spin: right-back, left-fwd             */
    NAV_REVERSE,             /* back up then big pivot (U-turn path)       */
} NavState_t;

static NavState_t g_nav           = NAV_FORWARD;
static uint32_t   g_nav_start     = 0U;
static uint32_t   g_pivot_ms      = PIVOT_90_MS; /* target pivot duration  */
static uint8_t    g_post_rev_dir  = 0U;           /* 0=L 1=R after reverse  */
static uint32_t   g_slow_until    = 0U;           /* keep slow until this time */

/* Shared with gap_delay for real-time alerts */
static volatile float    g_us5 = -1.0f;
static volatile float    g_us6 = -1.0f;
static volatile uint16_t g_ir  = 0xFFFFU;

/* =====================================================================
   PRIVATE FUNCTION PROTOTYPES
   ===================================================================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* =====================================================================
   PRINTF RETARGET → USART2
   ===================================================================== */
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 100U);
    return len;
}

/* =====================================================================
   SENSOR WRAPPERS  (thin C shims over assembly)
   ===================================================================== */

/* Convert echo pulse (in DWT cycles at 8 MHz) to cm */
static float ultrasonic_read(GPIO_TypeDef *tp, uint16_t t_pin,
                              GPIO_TypeDef *ep, uint16_t e_pin)
{
    uint32_t ticks = asm_ultrasonic_ticks(tp, t_pin, ep, e_pin);
    if (ticks == 0U) return -1.0f;
    /* At 8 MHz: 1 cycle = 0.125 µs; distance = (cycles/8 µs) × 0.0343/2 cm */
    float dt_us = (float)ticks / 8.0f;          /* µs                       */
    float cm    = (dt_us * 0.0343f) / 2.0f;
    return ((cm < US_MIN_CM) || (cm > US_MAX_CM)) ? -1.0f : cm;
}

/* ADC read: returns raw 12-bit count or 0xFFFF on error */
static uint16_t read_ir(void)
{
    return asm_adc1_read();
}

/* =====================================================================
   NAVIGATION HELPER
   ===================================================================== */
/* Treat invalid readings as "very far" so they don't falsely block nav */
static float nav_cm(float raw) { return (raw < US_MIN_CM) ? 999.0f : raw; }

static void nav_set(NavState_t s)
{
    g_nav       = s;
    g_nav_start = HAL_GetTick();
}

static uint32_t nav_elapsed(void) { return HAL_GetTick() - g_nav_start; }

/* =====================================================================
   MOTOR PRIMITIVES  (use assembly GPIO / timer writes)
   ===================================================================== */

/* Motor 1 (RIGHT) — speed via TIM3 CCR1 on PA6 */
static void m1_set_duty(uint32_t d)
{
    if (d > 999U) d = 999U;
    asm_tim_set_ccr((uint32_t)TIM3, TIM_CCR1_OFF, d);
    asm_tim_set_ccr((uint32_t)TIM3, TIM_CCR2_OFF, 0U); /* PA7 keep silent */
}

static void m1_forward(uint32_t duty)
{
    asm_gpio_set  (M1_IN1_BASE, M1_IN1_PIN);   /* IN1 high                  */
    asm_gpio_reset(M1_IN2_BASE, M1_IN2_PIN);   /* IN2 low                   */
    m1_set_duty(duty);
}

static void m1_backward(uint32_t duty)
{
    asm_gpio_reset(M1_IN1_BASE, M1_IN1_PIN);   /* IN1 low                   */
    asm_gpio_set  (M1_IN2_BASE, M1_IN2_PIN);   /* IN2 high                  */
    m1_set_duty(duty);
}

static void m1_stop(void) { m1_set_duty(0U); }

/* Motor 2 (LEFT) — GPIO on/off only (no PWM) */
static void m2_forward(void)
{
    asm_gpio_reset(M2_IN3_BASE, M2_IN3_PIN);   /* IN3 low                   */
    asm_gpio_set  (M2_IN4_BASE, M2_IN4_PIN);   /* IN4 high                  */
    asm_gpio_set  (M2_ENB_BASE, M2_ENB_PIN);   /* ENB on                    */
}

static void m2_backward(void)
{
    asm_gpio_set  (M2_IN3_BASE, M2_IN3_PIN);   /* IN3 high                  */
    asm_gpio_reset(M2_IN4_BASE, M2_IN4_PIN);   /* IN4 low                   */
    asm_gpio_set  (M2_ENB_BASE, M2_ENB_PIN);   /* ENB on                    */
}

static void m2_stop(void)
{
    asm_gpio_reset(M2_ENB_BASE, M2_ENB_PIN);   /* ENB off → coasts          */
}

/* ---- Composite commands -------------------------------------------- */
static void motors_forward (uint32_t d){ m1_forward(d);         m2_forward();  }
static void motors_backward(uint32_t d){ m1_backward(d);        m2_backward(); }
static void motors_stop    (void)       { m1_stop();             m2_stop();     }

/* PIVOT LEFT  (CCW): right-fwd + left-back */
static void motors_pivot_left (void) { m1_forward (DUTY_FULL); m2_backward(); }
/* PIVOT RIGHT (CW):  right-back + left-fwd */
static void motors_pivot_right(void) { m1_backward(DUTY_FULL); m2_forward();  }

/* Gentle steer (one wheel slows, other full) */
static void motors_steer_left (void) { m1_forward(DUTY_SLOW);  m2_forward();  }
static void motors_steer_right(void) { m1_forward(DUTY_FULL);  m2_stop();     }

/* =====================================================================
   SERVO — context-aware position / sweep
   ===================================================================== */
static void servo_config_50hz(void)
{
    RCC_ClkInitTypeDef clk = {0};
    uint32_t fl = 0U;
    HAL_RCC_GetClockConfig(&clk, &fl);
    uint32_t pclk2   = HAL_RCC_GetPCLK2Freq();
    uint32_t ppre2   = (clk.APB2CLKDivider == RCC_HCLK_DIV1) ? 1U : 2U;
    uint32_t tim_clk = (ppre2 == 1U) ? pclk2 : (pclk2 * 2U);
    if (tim_clk < 1000000U) return;
    __HAL_TIM_SET_PRESCALER(&htim1, (tim_clk / 1000000U) - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, 20000U - 1U);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
}

static void servo_set_us(uint16_t us)
{
    if (us < SERVO_LEFT_US)  us = SERVO_LEFT_US;
    if (us > SERVO_RIGHT_US) us = SERVO_RIGHT_US;
    asm_tim_set_ccr((uint32_t)TIM1, TIM_CCR4_OFF, (uint32_t)us);
}

/* Full-range ping-pong sweep, period = 2 × SERVO_HALF_MS */
static void servo_do_sweep(void)
{
    uint32_t period = (uint32_t)SERVO_HALF_MS * 2U;
    uint32_t phase  = HAL_GetTick() % period;
    uint16_t us;
    if (phase < (uint32_t)SERVO_HALF_MS) {
        float t = (float)phase / (float)SERVO_HALF_MS;
        us = (uint16_t)((float)SERVO_LEFT_US +
                         t * (float)(SERVO_RIGHT_US - SERVO_LEFT_US));
    } else {
        float t = (float)(phase - (uint32_t)SERVO_HALF_MS) /
                  (float)SERVO_HALF_MS;
        us = (uint16_t)((float)SERVO_RIGHT_US -
                         t * (float)(SERVO_RIGHT_US - SERVO_LEFT_US));
    }
    servo_set_us(us);
}

/*
 * servo_update — call every loop tick.
 * Behaviour per state:
 *   PIVOT_LEFT   → freeze LEFT  (servo "signals" turn direction)
 *   PIVOT_RIGHT  → freeze RIGHT
 *   REVERSE      → freeze CENTER (looking forward while backing)
 *   FORWARD+warn → freeze toward closest front obstacle for 1 s, then sweep
 *   FORWARD+free → full sweep
 */
static void servo_update(NavState_t nav, float s2, float s4)
{
    static uint32_t freeze_until = 0U;
    static uint16_t freeze_pos   = SERVO_CENTER_US;
    uint32_t now = HAL_GetTick();

    switch (nav) {
    case NAV_PIVOT_LEFT:
        freeze_until = now + 120U;
        freeze_pos   = SERVO_LEFT_US;
        servo_set_us(SERVO_LEFT_US);
        return;

    case NAV_PIVOT_RIGHT:
        freeze_until = now + 120U;
        freeze_pos   = SERVO_RIGHT_US;
        servo_set_us(SERVO_RIGHT_US);
        return;

    case NAV_REVERSE:
        servo_set_us(SERVO_CENTER_US);
        return;

    case NAV_FORWARD:
    default:
        {
            float f2 = nav_cm(s2);   /* front-right distance */
            float f4 = nav_cm(s4);   /* front-left  distance */

            if ((f2 < NAV_FRONT_WARN_CM) || (f4 < NAV_FRONT_WARN_CM)) {
                /* Obstacle approaching — freeze servo toward the blocker  */
                uint16_t target;
                if ((f2 < NAV_FRONT_WARN_CM) && (f4 < NAV_FRONT_WARN_CM)) {
                    target = SERVO_CENTER_US;    /* both sides blocked       */
                } else if (f2 < f4) {
                    target = SERVO_RIGHT_US;     /* right-front closer       */
                } else {
                    target = SERVO_LEFT_US;      /* left-front  closer       */
                }
                /* Refresh freeze for 1 s every call while obstacle present */
                freeze_until = now + 1000U;
                freeze_pos   = target;
                servo_set_us(target);
                return;
            }

            /* No obstacle — resume sweep if freeze expired */
            if (now < freeze_until) {
                servo_set_us(freeze_pos);
            } else {
                servo_do_sweep();
            }
        }
        break;
    }
}

/* =====================================================================
   ALERT OUTPUTS — LED1, LED2, buzzer (non-blocking)
   ===================================================================== */
static void alert_update(NavState_t state, float us5_cm, float us6_cm,
                         uint16_t ir_raw)
{
    static uint32_t      last_tog = 0U;
    static uint8_t       led1     = 0U;
    static uint8_t       led2     = 0U;
    static uint8_t       buz      = 0U;

    uint32_t now    = HAL_GetTick();
    uint8_t  ir_on  = (ir_raw != 0xFFFFU) && (ir_raw < IR_ALERT_TH);
    float    s5     = nav_cm(us5_cm);
    float    s6     = nav_cm(us6_cm);
    float    rear   = (s5 < s6) ? s5 : s6;

    /* Priority 1: IR object or critically close rear → solid on */
    if (ir_on || (rear < NAV_CRITICAL_CM)) {
        led1 = led2 = buz = 1U;
        goto apply;
    }

    switch (state) {

    case NAV_FORWARD:
        if (rear < NAV_REAR_STOP_CM) {
            /* Rear proximity: fast blink + buzzer */
            uint32_t half = (uint32_t)(80U + 270U *
                            ((rear - NAV_CRITICAL_CM) /
                             (NAV_REAR_STOP_CM - NAV_CRITICAL_CM)));
            if ((now - last_tog) >= half) {
                led1 = (s5 < NAV_REAR_STOP_CM) ? (uint8_t)(!led1) : 0U;
                led2 = (s6 < NAV_REAR_STOP_CM) ? (uint8_t)(!led2) : 0U;
                buz  = (uint8_t)(led1 | led2);
                last_tog = now;
            }
        } else {
            /* Clear road: slow heartbeat, no buzzer */
            if ((now - last_tog) >= 500U) {
                led1 = led2 = (uint8_t)(!led1);
                buz         = 0U;
                last_tog    = now;
            }
        }
        break;

    case NAV_PIVOT_LEFT:
        /* Left turn indicator: LED1 rapid, LED2 off */
        if ((now - last_tog) >= 100U) {
            led1 = (uint8_t)(!led1);
            led2 = 0U;  buz = 0U;
            last_tog = now;
        }
        break;

    case NAV_PIVOT_RIGHT:
        /* Right turn indicator: LED2 rapid, LED1 off */
        if ((now - last_tog) >= 100U) {
            led2 = (uint8_t)(!led2);
            led1 = 0U;  buz = 0U;
            last_tog = now;
        }
        break;

    case NAV_REVERSE:
        /* Reversing: LEDs alternate + buzzer chirps */
        if ((now - last_tog) >= 130U) {
            led1     = (uint8_t)(!led1);
            led2     = (uint8_t)(!led1);   /* opposite of led1              */
            buz      = led1;
            last_tog = now;
        }
        break;

    default:
        break;
    }

apply:
    if (led1) asm_gpio_set(LED1_BASE, LED1_PIN);
    else       asm_gpio_reset(LED1_BASE, LED1_PIN);
    if (led2) asm_gpio_set(LED2_BASE, LED2_PIN);
    else       asm_gpio_reset(LED2_BASE, LED2_PIN);
    if (buz)  asm_gpio_set(BUZZER_BASE, BUZZER_PIN);
    else       asm_gpio_reset(BUZZER_BASE, BUZZER_PIN);
}

/* =====================================================================
   NAVIGATION STATE MACHINE  — never fully stops the robot
   ===================================================================== */
static void nav_update(float us1, float us2, float us3, float us4,
                       float us5, float us6, uint8_t ir)
{
    /* Safe-convert: invalid echo (-1) → 999 cm (treat as clear) */
    float s1 = nav_cm(us1);   /* RIGHT       */
    float s2 = nav_cm(us2);   /* FRONT-RIGHT */
    float s3 = nav_cm(us3);   /* LEFT        */
    float s4 = nav_cm(us4);   /* FRONT-LEFT  */
    float s5 = nav_cm(us5);   /* REAR-RIGHT  */
    float s6 = nav_cm(us6);   /* REAR-LEFT   */

    uint8_t front_r_blk = (s2 < NAV_FRONT_STOP_CM);
    uint8_t front_l_blk = (s4 < NAV_FRONT_STOP_CM);
    uint8_t front_blk   = front_r_blk | front_l_blk;
    uint8_t front_crit  = (s2 < NAV_CRITICAL_CM) | (s4 < NAV_CRITICAL_CM);
    uint8_t rear_blk    = (s5 < NAV_REAR_STOP_CM) | (s6 < NAV_REAR_STOP_CM);
    uint8_t right_wall  = (s1 < NAV_SIDE_WARN_CM);
    uint8_t left_wall   = (s3 < NAV_SIDE_WARN_CM);

    switch (g_nav) {

    /* ================================================================
       FORWARD — primary running state
       ================================================================ */
    case NAV_FORWARD:

        /* ---- Critical emergency: pivot hard away from blocked sensor ---- */
        if (front_crit) {
            motors_stop();
            if (s2 < s4) {             /* right front closer → go left      */
                g_pivot_ms = PIVOT_90_MS;
                nav_set(NAV_PIVOT_LEFT);
            } else {
                g_pivot_ms = PIVOT_90_MS;
                nav_set(NAV_PIVOT_RIGHT);
            }
            break;
        }

        /* ---- IR object: quick dodge toward the more open side ---------- */
        if (ir) {
            motors_stop();
            g_pivot_ms = PIVOT_DODGE_MS;
            nav_set((s1 >= s3) ? NAV_PIVOT_RIGHT : NAV_PIVOT_LEFT);
            break;
        }

        /* ---- Front blocked: choose 90° turn or reverse for U-turn ----- */
        if (front_blk) {
            motors_stop();
            if (front_r_blk && !front_l_blk) {
                /* Only right-front blocked → 90° left turn */
                g_pivot_ms = PIVOT_90_MS;
                nav_set(NAV_PIVOT_LEFT);
            } else if (front_l_blk && !front_r_blk) {
                /* Only left-front blocked → 90° right turn */
                g_pivot_ms = PIVOT_90_MS;
                nav_set(NAV_PIVOT_RIGHT);
            } else {
                /* Both front sensors blocked */
                float gap = s1 - s3;             /* positive = more R space */
                if (gap >  8.0f) {
                    /* Clearly more room on the right → 90° right           */
                    g_pivot_ms = PIVOT_90_MS;
                    nav_set(NAV_PIVOT_RIGHT);
                } else if (gap < -8.0f) {
                    /* Clearly more room on the left → 90° left             */
                    g_pivot_ms = PIVOT_90_MS;
                    nav_set(NAV_PIVOT_LEFT);
                } else {
                    /*
                     * Symmetric box-in: reverse then execute a 180° pivot.
                     * Pick the post-reverse pivot direction based on which
                     * REAR sensor has more clearance.
                     */
                    g_post_rev_dir = (s5 >= s6) ? 1U : 0U; /* 1=R, 0=L     */
                    nav_set(NAV_REVERSE);
                }
            }
            break;
        }

        /* ---- Side-wall gentle steering --------------------------------- */
        if (right_wall && !left_wall) {
            motors_steer_left();   /* right wall too close → drift left     */
            break;
        }
        if (left_wall && !right_wall) {
            motors_steer_right();  /* left wall too close → drift right     */
            break;
        }

        /* ---- Open road: speed proportional to nearest front sensor ----- */
        {
            float front_min = (s2 < s4) ? s2 : s4;
            uint32_t duty   = DUTY_FULL;
            if (front_min < NAV_FRONT_WARN_CM) {
                float ratio = (front_min - NAV_FRONT_STOP_CM) /
                              (NAV_FRONT_WARN_CM - NAV_FRONT_STOP_CM);
                if (ratio < 0.0f) ratio = 0.0f;
                if (ratio > 1.0f) ratio = 1.0f;
                duty = (uint32_t)(DUTY_SLOW +
                       ratio * (float)(DUTY_FULL - DUTY_SLOW));
            }
            if (HAL_GetTick() < g_slow_until) {
                if (duty > DUTY_SLOW) duty = DUTY_SLOW;
            }
            motors_forward(duty);
        }
        break;

    /* ================================================================
       PIVOT LEFT (CCW): right-fwd, left-back
       Exits early if front clears after minimum time.
       Forced exit at g_pivot_ms → go forward and reassess.
       ================================================================ */
    case NAV_PIVOT_LEFT:
        motors_pivot_left();

        if (nav_elapsed() >= g_pivot_ms) {
            /* Time's up — move forward regardless (bot keeps going) */
            g_slow_until = HAL_GetTick() + 1500U;
            nav_set(NAV_FORWARD);
            break;
        }
        if (!front_blk && (nav_elapsed() >= PIVOT_MIN_MS)) {
            /* Front cleared early → go! */
            g_slow_until = HAL_GetTick() + 1500U;
            nav_set(NAV_FORWARD);
        }
        break;

    /* ================================================================
       PIVOT RIGHT (CW): right-back, left-fwd
       ================================================================ */
    case NAV_PIVOT_RIGHT:
        motors_pivot_right();

        if (nav_elapsed() >= g_pivot_ms) {
            g_slow_until = HAL_GetTick() + 1500U;
            nav_set(NAV_FORWARD);
            break;
        }
        if (!front_blk && (nav_elapsed() >= PIVOT_MIN_MS)) {
            g_slow_until = HAL_GetTick() + 1500U;
            nav_set(NAV_FORWARD);
        }
        break;

    /* ================================================================
       REVERSE — back up, then commit to a big 180° pivot (U-turn).
       If rear gets blocked mid-reverse, abort immediately into pivot.
       ================================================================ */
    case NAV_REVERSE:

        if (rear_blk) {
            /*
             * Something appeared behind us — stop reversing and spin
             * away from the blocked rear sensor.
             *   US5 = rear-right, US6 = rear-left
             * Blocked rear-right → pivot LEFT (spin away from right wall).
             * Blocked rear-left  → pivot RIGHT.
             */
            motors_stop();
            g_pivot_ms = PIVOT_180_MS;
            nav_set((s5 < s6) ? NAV_PIVOT_LEFT : NAV_PIVOT_RIGHT);
            break;
        }

        motors_backward(DUTY_SLOW);

        if (nav_elapsed() >= BACKUP_MS) {
            /* Backup complete — now do the U-turn (180° pivot) */
            g_pivot_ms = PIVOT_180_MS;
            nav_set(g_post_rev_dir ? NAV_PIVOT_RIGHT : NAV_PIVOT_LEFT);
        }
        break;

    default:
        /* Safety net — should never reach here */
        nav_set(NAV_FORWARD);
        break;
    }
}

/* =====================================================================
   GAP DELAY — keeps alerts + servo alive during inter-sensor pauses
   ===================================================================== */
static void gap_delay(uint32_t ms)
{
    uint32_t end = HAL_GetTick() + ms;
    while (HAL_GetTick() < end) {
        alert_update(g_nav, g_us5, g_us6, g_ir);
        servo_update(g_nav, 999.0f, 999.0f); /* use cached front vals      */
        HAL_Delay(4U);
    }
}

/* =====================================================================
   MAIN
   ===================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    /* Assembly DWT init (replaces C DWT_Init) */
    asm_dwt_init();

    /* Servo: configure 50 Hz, center position */
    servo_config_50hz();
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    servo_set_us(SERVO_CENTER_US);

    /* Motor PWM timer: start CH1 and CH2 on TIM3 */
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /* Start immediately driving forward */
    motors_forward(DUTY_SLOW);
    nav_set(NAV_FORWARD);

    printf("BOOT: ADAS always-moving obstacle-avoidance\r\n");

    while (1)
    {
        /* -------- Read IR (assembly ADC) -------- */
        g_ir = read_ir();

        /* -------- Read all 6 ultrasonic sensors (gaps service alerts) --- */
        float us1 = ultrasonic_read(US1_TRIG_PORT, US1_TRIG_PIN,
                                     US1_ECHO_PORT, US1_ECHO_PIN);
        gap_delay(US_GAP_MS);

        float us2 = ultrasonic_read(US2_TRIG_PORT, US2_TRIG_PIN,
                                     US2_ECHO_PORT, US2_ECHO_PIN);
        gap_delay(US_GAP_MS);

        float us3 = ultrasonic_read(US3_TRIG_PORT, US3_TRIG_PIN,
                                     US3_ECHO_PORT, US3_ECHO_PIN);
        gap_delay(US_GAP_MS);

        float us4 = ultrasonic_read(US4_TRIG_PORT, US4_TRIG_PIN,
                                     US4_ECHO_PORT, US4_ECHO_PIN);
        gap_delay(US_GAP_MS);

        float us5 = ultrasonic_read(US5_TRIG_PORT, US5_TRIG_PIN,
                                     US5_ECHO_PORT, US5_ECHO_PIN);
        gap_delay(US_GAP_MS);

        float us6 = ultrasonic_read(US6_TRIG_PORT, US6_TRIG_PIN,
                                     US6_ECHO_PORT, US6_ECHO_PIN);
        gap_delay(US_GAP_MS);

        /* Cache rear readings for gap_delay alert use next iteration */
        g_us5 = us5;
        g_us6 = us6;

        /* -------- Navigation decision -------- */
        uint8_t ir_flag = (uint8_t)((g_ir != 0xFFFFU) && (g_ir < IR_ALERT_TH));
        nav_update(us1, us2, us3, us4, us5, us6, ir_flag);

        /* -------- Alerts & servo -------- */
        alert_update(g_nav, us5, us6, g_ir);
        servo_update(g_nav, us2, us4);

        /* -------- Debug UART (throttled to 500 ms) -------- */
        {
            static uint32_t last_p = 0U;
            uint32_t now = HAL_GetTick();
            if ((now - last_p) >= 500U) {
                static const char *st[] = {"FWD","PIL","PIR","REV"};
                int i1=(us1<0.0f)?-1:(int)(us1*10.0f);
                int i2=(us2<0.0f)?-1:(int)(us2*10.0f);
                int i3=(us3<0.0f)?-1:(int)(us3*10.0f);
                int i4=(us4<0.0f)?-1:(int)(us4*10.0f);
                int i5=(us5<0.0f)?-1:(int)(us5*10.0f);
                int i6=(us6<0.0f)?-1:(int)(us6*10.0f);
                printf("ST=%s IR=%u "
                       "R=%d.%d FR=%d.%d L=%d.%d "
                       "FL=%d.%d RR=%d.%d RL=%d.%d\r\n",
                       st[g_nav], (unsigned int)g_ir,
                       i1/10,(i1<0)?0:(i1%10),
                       i2/10,(i2<0)?0:(i2%10),
                       i3/10,(i3<0)?0:(i3%10),
                       i4/10,(i4<0)?0:(i4%10),
                       i5/10,(i5<0)?0:(i5%10),
                       i6/10,(i6<0)?0:(i6%10));
                last_p = now;
            }
        }
    }
}

/* =====================================================================
   SYSTEM CLOCK   (HSI/2 → PLL×2 = 8 MHz SYSCLK)
   ===================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       osc  = {0};
    RCC_ClkInitTypeDef       clk  = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL          = RCC_PLL_MUL2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) Error_Handler();

    pclk.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    pclk.AdcClockSelection    = RCC_ADCPCLK2_DIV2;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) Error_Handler();
}

/* =====================================================================
   ADC1 — channel 14 (PC4), IR sensor
   ===================================================================== */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sc = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sc.Channel      = ADC_CHANNEL_14;
    sc.Rank         = ADC_REGULAR_RANK_1;
    sc.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sc) != HAL_OK) Error_Handler();
}

/* =====================================================================
   I2C1 — 100 kHz (IMU reserved, unused this build)
   ===================================================================== */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

/* =====================================================================
   TIM1 — 50 Hz PWM for servo (CH4 → PA11)
   Prescaler corrected at runtime by servo_config_50hz().
   ===================================================================== */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef        scc = {0};
    TIM_MasterConfigTypeDef       mc  = {0};
    TIM_OC_InitTypeDef            oc  = {0};
    TIM_BreakDeadTimeConfigTypeDef bd  = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 63;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 19999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    scc.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &scc) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    mc.MasterOutputTrigger = TIM_TRGO_RESET;
    mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &mc) != HAL_OK)
        Error_Handler();

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = SERVO_CENTER_US;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    bd.OffStateRunMode  = TIM_OSSR_DISABLE;
    bd.OffStateIDLEMode = TIM_OSSI_DISABLE;
    bd.LockLevel        = TIM_LOCKLEVEL_OFF;
    bd.DeadTime         = 0;
    bd.BreakState       = TIM_BREAK_DISABLE;
    bd.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    bd.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bd) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim1);
}

/* =====================================================================
   TIM3 — ~111 Hz PWM for Motor 1 speed (CH1 → PA6)
   8 MHz / 72 / 1000 = 111 Hz
   ===================================================================== */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  scc = {0};
    TIM_MasterConfigTypeDef mc  = {0};
    TIM_OC_InitTypeDef      oc  = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 71;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    scc.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &scc) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

    mc.MasterOutputTrigger = TIM_TRGO_RESET;
    mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &mc) != HAL_OK)
        Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

/* =====================================================================
   USART2 — 115200 8N1, printf retarget
   ===================================================================== */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* =====================================================================
   GPIO INIT
   ===================================================================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Pre-drive outputs LOW */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|
                      GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

    /* Blue button PC13 — EXTI rising, no pull */
    g.Pin  = B1_Pin;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &g);

    /* US TRIG outputs: PC0, PC1, PC3, PC5 */
    g.Pin   = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);

    /* US ECHO inputs PA4 (US1), PA5 (US4) — pull-down */
    g.Pin  = GPIO_PIN_4|GPIO_PIN_5;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &g);

    /* US ECHO PC2 (US2) — pull-down input */
    g.Pin  = GPIO_PIN_2;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &g);

    /* US ECHO PC6 (US3) — pull-down input */
    g.Pin  = GPIO_PIN_6;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &g);

    /* GPIOB outputs: US5-trig PB0, buzzer PB8, LED1 PB9,
                      US6-trig PB10, LED2 PB12,
                      ENB PB13, IN1 PB14, IN2 PB15 */
    g.Pin   = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|
              GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* US ECHO PB1 (US5), PB11 (US6) — pull-down input */
    g.Pin  = GPIO_PIN_1|GPIO_PIN_11;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &g);

    /* Motor 2 direction: PC7 (IN3), PC8 (IN4) — output */
    g.Pin   = GPIO_PIN_7|GPIO_PIN_8;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
}

/* =====================================================================
   ERROR HANDLER
   ===================================================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
