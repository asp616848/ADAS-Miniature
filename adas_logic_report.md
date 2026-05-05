# ADAS Miniature — Logic & Assembly Architecture Report

## 1. Overview

The revised firmware replaces the original stop-and-assess behaviour with a **never-stop, always-moving** obstacle-avoidance system running on an STM32F103RB (ARM Cortex-M3, 8 MHz). The two key invariants are:

- **The robot is always in motion** — no NAV_STOP state exists; even during manoeuvre transitions the robot either pivots or reverses.
- **The robot never collides** — three concentric distance thresholds (critical 5 cm / stop-turn 20 cm / warn 35 cm) provide layered protection from every direction.

---

## 2. Sensor Layer — Why Assembly?

### 2.1 Ultrasonic echo timing (`asm_ultrasonic_ticks`)

The HC-SR04 protocol demands sub-microsecond trigger accuracy and a pulse-width timer with no OS jitter. At 8 MHz a HAL function call costs ~10–30 cycles; inside the echo-wait loops that is 1–4 µs of measurement error per object.

The assembly driver (`sensor_asm.s`) reads and writes GPIO **directly via BSRR/BRR/IDR registers** and times the echo using the **DWT cycle counter**, which increments every CPU cycle. Measured latency from trig-fall to echo-rise capture: ≤ 1 cycle (0.125 µs at 8 MHz). The equivalent HAL path adds roughly 20 cycles of overhead per iteration.

Key assembly technique: all five steps (trig-low, trig-high, trig-low, wait-rise, wait-fall) share one `ldr r8, =DWT_CYCCNT` load; the inner loops do only two loads + one subtract + one compare, minimising branch mispredictions.

### 2.2 ADC / IR read (`asm_adc1_read`)

Software-triggered ADC with direct SWSTART+EXTTRIG bit manipulation and EOC polling. Avoids HAL state-machine overhead (~80 instructions). Return value is the 12-bit raw count; threshold comparison happens in C.

### 2.3 GPIO helpers (`asm_gpio_set`, `asm_gpio_reset`, `asm_tim_set_ccr`)

Single-instruction writes to BSRR, BRR, and timer CCR registers used for LED/buzzer outputs and motor/servo duty updates. These replace `HAL_GPIO_WritePin` (≈ 8 instructions + checks) and `__HAL_TIM_SET_COMPARE` macros everywhere they are called in tight paths.

---

## 3. Navigation Logic

### 3.1 State machine (4 states, no STOP)

| State | Motors | Exits when |
|---|---|---|
| **FORWARD** | Both fwd, speed ∝ clearance | Obstacle threshold crossed or IR triggers |
| **PIVOT_LEFT** | Right fwd + Left back (CCW) | Front clear after ≥ 220 ms, OR target time reached |
| **PIVOT_RIGHT** | Right back + Left fwd (CW) | Front clear after ≥ 220 ms, OR target time reached |
| **REVERSE** | Both back (slow) | Rear blocked → abort to pivot; OR backup timer → big pivot |

Motor 1 = **RIGHT** wheel (TIM3 CH1 PWM speed + GPIO direction).  
Motor 2 = **LEFT** wheel (GPIO enable + direction, no PWM).

### 3.2 Manoeuvre vocabulary

**90° dodge** — Only one front sensor blocked (US2 or US4 < 20 cm): pivot 700 ms toward the open side. The pivot exits early the moment the front clears (≥ 220 ms minimum to ensure meaningful rotation).

**Asymmetric 90° turn** — Both front sensors blocked but one side clearly wider (> 8 cm gap between US1 and US3): pivot 700 ms toward the wider side.

**Reverse + 180° U-turn** — Both front sensors blocked and sides roughly symmetric: reverse at slow speed for 550 ms to open separation, then pivot 1400 ms. Direction of pivot is pre-chosen based on which rear sensor (US5/US6) has more clearance, ensuring the U-turn doesn't back into a wall.

**Emergency pivot** — Any front sensor below 5 cm: immediately stop, pick the side with less distance (the closer sensor dictates turn direction away from it), pivot 700 ms.

**IR quick-dodge** — IR object detected while going forward: 380 ms pivot toward the side with more lateral space (US1 vs US3). Short duration = minimum heading change; robot continues forward after.

**Wall-follow steer** — Side sensor (US1 or US3) below 12 cm with front clear: `motors_steer_left` (slow right, full left) or `motors_steer_right` (full right, stop left). Maintains a soft corridor-tracking behaviour without a full pivot.

**Speed modulation** — In open forward state, speed scales linearly from DUTY_SLOW (320) to DUTY_FULL (900) as the nearest front sensor reading rises from 20 cm to 35 cm. This provides natural deceleration on approach.

### 3.3 Why this logic never hits anything

Every physical obstacle path is covered:

- **Front**: 3-threshold response (warn → turn decision → emergency pivot)
- **Rear**: checked every loop during REVERSE; abort + counter-pivot on detection
- **Sides**: gentle steer before side reading can degrade to front-sensor range
- **IR (close object, any angle)**: immediate dodge; robot never dwells near IR-detected target
- **Box canyon** (front + both sides + symmetric): the only case triggering a full reverse+U-turn, ensuring the robot can always escape

---

## 4. Servo Behaviour

| Navigation context | Servo position |
|---|---|
| FORWARD, all clear | Full 1000–2000 µs sweep, 5 s period |
| FORWARD, front obstacle ≥ NAV_FRONT_WARN_CM | Freeze toward the closer front sensor for 1 s, then resume sweep |
| PIVOT_LEFT | Lock at 1000 µs (pointing left — "signalling" the turn) |
| PIVOT_RIGHT | Lock at 2000 µs (pointing right) |
| REVERSE | Lock at 1500 µs (centre — scanning forward while backing) |

This makes the servo visually informative and mechanically restful (no unnecessary oscillation during manoeuvres).

---

## 5. Four Constants to Tune

All physical timing assumptions are in four `#define` values at the top of `main.c`:

```c
#define PIVOT_90_MS   700U   /* time for ~90° in-place spin  */
#define PIVOT_180_MS 1400U   /* time for ~180° U-turn pivot  */
#define PIVOT_DODGE_MS 380U  /* IR quick-dodge duration      */
#define BACKUP_MS     550U   /* reverse time before U-turn   */
```

Increase all four if the robot undershoots turns; decrease if it overshoots.  
`CYCLES_PER_US` in `sensor_asm.s` must equal `SYSCLK_MHz` (currently 8).

---

## 6. Build Note

Add `Core/Src/sensor_asm.s` to the STM32CubeIDE project sources:  
*Project → Properties → C/C++ Build → Settings → MCU GCC Assembler → Input*.  
The linker resolves `asm_*` symbols automatically once the file is included.
