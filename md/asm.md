This file is an **ARM Cortex-M3 Assembly driver code** for the **STM32F103 microcontroller**. It provides very fast low-level hardware control functions written in Assembly language instead of C. 

### Main Purpose

The code is used for:

* precise timing
* GPIO pin control
* ultrasonic sensor measurement
* ADC sensor reading
* timer PWM control

---

# Short Viva Explanation

## 1. `asm_dwt_init()`

Initializes the **DWT cycle counter** inside Cortex-M3.

### Purpose:

* Enables accurate timing using CPU clock cycles.
* Used for microsecond delays and pulse measurements.

### Important Registers:

* `DEMCR`
* `DWT_CTRL`
* `DWT_CYCCNT`

---

## 2. `asm_delay_us(us)`

Creates a **microsecond delay** using the DWT counter.

### Working:

* Reads current CPU cycles.
* Waits until required cycles pass.

### Formula:

At 8 MHz:
\text{cycles} = \text{microseconds} \times 8

---

## 3. `asm_gpio_set()`

Sets GPIO pin HIGH.

### Working:

Writes pin mask into:

* `GPIO_BSRR` register

Used for:

* turning ON LEDs
* sending trigger pulse

---

## 4. `asm_gpio_reset()`

Sets GPIO pin LOW.

### Working:

Writes pin mask into:

* `GPIO_BRR` register

---

## 5. `asm_tim_set_ccr()`

Updates Timer Compare Register (CCR).

### Purpose:

Controls:

* PWM duty cycle
* servo motor position
* motor speed

---

# 6. `asm_ultrasonic_ticks()`

Measures ultrasonic sensor echo pulse width.

### Steps:

1. Make TRIG LOW
2. Send 10 µs HIGH pulse
3. Wait for ECHO HIGH
4. Measure HIGH pulse duration
5. Return pulse width in CPU cycles

### Used For:

Distance measurement using HC-SR04 sensor.

### Timeout:

Returns `0` if no echo detected.

---

# 7. `asm_adc1_read()`

Reads analog value from ADC1.

### Working:

* Starts ADC conversion
* Waits for EOC (End Of Conversion)
* Returns 12-bit ADC value

### Error:

Returns `0xFFFF` if timeout/error occurs.

---

# Why Assembly is Used?

Assembly gives:

* faster execution
* precise timing
* direct hardware access
* less overhead than C

Very useful in:

* embedded systems
* real-time applications
* sensor interfacing

---

# One-Line Viva Summary

> “This assembly file provides fast low-level drivers for STM32F103 to perform accurate timing, GPIO control, ultrasonic sensing, ADC reading, and timer PWM operations using Cortex-M3 hardware registers.” 
