/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Set to 1 for minimal UART-only debug loop */
#define UART_DEBUG_ONLY 0
#define ADC_TIMEOUT_MS  10U
#define I2C_TIMEOUT_MS  20U
#define IMU_ADDR_68     (0x68U << 1)
#define IMU_ADDR_69     (0x69U << 1)
#define US_MIN_CM       2.0f
#define US_MAX_CM       300.0f
#define US_GAP_MS       25U
#define IR_ALERT_TH     1000U
#define US_ALERT_START_CM 20.0f
#define US_ALERT_FULL_CM  5.0f
#define US_BEEP_SLOW_MS   400U
#define US_BEEP_FAST_MS    80U

/* Motor-1 mapping
 * ENA -> PA8, IN1 -> PB14, IN2 -> PB15, C1 -> PA6(TIM3_CH1), C2 -> PA7(TIM3_CH2)
 */
#define M1_ENA_PORT       GPIOA
#define M1_ENA_PIN        GPIO_PIN_8
#define M1_IN1_PORT       GPIOB
#define M1_IN1_PIN        GPIO_PIN_14
#define M1_IN2_PORT       GPIOB
#define M1_IN2_PIN        GPIO_PIN_15
#define M1_PWM_PERIOD     999U
#define M1_US1_START_CM   30.0f
#define M1_US1_FULL_CM     8.0f
#define M1_DUTY_MIN       250U
#define M1_DUTY_MAX       999U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint16_t imu_i2c_addr = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ─── 2. printf → UART retarget ───────────────────────────── */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ─── 3. Ultrasonic pin definitions ───────────────────────── */
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

#define BUZZER_PORT    GPIOB
#define BUZZER_PIN     GPIO_PIN_8
#define ALERT_LED_PORT GPIOB
#define ALERT_LED_PIN  GPIO_PIN_12
#define ALERT_LED2_PORT GPIOA
#define ALERT_LED2_PIN  GPIO_PIN_13

/* ─── 4. Microsecond delay (DWT cycle counter) ────────────── */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ─── 5. Ultrasonic distance read (returns cm, -1 on timeout) */
float ultrasonic_read(GPIO_TypeDef *trig_port, uint16_t trig_pin,
                      GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
    uint32_t timeout = 30000;

    /* Send 10 µs trigger pulse */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    /* Wait for ECHO to go HIGH */
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET)
        if (--timeout == 0) return -1.0f;

    uint32_t start = DWT->CYCCNT;

    /* Wait for ECHO to go LOW */
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET)
        if (--timeout == 0) return -1.0f;

    uint32_t end = DWT->CYCCNT;

    /* Convert cycle count → µs → cm  (speed of sound = 0.0343 cm/µs) */
    float time_us = (end - start) / (SystemCoreClock / 1000000.0f);
    float distance_cm = (time_us * 0.0343f) / 2.0f;

    if ((distance_cm < US_MIN_CM) || (distance_cm > US_MAX_CM))
    {
        return -1.0f;
    }

    return distance_cm;
}

/* ─── 6. IR sensor read (ADC, 12-bit, 0–4095) ─────────────── */
uint16_t read_ir(void)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0xFFFFU;
    }

    if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK)
    {
        return 0xFFFFU;
    }

    return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

/* ─── 7. IMU MPU6050 – WHO_AM_I check (expect 0x68) ──────── */
static uint16_t imu_detect_address(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADDR_68, 2, I2C_TIMEOUT_MS) == HAL_OK)
    {
        return IMU_ADDR_68;
    }

    if (HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADDR_69, 2, I2C_TIMEOUT_MS) == HAL_OK)
    {
        return IMU_ADDR_69;
    }

    return 0U;
}

static void i2c_scan_print(void)
{
    uint8_t found = 0U;

    printf("I2C scan: ");
    for (uint16_t addr = 0x03U; addr <= 0x77U; addr++)
    {
        uint16_t dev = (uint16_t)(addr << 1);
        if (HAL_I2C_IsDeviceReady(&hi2c1, dev, 1, 3) == HAL_OK)
        {
            printf("0x%02X ", (unsigned int)addr);
            found = 1U;
        }
    }

    if (found == 0U)
    {
        printf("none");
    }

    printf("\r\n");
}

uint8_t imu_whoami(void)
{
    uint8_t reg  = 0x75;
    uint8_t data = 0xFF;

    if (imu_i2c_addr == 0U)
    {
        imu_i2c_addr = imu_detect_address();
        if (imu_i2c_addr == 0U)
        {
            return 0xFF;
        }
    }

    if (HAL_I2C_Mem_Read(&hi2c1, imu_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_MS) != HAL_OK)
    {
        imu_i2c_addr = 0U;
        return 0xFE;
    }

    return data;
}

static float min_valid_distance_cm2(float d1, float d2)
{
    float min_cm = -1.0f;
    float values[2] = { d1, d2 };

    for (uint32_t i = 0U; i < 2U; i++)
    {
        if (values[i] >= US_MIN_CM)
        {
            if ((min_cm < 0.0f) || (values[i] < min_cm))
            {
                min_cm = values[i];
            }
        }
    }

    return min_cm;
}

static void update_proximity_output(float us_cm,
                                    GPIO_TypeDef *port,
                                    uint16_t pin,
                                    uint32_t now_ms,
                                    uint32_t *last_toggle_ms,
                                    GPIO_PinState *state)
{
    if ((us_cm < 0.0f) || (us_cm > US_ALERT_START_CM))
    {
        *state = GPIO_PIN_RESET;
        HAL_GPIO_WritePin(port, pin, *state);
        *last_toggle_ms = now_ms;
        return;
    }

    if (us_cm <= US_ALERT_FULL_CM)
    {
        *state = GPIO_PIN_SET;
        HAL_GPIO_WritePin(port, pin, *state);
        return;
    }

    /* Distance 20->5 cm maps to beep period 400->80 ms (closer = faster beep). */
    float ratio = (US_ALERT_START_CM - us_cm) / (US_ALERT_START_CM - US_ALERT_FULL_CM);
    uint32_t period_ms = (uint32_t)(US_BEEP_SLOW_MS - (ratio * (float)(US_BEEP_SLOW_MS - US_BEEP_FAST_MS)));
    uint32_t half_period_ms = period_ms / 2U;

    if ((now_ms - *last_toggle_ms) >= half_period_ms)
    {
        *state = (*state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(port, pin, *state);
        *last_toggle_ms = now_ms;
    }
}

static void alert_update(uint16_t ir_raw, float us5_cm, float us6_cm)
{
    static uint32_t buzzer_toggle_ms = 0U;
    static uint32_t us5_led_toggle_ms = 0U;
    static uint32_t us6_led_toggle_ms = 0U;
    static GPIO_PinState buzzer_state = GPIO_PIN_RESET;
    static GPIO_PinState us5_led_state = GPIO_PIN_RESET;
    static GPIO_PinState us6_led_state = GPIO_PIN_RESET;
    uint32_t now_ms = HAL_GetTick();

    /* Keep existing IR behavior: IR alert forces buzzer + existing LED ON. */
    if ((ir_raw != 0xFFFFU) && (ir_raw < IR_ALERT_TH))
    {
        buzzer_state = GPIO_PIN_SET;
        us5_led_state = GPIO_PIN_SET;
        us6_led_state = GPIO_PIN_SET;
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, buzzer_state);
        HAL_GPIO_WritePin(ALERT_LED_PORT, ALERT_LED_PIN, us5_led_state);
        HAL_GPIO_WritePin(ALERT_LED2_PORT, ALERT_LED2_PIN, us6_led_state);
        return;
    }

    /* Buzzer frequency follows closest of US5/US6 only. */
    float min_us_cm = min_valid_distance_cm2(us5_cm, us6_cm);
    update_proximity_output(min_us_cm,
                            BUZZER_PORT,
                            BUZZER_PIN,
                            now_ms,
                            &buzzer_toggle_ms,
                            &buzzer_state);

    /* Existing LED tracks US5 channel. */
    update_proximity_output(us5_cm,
                            ALERT_LED_PORT,
                            ALERT_LED_PIN,
                            now_ms,
                            &us5_led_toggle_ms,
                            &us5_led_state);

    /* New LED tracks US6 channel. */
    update_proximity_output(us6_cm,
                            ALERT_LED2_PORT,
                            ALERT_LED2_PIN,
                            now_ms,
                            &us6_led_toggle_ms,
                            &us6_led_state);
}

  static void motor1_set_pwm(uint32_t ch1, uint32_t ch2)
  {
    if (ch1 > M1_PWM_PERIOD)
    {
      ch1 = M1_PWM_PERIOD;
    }

    if (ch2 > M1_PWM_PERIOD)
    {
      ch2 = M1_PWM_PERIOD;
    }

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ch1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ch2);
  }

  static void motor1_update_from_us1(float us1_cm)
  {
    (void)us1_cm;

    /* Enable motor driver and set forward direction on IN1/IN2. */
    HAL_GPIO_WritePin(M1_ENA_PORT, M1_ENA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, GPIO_PIN_RESET);

    /* Debug mode: force motor always ON at maximum duty. */
    motor1_set_pwm(M1_DUTY_MAX, 0U);
  }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  static const char boot_msg[] = "\r\n[BOOT] USART2 raw TX OK\r\n";
  static const char alive_msg[] = "[ALIVE] loop\r\n";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
#if UART_DEBUG_ONLY
  while (1)
  {
    printf("UART DEBUG: hello from NUCLEO-F103RB\r\n");
    HAL_Delay(1000);
  }
#endif

  HAL_UART_Transmit(&huart2, (uint8_t *)boot_msg, sizeof(boot_msg) - 1U, HAL_MAX_DELAY);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  motor1_set_pwm(0U, 0U);
  DWT_Init();
  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\r\n=== STM32F103RB Sensor Bring-Up Test ===\r\n");
  printf("US1-US6 | IR (ADC) | IMU WHO_AM_I\r\n");
  printf("----------------------------------------\r\n");
  i2c_scan_print();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)alive_msg, sizeof(alive_msg) - 1U, HAL_MAX_DELAY);

    float d1 = ultrasonic_read(US1_TRIG_PORT, US1_TRIG_PIN, US1_ECHO_PORT, US1_ECHO_PIN);
    HAL_Delay(US_GAP_MS);
    float d2 = ultrasonic_read(US2_TRIG_PORT, US2_TRIG_PIN, US2_ECHO_PORT, US2_ECHO_PIN);
    HAL_Delay(US_GAP_MS);
    float d3 = ultrasonic_read(US3_TRIG_PORT, US3_TRIG_PIN, US3_ECHO_PORT, US3_ECHO_PIN);
    HAL_Delay(US_GAP_MS);
    float d4 = ultrasonic_read(US4_TRIG_PORT, US4_TRIG_PIN, US4_ECHO_PORT, US4_ECHO_PIN);
    HAL_Delay(US_GAP_MS);
    float d5 = ultrasonic_read(US5_TRIG_PORT, US5_TRIG_PIN, US5_ECHO_PORT, US5_ECHO_PIN);
    HAL_Delay(US_GAP_MS);
    float d6 = ultrasonic_read(US6_TRIG_PORT, US6_TRIG_PIN, US6_ECHO_PORT, US6_ECHO_PIN);

    uint16_t ir = read_ir();
    uint8_t whoami = imu_whoami();

    motor1_update_from_us1(d1);
    alert_update(ir, d5, d6);

    const char *imu_status;
    if (whoami == 0x68)
    {
      imu_status = " [OK]";
    }
    else if (whoami == 0xFF)
    {
      imu_status = " [NO DEVICE 0x68/0x69]";
    }
    else if (whoami == 0xFE)
    {
      imu_status = " [I2C READ ERROR]";
    }
    else
    {
      imu_status = " [WHO_AM_I MISMATCH]";
    }

    printf(
      "US(cm): %3d.%d %3d.%d %3d.%d %3d.%d %3d.%d %3d.%d | IR: %4u | IMU: 0x%02X @0x%02X%s\r\n",
      (int)d1, (int)(d1 * 10.0f) % 10,
      (int)d2, (int)(d2 * 10.0f) % 10,
      (int)d3, (int)(d3 * 10.0f) % 10,
      (int)d4, (int)(d4 * 10.0f) % 10,
      (int)d5, (int)(d5 * 10.0f) % 10,
      (int)d6, (int)(d6 * 10.0f) % 10,
      ir,
      whoami,
      (unsigned int)imu_i2c_addr,
      imu_status
    );

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(80);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,                                   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_ENA_PORT, M1_ENA_PIN,                              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN,                              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN,                              GPIO_PIN_RESET);
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC3 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB12 PB13
                           PB14 PB15 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* PA13 = alert LED2 (US6) */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
