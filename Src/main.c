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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Controller parameters */
#define PID_KP  0.1912f
#define PID_KI  0.2833f
#define PID_KD  0.0163f
#define PID_TAU 0.0913f

#define PID_LIM_MIN     -10.0f
#define PID_LIM_MAX      10.0f
#define PID_LIM_MIN_INT  -5.0f
#define PID_LIM_MAX_INT   5.0f

#define SAMPLE_TIME_S 0.01f

/* PWM */
#define PWM_REV_TIM      htim1
#define PWM_REV_CH       TIM_CHANNEL_1

#define PWM_FWD_TIM      htim3
#define PWM_FWD_CH       TIM_CHANNEL_1

/* Solenoid output pin: change to your real pin */
#define SOLENOID_GPIO_Port GPIOB
#define SOLENOID_Pin       GPIO_PIN_5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile float desiredAngle = 60.0f;   /* default setpoint */
volatile uint8_t solenoidState = 0;    /* 0 = release, 1 = press */

/* PID is global so UART callback can hot-update gains */
PIDController pid = {
    PID_KP, PID_KI, PID_KD,
    PID_TAU,
    PID_LIM_MIN, PID_LIM_MAX,
    PID_LIM_MIN_INT, PID_LIM_MAX_INT,
    SAMPLE_TIME_S
};

static uint8_t rxByte;
static char rxLine[32];
static uint8_t rxIdx = 0;

/* Limit switch flag */
volatile uint8_t limitSwitchRight = 0;

/* Direction state */
static int8_t dir_state = 0;
static int8_t prev_dir  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static inline void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty);
static inline void Solenoid_Set(uint8_t on);
static void VOFA_ParseCommand(const char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  PIDController_Init(&pid);
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* REV (PA8) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* FWD (PB4) */
  HAL_UART_Receive_IT(&huart2, &rxByte, 1);

  Solenoid_Set(0);   /* ensure released at boot */

  const int countsPerRev = 700;

  float angleValue = 0.0f;
  float u = 0.0f;
  float duty_cmd = 0.0f;

  static int32_t prevRawCount = 0;
  static float absAngle = 0.0f;
  prevRawCount = (int32_t)TIM4->CNT;

  const float U_ON  = 0.3f;
  const float U_OFF = 0.05f;
  const float DUTY_MAX = 1.0f;

  int8_t dir = 0;
  const uint32_t DEADTIME_MS = 50;

  char printMessage[220];

  uint32_t lastTick = HAL_GetTick();
  uint32_t nextPrintTick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Homing routine */
  dir = -1;
  PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.3f);
  PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
  HAL_Delay(500);
  PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
  HAL_Delay(50);

  dir = 1;
  limitSwitchRight = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 0 : 1;
  while (limitSwitchRight != 1)
  {
      PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.2f);
      PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
      HAL_Delay(5);
      limitSwitchRight = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 0 : 1;
  }

  PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
  PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
  Solenoid_Set(0);

  angleValue         = 0.0f;
  desiredAngle       = 0.0f;
  prevRawCount       = (int32_t)TIM4->CNT;
  absAngle           = 0.0f;
  pid.integrator     = 0.0f;
  pid.differentiator = 0.0f;
  dir_state          = 0;
  prev_dir           = 0;
  limitSwitchRight   = 0;
  HAL_Delay(200);

  while (1)
  {
    uint32_t now = HAL_GetTick();
    if ((now - lastTick) < (uint32_t)(SAMPLE_TIME_S * 1000.0f))
    {
        continue;
    }
    lastTick = now;

    int32_t rawCount = (int32_t)TIM4->CNT;

    int32_t delta = rawCount - prevRawCount;
    if (delta > 32767)  delta -= 65536;
    if (delta < -32767) delta += 65536;
    prevRawCount = rawCount;

    absAngle += (360.0f / (float)countsPerRev) * (float)delta;
    angleValue = absAngle;

    u = PIDController_Update(&pid, desiredAngle, angleValue);

    duty_cmd = fabsf(u) / pid.limMax;
    if (duty_cmd > DUTY_MAX) duty_cmd = DUTY_MAX;
    if (duty_cmd < 0.15f) duty_cmd = 0.0f;

    if (dir_state == 0)
    {
        if (u > U_ON) dir_state = +1;
        else if (u < -U_ON) dir_state = -1;
    }
    else if (dir_state == +1)
    {
        if (u < U_OFF) dir_state = 0;
        else if (u < -U_ON) dir_state = -1;
    }
    else
    {
        if (u > -U_OFF) dir_state = 0;
        else if (u > U_ON) dir_state = +1;
    }

    dir = dir_state;

    limitSwitchRight = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 0 : 1;

    if (limitSwitchRight && dir > 0)
    {
        dir             = 0;
        dir_state       = 0;
        duty_cmd        = 0.0f;
        pid.integrator  = 0.0f;
        absAngle        = 0.0f;
        desiredAngle    = 0.0f;
        Solenoid_Set(0);
    }

    if (dir == 0) duty_cmd = 0.0f;

    if ((dir != prev_dir) && (dir != 0) && (prev_dir != 0))
    {
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
        HAL_Delay(DEADTIME_MS);
    }
    prev_dir = dir;

    if (dir < 0)
    {
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, duty_cmd);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    }
    else if (dir > 0)
    {
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, duty_cmd);
    }
    else
    {
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    }

    if (now >= nextPrintTick)
    {
        nextPrintTick = now + 10;

        int n = snprintf(
            printMessage,
            sizeof(printMessage),
            "Desired, Actual, Duty, u, P, I, D, tau, dir, sw: %.3f, %.3f, %.3f, %.3f, %.5f, %.5f, %.5f, %.5f, %d, %d\r\n",
            desiredAngle,
            angleValue,
            duty_cmd,
            u,
            pid.Kp,
            pid.integrator,
            pid.differentiator,
            pid.tau,
            dir,
            solenoidState
        );

        if (n > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t*)printMessage, (uint16_t)strlen(printMessage), 300);
        }
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static inline void Solenoid_Set(uint8_t on)
{
    solenoidState = (on != 0) ? 1 : 0;

    if (solenoidState)
        HAL_GPIO_WritePin(SOLENOID_GPIO_Port, SOLENOID_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(SOLENOID_GPIO_Port, SOLENOID_Pin, GPIO_PIN_SET);
}

static void VOFA_ParseCommand(const char *line)
{
    const char *eq = strchr(line, '=');
    if (eq == NULL) return;

    float val = strtof(eq + 1, NULL);

    size_t prefixLen = (size_t)(eq - line);
    char prefix[8] = {0};
    if (prefixLen >= sizeof(prefix)) return;
    memcpy(prefix, line, prefixLen);

    char ack[64];

    if (strcmp(prefix, "KP") == 0)
    {
        pid.Kp = val;
        snprintf(ack, sizeof(ack), "ACK KP=%.5f\r\n", pid.Kp);
    }
    else if (strcmp(prefix, "KI") == 0)
    {
        pid.Ki = val;
        pid.integrator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KI=%.5f\r\n", pid.Ki);
    }
    else if (strcmp(prefix, "KD") == 0)
    {
        pid.Kd = val;
        pid.differentiator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KD=%.5f\r\n", pid.Kd);
    }
    else if (strcmp(prefix, "TAU") == 0)
    {
        pid.tau = val;
        snprintf(ack, sizeof(ack), "ACK TAU=%.5f\r\n", pid.tau);
    }
    else if (strcmp(prefix, "SP") == 0)
    {
        desiredAngle = val;
        snprintf(ack, sizeof(ack), "ACK SP=%.3f\r\n", desiredAngle);
    }
    else if (strcmp(prefix, "SL") == 0)
    {
        if (val >= 0.5f)
        {
            Solenoid_Set(1);
            snprintf(ack, sizeof(ack), "ACK SL=1\r\n");
        }
        else
        {
            Solenoid_Set(0);
            snprintf(ack, sizeof(ack), "ACK SL=0\r\n");
        }
    }
    else
    {
        snprintf(ack, sizeof(ack), "ERR unknown cmd: %s\r\n", prefix);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)ack, (uint16_t)strlen(ack), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char c = (char)rxByte;

        if (c == '!' || c == '\n' || c == '\r')
        {
            if (rxIdx > 0)
            {
                rxLine[rxIdx] = '\0';
                VOFA_ParseCommand(rxLine);
                rxIdx = 0;
            }
        }
        else
        {
            if (rxIdx < (sizeof(rxLine) - 1U))
                rxLine[rxIdx++] = c;
            else
                rxIdx = 0;
        }

        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        if (dir_state < 0)
        {
            PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
            PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
        }
        limitSwitchRight = 1;
        Solenoid_Set(0);
    }
}

static inline void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)(duty * (float)arr);
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
