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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Controller parameters */
#define PID_KP  0.045f
#define PID_KI  0.0069f
#define PID_KD  0.0065f
#define PID_TAU 0.01f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

// PWM
#define PWM_REV_TIM      htim1
#define PWM_REV_CH       TIM_CHANNEL_1

#define PWM_FWD_TIM htim3
#define PWM_FWD_CH  TIM_CHANNEL_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float desiredAngle = 60.0f;   // default setpoint

/* PID is global so the UART callback can hot-update gains at runtime */
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty);
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
  PIDController_Init(&pid);   // pid is declared globally above
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // REV (PA8)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // FWD (PB4)
  HAL_UART_Receive_IT(&huart2, &rxByte, 1);

  const int countsPerRev = 444;

  float angleValue = 0.0f;
  float u = 0.0f;
  float duty_cmd = 0.0f;

  static int32_t prevRawCount = 0;
  static float   absAngle     = 0.0f;
  prevRawCount = (int32_t)TIM2->CNT;   // seed with current position on boot

  // Direction hysteresis thresholds (tune these if needed)
  const float U_ON  = 0.3f;   // start moving
  const float U_OFF = 0.15f;  // stop moving

  // Limit max duty (soften motion; reduce jitter)
  const float DUTY_MAX = 0.7f;

  // Direction state machine
  int8_t dir = 0;
  static int8_t dir_state = 0;
  static int8_t prev_dir = 0;

  // Direction change deadtime (ms)
  const uint32_t DEADTIME_MS = 50;

  char printMessage[200];
  char printMessage2[200];

  uint32_t lastTick = HAL_GetTick();
  uint32_t nextPrintTick = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

    // --- 1) Enforce fixed sample time ---
    uint32_t now = HAL_GetTick();
    if ((now - lastTick) < (uint32_t)(SAMPLE_TIME_S * 1000.0f)) {
        continue; // wait until next sample
    }
    lastTick = now;

    // --- 2) Read encoder as absolute linear position (no 0-360 wraparound) ---
    int32_t rawCount = (int32_t)TIM2->CNT;

    // Detect 16-bit counter rollover and accumulate
    int32_t delta = rawCount - prevRawCount;
    if (delta >  32767) delta -= 65536;   // rolled over forward
    if (delta < -32767) delta += 65536;   // rolled over backward
    prevRawCount = rawCount;

    absAngle += (360.0f / (float)countsPerRev) * (float)delta;
    angleValue = absAngle;

    // --- 3) PID update (pid.c handles wrap-aware error + derivative now) ---
    u = PIDController_Update(&pid, desiredAngle, angleValue);

    // --- 4) Convert u -> duty magnitude, apply duty cap ---
    duty_cmd = fabsf(u) / pid.limMax;

    if (duty_cmd > DUTY_MAX) duty_cmd = DUTY_MAX;

    // --- 5) Direction hysteresis state machine (prevents chatter) ---
    if (dir_state == 0) {
        if (u >  U_ON) dir_state = +1;
        else if (u < -U_ON) dir_state = -1;
    } else if (dir_state == +1) {
        if (u <  U_OFF) dir_state = 0;
        else if (u < -U_ON) dir_state = -1;
    } else { // dir_state == -1
        if (u > -U_OFF) dir_state = 0;
        else if (u >  U_ON) dir_state = +1;
    }
    dir = dir_state;

    // If stopped, force duty to 0
    if (dir == 0) duty_cmd = 0.0f;

    // --- 6) Apply deadtime on direction changes (+1 <-> -1) ---
    if ((dir != prev_dir) && (dir != 0) && (prev_dir != 0))
    {
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
        HAL_Delay(DEADTIME_MS);
    }
    prev_dir = dir;

    // --- 7) Drive PWM outputs ---
    if (dir < 0) {
        // Forward: PB4 PWM, PA8 off
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, duty_cmd);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    } else if (dir > 0) {
        // Reverse: PA8 PWM, PB4 off
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, duty_cmd);
    } else {
        // Stop: both off
        PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
        PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    }

    // --- 8) Throttled UART print (every 100 ms) ---
    if (now >= nextPrintTick)
    {
        nextPrintTick = now + 100;

        int n = snprintf(printMessage, sizeof(printMessage),
                         "Desired, Actual, Duty, u, P, I, D, tau, dir: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d\r\n",
                         desiredAngle, angleValue, duty_cmd, u, pid.Kp, pid.Ki, pid.Kd, pid.tau, dir);
        if (n > 0) {
            HAL_UART_Transmit(&huart2, (uint8_t*)printMessage, (uint16_t)strlen(printMessage), 300);
        }
    }

    /* USER CODE END 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

/*
 * VOFA+ command parser
 * ---------------------
 * Send commands from VOFA's "RawData → FireWater" or plain serial terminal.
 * Format:  PREFIX=VALUE!
 *   KP=0.012!    -> set Kp
 *   KI=0.001!    -> set Ki
 *   KD=0.005!    -> set Kd
 *   TAU=0.05!    -> set derivative filter tau
 *   SP=90!       -> set target angle (setpoint)
 *
 * The '!' character terminates each command (no newline required,
 * but newline is also accepted as a fallback terminator).
 *
 * A confirmation string is echoed back after each accepted command.
 */
static void VOFA_ParseCommand(const char *line)
{
    /* Find the '=' separator */
    const char *eq = strchr(line, '=');
    if (eq == NULL) return;

    /* Value string starts after '=' */
    float val = strtof(eq + 1, NULL);

    /* Match prefix (case-sensitive, must match exactly) */
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
        pid.integrator = 0.0f;   // reset integrator on Ki change
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

        /* '!' is the primary VOFA command terminator; newline also accepted */
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
            if (rxIdx < (sizeof(rxLine) - 1))
                rxLine[rxIdx++] = c;
            else
                rxIdx = 0;  // overflow — reset
        }

        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
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
