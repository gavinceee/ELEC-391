/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : F411 dual-motor PID control + homing + UART command
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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

typedef struct
{
    volatile float desiredAngle;
    volatile float actualAngle;
    volatile float duty_cmd;
    volatile float control_u;
    volatile int8_t dir;

    PIDController pid;

    volatile int32_t prevRawCount;
    volatile float absAngle;

    volatile int8_t dir_state;
    volatile int8_t applied_dir;
    volatile uint16_t deadtimeTicks;

    TIM_HandleTypeDef *enc_tim;
    TIM_HandleTypeDef *pwm_rev_tim;
    uint32_t pwm_rev_ch;
    TIM_HandleTypeDef *pwm_fwd_tim;
    uint32_t pwm_fwd_ch;
} MotorControl;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---------- PID defaults ---------- */
#define PID_KP           0.1912f
#define PID_KI           0.2833f
#define PID_KD           0.0163f
#define PID_TAU          0.0913f

#define PID_LIM_MIN     -10.0f
#define PID_LIM_MAX      10.0f
#define PID_LIM_MIN_INT  -5.0f
#define PID_LIM_MAX_INT   5.0f

#define SAMPLE_TIME_S    0.001f
#define TELEMETRY_MS     10U

/* ---------- Encoders ---------- */
#define COUNTS_PER_REV_M1   700.0f
#define COUNTS_PER_REV_M2   700.0f

/* ---------- Motor 1: carriage ---------- */
#define M1_PWM_REV_TIM      htim1
#define M1_PWM_REV_CH       TIM_CHANNEL_1
#define M1_PWM_FWD_TIM      htim1
#define M1_PWM_FWD_CH       TIM_CHANNEL_4
#define M1_ENC_TIM          htim4

/* ---------- Motor 2: finger ---------- */
#define M2_PWM_REV_TIM      htim2
#define M2_PWM_REV_CH       TIM_CHANNEL_1
#define M2_PWM_FWD_TIM      htim2
#define M2_PWM_FWD_CH       TIM_CHANNEL_3
#define M2_ENC_TIM          htim3

/* ---------- Solenoid ---------- */
#define SOLENOID_GPIO_Port   GPIOB
#define SOLENOID_Pin         GPIO_PIN_5

/* ---------- Limit switch for motor1 home ---------- */
#define LIMIT_SW_GPIO_Port   GPIOA
#define LIMIT_SW_Pin         GPIO_PIN_0

/* ---------- Hysteresis / safety ---------- */
#define U_ON              0.30f
#define U_OFF             0.05f
#define DUTY_MAX          0.80f
#define DUTY_MIN_ACTIVE   0.15f
#define DEADTIME_MS       50U

/* ---------- Homing ---------- */
#define HOMING_MOVE_AWAY_DUTY   0.35f
#define HOMING_SEEK_DUTY        0.30f
#define HOMING_MOVE_AWAY_MS     250U
#define HOMING_SETTLE_MS         30U
#define HOMING_TIMEOUT_MS      4000U

#define UART_TX_BUF_SIZE 512U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint8_t solenoidState = 0;
volatile uint8_t limitSwitchRight = 0;

static uint8_t rxByte;
static char rxLine[48];
static uint8_t rxIdx = 0;

static volatile uint8_t controlEnabled = 0U;
static volatile uint8_t homingActive = 0U;
static volatile uint8_t homingRequest = 0U;

static uint8_t uartTxBuf[UART_TX_BUF_SIZE];
static volatile uint16_t uartTxHead = 0U;
static volatile uint16_t uartTxTail = 0U;
static volatile uint16_t uartTxCount = 0U;
static volatile uint16_t uartTxChunkLen = 0U;
static volatile uint8_t uartTxBusy = 0U;

static char printMessage[192];

/* Motor 1 object */
MotorControl motor1 =
{
    .desiredAngle = 60.0f,
    .actualAngle = 0.0f,
    .duty_cmd = 0.0f,
    .control_u = 0.0f,
    .dir = 0,

    .pid = {
        PID_KP,
        PID_KI,
        PID_KD,
        PID_TAU,
        PID_LIM_MIN,
        PID_LIM_MAX,
        PID_LIM_MIN_INT,
        PID_LIM_MAX_INT,
        SAMPLE_TIME_S,
        0.0f,
        0.0f
    },

    .prevRawCount = 0,
    .absAngle = 0.0f,

    .dir_state = 0,
    .applied_dir = 0,
    .deadtimeTicks = 0U,

    .enc_tim = &M1_ENC_TIM,
    .pwm_rev_tim = &M1_PWM_REV_TIM,
    .pwm_rev_ch = M1_PWM_REV_CH,
    .pwm_fwd_tim = &M1_PWM_FWD_TIM,
    .pwm_fwd_ch = M1_PWM_FWD_CH
};

/* Motor 2 object */
MotorControl motor2 =
{
    .desiredAngle = 0.0f,
    .actualAngle = 0.0f,
    .duty_cmd = 0.0f,
    .control_u = 0.0f,
    .dir = 0,

    .pid = {
        PID_KP,
        PID_KI,
        PID_KD,
        PID_TAU,
        PID_LIM_MIN,
        PID_LIM_MAX,
        PID_LIM_MIN_INT,
        PID_LIM_MAX_INT,
        SAMPLE_TIME_S,
        0.0f,
        0.0f
    },

    .prevRawCount = 0,
    .absAngle = 0.0f,

    .dir_state = 0,
    .applied_dir = 0,
    .deadtimeTicks = 0U,

    .enc_tim = &M2_ENC_TIM,
    .pwm_rev_tim = &M2_PWM_REV_TIM,
    .pwm_rev_ch = M2_PWM_REV_CH,
    .pwm_fwd_tim = &M2_PWM_FWD_TIM,
    .pwm_fwd_ch = M2_PWM_FWD_CH
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static inline void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty);
static inline void Solenoid_Set(uint8_t on);
static inline uint8_t Read_Right_Limit_Switch(void);

static void UART_TxKickoff(void);
static void UART_QueueBytes(const uint8_t *data, uint16_t len);
static void UART_QueueString(const char *s);

static void Reset_Motor_PID_State(MotorControl *m);
static void VOFA_ParseCommand(const char *line);
static void Send_Telemetry(void);
static void Start_Peripherals(void);
static void Run_Homing_Routine(void);
static void Motor_Control_Update(MotorControl *m, float counts_per_rev, uint8_t use_limit_switch);
void Control_Loop_1kHz(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)(duty * (float)arr);
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

static inline void Solenoid_Set(uint8_t on)
{
    solenoidState = (on != 0U) ? 1U : 0U;

    HAL_GPIO_WritePin(
        SOLENOID_GPIO_Port,
        SOLENOID_Pin,
        (solenoidState != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

static inline uint8_t Read_Right_Limit_Switch(void)
{
    return (HAL_GPIO_ReadPin(LIMIT_SW_GPIO_Port, LIMIT_SW_Pin) == GPIO_PIN_SET) ? 0U : 1U;
}

static void UART_TxKickoff(void)
{
    if ((uartTxBusy != 0U) || (uartTxCount == 0U))
    {
        return;
    }

    if (uartTxHead > uartTxTail)
    {
        uartTxChunkLen = (uint16_t)(uartTxHead - uartTxTail);
    }
    else
    {
        uartTxChunkLen = (uint16_t)(UART_TX_BUF_SIZE - uartTxTail);
    }

    uartTxBusy = 1U;

    if (HAL_UART_Transmit_IT(&huart2, &uartTxBuf[uartTxTail], uartTxChunkLen) != HAL_OK)
    {
        uartTxBusy = 0U;
        uartTxChunkLen = 0U;
    }
}

static void UART_QueueBytes(const uint8_t *data, uint16_t len)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint16_t i = 0U; i < len; i++)
    {
        if (uartTxCount >= UART_TX_BUF_SIZE)
        {
            break;
        }

        uartTxBuf[uartTxHead] = data[i];
        uartTxHead = (uint16_t)((uartTxHead + 1U) % UART_TX_BUF_SIZE);
        uartTxCount++;
    }

    UART_TxKickoff();

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void UART_QueueString(const char *s)
{
    UART_QueueBytes((const uint8_t *)s, (uint16_t)strlen(s));
}

static void Reset_Motor_PID_State(MotorControl *m)
{
    m->pid.integrator = 0.0f;
    m->pid.differentiator = 0.0f;
    m->pid.prevError = 0.0f;
    m->pid.prevMeasurement = m->actualAngle;

    m->control_u = 0.0f;
    m->duty_cmd = 0.0f;
    m->dir_state = 0;
    m->applied_dir = 0;
    m->dir = 0;
    m->deadtimeTicks = 0U;
}

static void VOFA_ParseCommand(const char *line)
{
    const char *eq = strchr(line, '=');
    if (eq == NULL)
    {
        return;
    }

    float val = strtof(eq + 1, NULL);

    size_t prefixLen = (size_t)(eq - line);
    char prefix[10] = {0};
    char ack[80];

    if (prefixLen >= sizeof(prefix))
    {
        return;
    }

    memcpy(prefix, line, prefixLen);

    if (strcmp(prefix, "KP") == 0)
    {
        motor1.pid.Kp = val;
        snprintf(ack, sizeof(ack), "ACK KP=%.5f\r\n", motor1.pid.Kp);
    }
    else if (strcmp(prefix, "KI") == 0)
    {
        motor1.pid.Ki = val;
        motor1.pid.integrator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KI=%.5f\r\n", motor1.pid.Ki);
    }
    else if (strcmp(prefix, "KD") == 0)
    {
        motor1.pid.Kd = val;
        motor1.pid.differentiator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KD=%.5f\r\n", motor1.pid.Kd);
    }
    else if (strcmp(prefix, "TAU") == 0)
    {
        motor1.pid.tau = val;
        snprintf(ack, sizeof(ack), "ACK TAU=%.5f\r\n", motor1.pid.tau);
    }
    else if (strcmp(prefix, "SP") == 0)
    {
        motor1.desiredAngle = val;
        snprintf(ack, sizeof(ack), "ACK SP=%.3f\r\n", motor1.desiredAngle);
    }
    else if (strcmp(prefix, "KP2") == 0)
    {
        motor2.pid.Kp = val;
        snprintf(ack, sizeof(ack), "ACK KP2=%.5f\r\n", motor2.pid.Kp);
    }
    else if (strcmp(prefix, "KI2") == 0)
    {
        motor2.pid.Ki = val;
        motor2.pid.integrator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KI2=%.5f\r\n", motor2.pid.Ki);
    }
    else if (strcmp(prefix, "KD2") == 0)
    {
        motor2.pid.Kd = val;
        motor2.pid.differentiator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KD2=%.5f\r\n", motor2.pid.Kd);
    }
    else if (strcmp(prefix, "TAU2") == 0)
    {
        motor2.pid.tau = val;
        snprintf(ack, sizeof(ack), "ACK TAU2=%.5f\r\n", motor2.pid.tau);
    }
    else if (strcmp(prefix, "SP2") == 0)
    {
        motor2.desiredAngle = val;
        snprintf(ack, sizeof(ack), "ACK SP2=%.3f\r\n", motor2.desiredAngle);
    }
    else if (strcmp(prefix, "SL") == 0)
    {
        if (val >= 0.5f)
        {
            Solenoid_Set(1U);
            snprintf(ack, sizeof(ack), "ACK SL=1\r\n");
        }
        else
        {
            Solenoid_Set(0U);
            snprintf(ack, sizeof(ack), "ACK SL=0\r\n");
        }
    }
    else if (strcmp(prefix, "RE") == 0)
    {
        if (val >= 0.5f)
        {
            homingRequest = 1U;
            snprintf(ack, sizeof(ack), "ACK RE=1\r\n");
        }
        else
        {
            snprintf(ack, sizeof(ack), "ACK RE=0\r\n");
        }
    }
    else
    {
        snprintf(ack, sizeof(ack), "ERR unknown cmd: %s\r\n", prefix);
    }

    UART_QueueString(ack);
}

static void Send_Telemetry(void)
{
    int n = snprintf(
        printMessage,
        sizeof(printMessage),
        "%.3f, %.3f, %.3f, %.3f, %d, %.3f, %.3f, %.3f, %.3f, %d, %d, %d\r\n",
        motor1.desiredAngle,
        motor1.actualAngle,
        motor1.duty_cmd,
        motor1.control_u,
        motor1.dir,
        motor2.desiredAngle,
        motor2.actualAngle,
        motor2.duty_cmd,
        motor2.control_u,
        motor2.dir,
        solenoidState,
        (int)homingActive
    );

    if (n > 0)
    {
        UART_QueueBytes((const uint8_t *)printMessage, (uint16_t)n);
    }
}

static void Start_Peripherals(void)
{
    PIDController_Init(&motor1.pid);
    PIDController_Init(&motor2.pid);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    PWM_SetDuty(&htim1, TIM_CHANNEL_1, 0.0f);
    PWM_SetDuty(&htim1, TIM_CHANNEL_4, 0.0f);
    PWM_SetDuty(&htim2, TIM_CHANNEL_1, 0.0f);
    PWM_SetDuty(&htim2, TIM_CHANNEL_3, 0.0f);

    motor1.prevRawCount = (int32_t)motor1.enc_tim->Instance->CNT;
    motor2.prevRawCount = (int32_t)motor2.enc_tim->Instance->CNT;

    motor1.absAngle = 0.0f;
    motor2.absAngle = 0.0f;

    Reset_Motor_PID_State(&motor1);
    Reset_Motor_PID_State(&motor2);

    Solenoid_Set(0U);
    HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    controlEnabled = 1U;
}

static void Run_Homing_Routine(void)
{
    uint32_t startTick;

    homingActive = 1U;
    Send_Telemetry();

    controlEnabled = 0U;

    PWM_SetDuty(&htim1, TIM_CHANNEL_1, 0.0f);
    PWM_SetDuty(&htim1, TIM_CHANNEL_4, 0.0f);

    Reset_Motor_PID_State(&motor1);
    limitSwitchRight = Read_Right_Limit_Switch();

    /* Step 1: move away from switch first */
    motor1.dir = -1;
    PWM_SetDuty(&htim1, TIM_CHANNEL_4, HOMING_MOVE_AWAY_DUTY);
    PWM_SetDuty(&htim1, TIM_CHANNEL_1, 0.0f);
    HAL_Delay(HOMING_MOVE_AWAY_MS);

    PWM_SetDuty(&htim1, TIM_CHANNEL_4, 0.0f);
    PWM_SetDuty(&htim1, TIM_CHANNEL_1, 0.0f);
    HAL_Delay(HOMING_SETTLE_MS);

    /* Step 2: seek toward switch */
    motor1.dir = +1;
    startTick = HAL_GetTick();

    while (Read_Right_Limit_Switch() == 0U)
    {
        PWM_SetDuty(&htim1, TIM_CHANNEL_1, HOMING_SEEK_DUTY);
        PWM_SetDuty(&htim1, TIM_CHANNEL_4, 0.0f);

        if ((HAL_GetTick() - startTick) > HOMING_TIMEOUT_MS)
        {
            break;
        }

        HAL_Delay(1);
    }

    PWM_SetDuty(&htim1, TIM_CHANNEL_1, 0.0f);
    PWM_SetDuty(&htim1, TIM_CHANNEL_4, 0.0f);
    Solenoid_Set(0U);

    TIM4->CNT = 0U;
    motor1.prevRawCount = 0;
    motor1.absAngle = 0.0f;
    motor1.actualAngle = 0.0f;
    motor1.desiredAngle = 0.0f;

    Reset_Motor_PID_State(&motor1);

    limitSwitchRight = Read_Right_Limit_Switch();

    HAL_Delay(100);

    homingActive = 0U;
    controlEnabled = 1U;
    Send_Telemetry();
}

static void Motor_Control_Update(MotorControl *m, float counts_per_rev, uint8_t use_limit_switch)
{
    int32_t rawCount;
    int32_t delta;
    int8_t requested_dir = 0;

    rawCount = (int32_t)m->enc_tim->Instance->CNT;

    delta = rawCount - m->prevRawCount;
    if (delta > 32767)  delta -= 65536;
    if (delta < -32767) delta += 65536;
    m->prevRawCount = rawCount;

    m->absAngle += (360.0f / counts_per_rev) * (float)delta;
    m->actualAngle = m->absAngle;

    m->control_u = PIDController_Update(&m->pid, m->desiredAngle, m->actualAngle);

    m->duty_cmd = fabsf(m->control_u) / m->pid.limMax;
    if (m->duty_cmd > DUTY_MAX) m->duty_cmd = DUTY_MAX;
    if (m->duty_cmd < DUTY_MIN_ACTIVE) m->duty_cmd = 0.0f;

    if (m->dir_state == 0)
    {
        if (m->control_u > U_ON)       requested_dir = +1;
        else if (m->control_u < -U_ON) requested_dir = -1;
    }
    else if (m->dir_state == +1)
    {
        if (m->control_u < U_OFF)      requested_dir = 0;
        else if (m->control_u < -U_ON) requested_dir = -1;
        else                           requested_dir = +1;
    }
    else
    {
        if (m->control_u > -U_OFF)     requested_dir = 0;
        else if (m->control_u > U_ON)  requested_dir = +1;
        else                           requested_dir = -1;
    }

    m->dir_state = requested_dir;

    if (use_limit_switch != 0U)
    {
        limitSwitchRight = Read_Right_Limit_Switch();

        if ((limitSwitchRight != 0U) && (requested_dir < 0))
        {
            requested_dir = 0;
            m->dir_state = 0;
            m->applied_dir = 0;
            m->deadtimeTicks = 0U;
            m->duty_cmd = 0.0f;
            m->control_u = 0.0f;
            m->pid.integrator = 0.0f;
            m->pid.differentiator = 0.0f;
            m->pid.prevError = 0.0f;
            m->pid.prevMeasurement = m->actualAngle;
            m->absAngle = 0.0f;
            m->actualAngle = 0.0f;
            m->desiredAngle = 0.0f;
            Solenoid_Set(0U);
        }
    }

    if (m->deadtimeTicks > 0U)
    {
        m->deadtimeTicks--;
        m->applied_dir = 0;
    }
    else if (requested_dir != m->applied_dir)
    {
        if ((requested_dir != 0) && (m->applied_dir != 0))
        {
            m->applied_dir = 0;
            m->deadtimeTicks = DEADTIME_MS;
        }
        else
        {
            m->applied_dir = requested_dir;
        }
    }

    m->dir = m->applied_dir;

    if (m->dir == 0)
    {
        m->duty_cmd = 0.0f;
    }

    if (m->dir < 0)
    {
        PWM_SetDuty(m->pwm_fwd_tim, m->pwm_fwd_ch, m->duty_cmd);
        PWM_SetDuty(m->pwm_rev_tim, m->pwm_rev_ch, 0.0f);
    }
    else if (m->dir > 0)
    {
        PWM_SetDuty(m->pwm_fwd_tim, m->pwm_fwd_ch, 0.0f);
        PWM_SetDuty(m->pwm_rev_tim, m->pwm_rev_ch, m->duty_cmd);
    }
    else
    {
        PWM_SetDuty(m->pwm_fwd_tim, m->pwm_fwd_ch, 0.0f);
        PWM_SetDuty(m->pwm_rev_tim, m->pwm_rev_ch, 0.0f);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  Start_Peripherals();
  Run_Homing_Routine();

  uint32_t nextPrintTick = HAL_GetTick();

  while (1)
  {
      uint32_t now = HAL_GetTick();

      if (homingRequest != 0U)
      {
          homingRequest = 0U;
          Run_Homing_Routine();
      }

      if ((uint32_t)(now - nextPrintTick) >= TELEMETRY_MS)
      {
          nextPrintTick = now;
          Send_Telemetry();
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;

  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;

  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /* PA0 = limit switch, PA1 spare/optional */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char c = (char)rxByte;

        if (c == '!' || c == '\n' || c == '\r')
        {
            if (rxIdx > 0U)
            {
                rxLine[rxIdx] = '\0';
                VOFA_ParseCommand(rxLine);
                rxIdx = 0U;
            }
        }
        else
        {
            if (rxIdx < (sizeof(rxLine) - 1U))
            {
                rxLine[rxIdx++] = c;
            }
            else
            {
                rxIdx = 0U;
            }
        }

        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LIMIT_SW_Pin)
    {
        limitSwitchRight = Read_Right_Limit_Switch();

        if (homingActive != 0U)
        {
            return;
        }

        if ((limitSwitchRight != 0U) && (motor1.dir < 0))
        {
            PWM_SetDuty(&htim1, TIM_CHANNEL_1, 0.0f);
            PWM_SetDuty(&htim1, TIM_CHANNEL_4, 0.0f);
            Reset_Motor_PID_State(&motor1);
            motor1.absAngle = 0.0f;
            motor1.actualAngle = 0.0f;
            motor1.desiredAngle = 0.0f;
            Solenoid_Set(0U);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uartTxTail = (uint16_t)((uartTxTail + uartTxChunkLen) % UART_TX_BUF_SIZE);

        if (uartTxCount >= uartTxChunkLen)
        {
            uartTxCount = (uint16_t)(uartTxCount - uartTxChunkLen);
        }
        else
        {
            uartTxCount = 0U;
        }

        uartTxChunkLen = 0U;
        uartTxBusy = 0U;
        UART_TxKickoff();
    }
}

void Control_Loop_1kHz(void)
{
    if (controlEnabled == 0U)
    {
        return;
    }

    Motor_Control_Update(&motor1, COUNTS_PER_REV_M1, 1U);
    Motor_Control_Update(&motor2, COUNTS_PER_REV_M2, 0U);
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
