/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : F411 1kHz CF
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "sr595.h"
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

/* Controller parameters - Motor 1 */
#define PID_KP           0.7915f
#define PID_KI           1.0f
#define PID_KD           0.0f
#define PID_TAU          0.02f

#define PID_LIM_MIN     -10.0f
#define PID_LIM_MAX      10.0f
#define PID_LIM_MIN_INT  -5.0f
#define PID_LIM_MAX_INT   5.0f

/* Controller parameters - Motor 2 */
#define PID2_KP           1.5f
#define PID2_KI           0.0f
#define PID2_KD           0.04f
#define PID2_TAU          0.2f

#define PID2_LIM_MIN     -10.0f
#define PID2_LIM_MAX      10.0f
#define PID2_LIM_MIN_INT  -5.0f
#define PID2_LIM_MAX_INT   5.0f

#define SAMPLE_TIME_S    0.001f
#define TELEMETRY_MS     20U

/* Encoder */
#define COUNTS_PER_REV   700.0f

/* H-bridge PWM mapping on the F411 version */
#define PWM_REV_TIM      htim1
#define PWM_REV_CH       TIM_CHANNEL_1   /* PA8  */
#define PWM_FWD_TIM      htim1
#define PWM_FWD_CH       TIM_CHANNEL_4   /* PA11 */

/* ---------- Motor 2 ---------- */
#define COUNTS_PER_REV_M2   700.0f

#define M2_PWM_REV_TIM      htim2
#define M2_PWM_REV_CH       TIM_CHANNEL_1

#define M2_PWM_FWD_TIM      htim2
#define M2_PWM_FWD_CH       TIM_CHANNEL_3

/* Solenoid shift-register control */
#define NUM_SOLENOIDS        5U
#define SL1_MASK             ((uint8_t)(1U << 0))   /* QA */
#define SL2_MASK             ((uint8_t)(1U << 4))   /* QE */
#define SL3_MASK             ((uint8_t)(1U << 3))   /* QD */
#define SL4_MASK             ((uint8_t)(1U << 2))   /* QC */
#define SL5_SPECIAL_MASK     ((uint8_t)0xE2)        /* QB + QF + QG + QH */

/* Limit switch */
#define LIMIT_SW_GPIO_Port GPIOB
#define LIMIT_SW_Pin       GPIO_PIN_12

/* Hysteresis / safety */
#define U_ON              0.30f
#define U_OFF             0.05f
#define DUTY_MAX          0.38f
#define DUTY_MIN_ACTIVE   0.15f
#define DEADTIME_MS       50U

#define DUTY_MAX2         0.5f

#define HOMING_MOVE_AWAY_DUTY   0.4f
#define HOMING_SEEK_DUTY        0.35f
#define HOMING_MOVE_AWAY_MS     250U
#define HOMING_SETTLE_MS         30U
#define HOMING_TIMEOUT_MS      4000U


#define HOMING2_SEEK_DUTY           0.4f
#define HOMING2_BACKOFF_DUTY        0.25f
#define HOMING2_WINDOW_MS           50U
#define HOMING2_STARTUP_IGNORE_MS   300U
#define HOMING2_STALL_COUNTS        1
#define HOMING2_STALL_CONFIRM_MS    300U
#define HOMING2_BACKOFF_COUNTS      20U
#define HOMING2_SETTLE_MS           30U

//homing limit
#define HOMING2_MAX_SEEK_MS      3000U
#define HOMING2_MAX_BACKOFF_MS   1500U

#define SPREAD_LEVEL_MIN     0
#define SPREAD_LEVEL_MAX     4

static const float kSpreadAngleTable[5] =
{
    0.0f,   /* level 0 */
    40.0f,  /* level 1 */
    80.0f,  /* level 2 */
    120.0f, /* level 3 */
    160.0f  /* level 4 */
};
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
volatile float desiredAngle = 60.0f;
volatile uint8_t solenoidState = 0;
volatile uint8_t limitSwitchRight = 0;

/* ---------- Motor 1 ---------- */
volatile float actualAngle = 0.0f;
volatile float duty_cmd = 0.0f;
volatile float control_u = 0.0f;
volatile int8_t dir = 0;

PIDController pid =
{
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
};

static uint8_t rxByte;
static char rxLine[32];
static uint8_t rxIdx = 0;

static volatile int8_t dir_state = 0;
static volatile int8_t applied_dir = 0;
static volatile uint16_t deadtimeTicks = 0U;

/* ---------- Motor 2 ---------- */
volatile float desiredAngle2 = 0.0f;
volatile float actualAngle2 = 0.0f;
volatile float duty_cmd2 = 0.0f;
volatile float control_u2 = 0.0f;
volatile int8_t dir2 = 0;

PIDController pid2 =
{
    PID2_KP,
    PID2_KI,
    PID2_KD,
    PID2_TAU,
    PID2_LIM_MIN,
    PID2_LIM_MAX,
    PID2_LIM_MIN_INT,
    PID2_LIM_MAX_INT,
    SAMPLE_TIME_S,
    0.0f,
    0.0f
};

static volatile int8_t dir_state2 = 0;
static volatile int8_t applied_dir2 = 0;
static volatile uint16_t deadtimeTicks2 = 0U;
/* -------------------------*/

static volatile int32_t prevRawCount2 = 0;
static volatile float absAngle2 = 0.0f;

static volatile uint8_t controlEnabled = 0U;

static volatile uint8_t homingActive = 0U;
static volatile uint8_t homingRequest = 0U;

static volatile int32_t prevRawCount = 0;
static volatile float absAngle = 0.0f;

#define UART_TX_BUF_SIZE 512U
static uint8_t uartTxBuf[UART_TX_BUF_SIZE];
static volatile uint16_t uartTxHead = 0U;
static volatile uint16_t uartTxTail = 0U;
static volatile uint16_t uartTxCount = 0U;
static volatile uint16_t uartTxChunkLen = 0U;
static volatile uint8_t uartTxBusy = 0U;

static char printMessage[128];

/* SR595 state */
SR595_HandleTypeDef hsr;
static uint8_t solenoidMask = 0U;

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
static void Solenoid_WriteMask(uint8_t mask);
static void Solenoid_Select(uint8_t selection);
static inline void Solenoid_Set(uint8_t on);
static uint8_t Solenoid_ParseExactMask(const char *s, uint8_t *outMask);
static inline uint8_t Read_Right_Limit_Switch(void);
static void UART_TxKickoff(void);
static void UART_QueueBytes(const uint8_t *data, uint16_t len);
static void UART_QueueString(const char *s);
static void VOFA_ParseCommand(const char *line);
static void Send_Telemetry(void);
static void Start_Peripherals(void);
static void Run_Homing_Routine(void);
static void home_motor1(void);
static void home_motor2(void);
void Control_Loop_1kHz(void);
static int32_t RoundFloatToInt(float x);
static float SpreadLevelToAngle(int32_t level);
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

static int32_t RoundFloatToInt(float x)
{
    if (x >= 0.0f)
    {
        return (int32_t)(x + 0.5f);
    }
    else
    {
        return (int32_t)(x - 0.5f);
    }
}

static float SpreadLevelToAngle(int32_t level)
{
    if (level < SPREAD_LEVEL_MIN)
    {
        level = SPREAD_LEVEL_MIN;
    }
    if (level > SPREAD_LEVEL_MAX)
    {
        level = SPREAD_LEVEL_MAX;
    }

    return kSpreadAngleTable[level];
}

static void Solenoid_WriteMask(uint8_t mask)
{
    /* Board is active-low:
       logical 1 = solenoid ON
       hardware needs 0 on that output to turn it ON
    */
	SR595_Write(&hsr, mask);
}

static void Solenoid_Select(uint8_t selection)
{
    uint8_t newMask = 0U;

    if (selection == 0U)
    {
        solenoidState = 0U;
        solenoidMask  = 0U;
        Solenoid_WriteMask(solenoidMask);
    }
    else if (selection == 1U)
    {
        newMask = SL1_MASK;
        solenoidState = 1U;
        solenoidMask  = newMask;
        Solenoid_WriteMask(solenoidMask);
    }
    else if (selection == 2U)
    {
        newMask = SL2_MASK;
        solenoidState = 1U;
        solenoidMask  = newMask;
        Solenoid_WriteMask(solenoidMask);
    }
    else if (selection == 3U)
    {
        newMask = SL3_MASK;
        solenoidState = 1U;
        solenoidMask  = newMask;
        Solenoid_WriteMask(solenoidMask);
    }
    else if (selection == 4U)
    {
        newMask = SL4_MASK;
        solenoidState = 1U;
        solenoidMask  = newMask;
        Solenoid_WriteMask(solenoidMask);
    }
    else if (selection == 5U)
    {
        newMask = SL5_SPECIAL_MASK;
        solenoidState = 1U;
        solenoidMask  = newMask;
        Solenoid_WriteMask(solenoidMask);
    }
}

static uint8_t Solenoid_ParseExactMask(const char *s, uint8_t *outMask)
{
    uint8_t mask = 0U;
    uint8_t sawDigit = 0U;

    if ((s == NULL) || (outMask == NULL))
    {
        return 0U;
    }

    while (*s != '\0')
    {
        char c = *s++;
        sawDigit = 1U;

        switch (c)
        {
            case '0':
                if (*s != '\0' || mask != 0U)
                {
                    return 0U;
                }
                mask = 0U;
                break;

            case '1':
                mask |= SL1_MASK;
                break;

            case '2':
                mask |= SL2_MASK;
                break;

            case '3':
                mask |= SL3_MASK;
                break;

            case '4':
                mask |= SL4_MASK;
                break;

            case '5':
                mask |= SL5_SPECIAL_MASK;
                break;

            default:
                return 0U;
        }
    }

    if (sawDigit == 0U)
    {
        return 0U;
    }

    *outMask = mask;
    return 1U;
}

static inline void Solenoid_Set(uint8_t on)
{
    if (on != 0U)
    {
        Solenoid_Select(1U);
    }
    else
    {
        Solenoid_Select(0U);
    }
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

    /* If whole message cannot fit, drop the whole message */
    if ((uint16_t)(uartTxCount + len) > UART_TX_BUF_SIZE)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
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

static void VOFA_ParseCommand(const char *line)
{
    const char *eq = strchr(line, '=');
    if (eq == NULL)
    {
        return;
    }

    float val = strtof(eq + 1, NULL);

    size_t prefixLen = (size_t)(eq - line);
    char prefix[8] = {0};

    if (prefixLen >= sizeof(prefix))
    {
        return;
    }

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
        uint8_t newMask = 0U;
        char roundedStr[16];
        char *endptr = NULL;
        float rawSel;
        long roundedSel;

        rawSel = strtof(eq + 1, &endptr);

        if ((endptr == (eq + 1)) || ((*endptr != '\0') && (*endptr != '\r') && (*endptr != '\n')))
        {
            snprintf(ack, sizeof(ack), "ERR SL format\r\n");
        }
        else
        {
            /* Round float from VOFA to nearest integer */
            if (rawSel >= 0.0f)
            {
                roundedSel = (long)(rawSel + 0.5f);
            }
            else
            {
                roundedSel = (long)(rawSel - 0.5f);
            }

            if ((roundedSel < 0L) || (roundedSel > 55555L))
            {
                snprintf(ack, sizeof(ack), "ERR SL range\r\n");
            }
            else
            {
                snprintf(roundedStr, sizeof(roundedStr), "%ld", roundedSel);

                if (Solenoid_ParseExactMask(roundedStr, &newMask) != 0U)
                {
                    solenoidMask  = newMask;
                    solenoidState = (newMask != 0U) ? 1U : 0U;
                    Solenoid_WriteMask(solenoidMask);
                    snprintf(ack, sizeof(ack), "ACK SL=%s\r\n", roundedStr);
                }
                else
                {
                    snprintf(ack, sizeof(ack), "ERR SL format\r\n");
                }
            }
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
    else if (strcmp(prefix, "KP2") == 0)
    {
        pid2.Kp = val;
        snprintf(ack, sizeof(ack), "ACK KP2=%.5f\r\n", pid2.Kp);
    }
    else if (strcmp(prefix, "KI2") == 0)
    {
        pid2.Ki = val;
        pid2.integrator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KI2=%.5f\r\n", pid2.Ki);
    }
    else if (strcmp(prefix, "KD2") == 0)
    {
        pid2.Kd = val;
        pid2.differentiator = 0.0f;
        snprintf(ack, sizeof(ack), "ACK KD2=%.5f\r\n", pid2.Kd);
    }
    else if (strcmp(prefix, "TAU2") == 0)
    {
        pid2.tau = val;
        snprintf(ack, sizeof(ack), "ACK TAU2=%.5f\r\n", pid2.tau);
    }
    else if (strcmp(prefix, "SP2") == 0)
{
    desiredAngle2 = val;
    snprintf(ack, sizeof(ack), "ACK SP2=%.3f\r\n", desiredAngle2);
}
	else if (strcmp(prefix, "SF") == 0)
	{
			int32_t spreadLevel = RoundFloatToInt(val);

			if ((spreadLevel < SPREAD_LEVEL_MIN) || (spreadLevel > SPREAD_LEVEL_MAX))
			{
					snprintf(
							ack,
							sizeof(ack),
							"ERR SF range (%d..%d)\r\n",
							SPREAD_LEVEL_MIN,
							SPREAD_LEVEL_MAX
					);
			}
			else
			{
					desiredAngle2 = SpreadLevelToAngle(spreadLevel);

					snprintf(
							ack,
							sizeof(ack),
							"ACK SF=%ld -> SP2=%.3f\r\n",
							(long)spreadLevel,
							desiredAngle2
					);
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
        desiredAngle,
        actualAngle,
        duty_cmd,
        control_u,
        dir,
        desiredAngle2,
        actualAngle2,
        duty_cmd2,
        control_u2,
        dir2,
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
    PIDController_Init(&pid);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&PWM_REV_TIM, PWM_REV_CH);
    HAL_TIM_PWM_Start(&PWM_FWD_TIM, PWM_FWD_CH);

    PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);

    prevRawCount = (int32_t)TIM4->CNT;
    absAngle = 0.0f;
    dir_state = 0;
    applied_dir = 0;
    dir = 0;
    deadtimeTicks = 0U;

    HAL_UART_Receive_IT(&huart2, &rxByte, 1);

    /* SR595 pin mapping:
     PB0 = data
     PB1 = clock
     PC5 = latch
    */
    hsr.data_port  = GPIOB;
    hsr.data_pin   = GPIO_PIN_0;
    hsr.clk_port   = GPIOB;
    hsr.clk_pin    = GPIO_PIN_1;
    hsr.latch_port = GPIOC;
    hsr.latch_pin  = GPIO_PIN_5;

    SR595_Init(&hsr);
    solenoidMask = 0U;
    Solenoid_WriteMask(solenoidMask);

    controlEnabled = 1U;

    PIDController_Init(&pid2);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&M2_PWM_REV_TIM, M2_PWM_REV_CH);
    HAL_TIM_PWM_Start(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH);

    PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);
    PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);

    prevRawCount2 = (int32_t)TIM3->CNT;
    absAngle2 = 0.0f;
    dir_state2 = 0;
    applied_dir2 = 0;
    dir2 = 0;
    deadtimeTicks2 = 0U;
}

static void Run_Homing_Routine(void)
{
    homingActive = 1U;
    Send_Telemetry();

    controlEnabled = 0U;

    /* Stop both motors before homing */
    PWM_SetDuty(&PWM_FWD_TIM,    PWM_FWD_CH,    0.0f);
    PWM_SetDuty(&PWM_REV_TIM,    PWM_REV_CH,    0.0f);
    PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);

    /* Home motor 2 first, then motor 1 */
    home_motor2();
    home_motor1();

    homingActive = 0U;
    controlEnabled = 1U;

    Send_Telemetry();
}

static void home_motor1(void)
{
    PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);

    pid.integrator      = 0.0f;
    pid.differentiator  = 0.0f;
    pid.prevError       = 0.0f;
    pid.prevMeasurement = actualAngle;
    control_u           = 0.0f;
    duty_cmd            = 0.0f;
    dir_state           = 0;
    applied_dir         = 0;
    deadtimeTicks       = 0U;
    limitSwitchRight    = Read_Right_Limit_Switch();

    /* If already sitting on the home switch, back off first so the next hit is real. */
    dir = -1;
    PWM_SetDuty(&PWM_FWD_TIM, PWM_REV_CH, HOMING_MOVE_AWAY_DUTY);
    PWM_SetDuty(&PWM_REV_TIM, PWM_FWD_CH, 0.0f);
    HAL_Delay(HOMING_MOVE_AWAY_MS);

    PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    HAL_Delay(HOMING_SETTLE_MS);

    /* Seek toward the home switch. No timeout. */
    dir = +1;

    while (Read_Right_Limit_Switch() == 0U)
    {
        PWM_SetDuty(&PWM_REV_TIM, PWM_FWD_CH, HOMING_SEEK_DUTY);
        PWM_SetDuty(&PWM_FWD_TIM, PWM_REV_CH, 0.0f);
        HAL_Delay(1);
    }
    PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    HAL_Delay(500);

    PWM_SetDuty(&PWM_FWD_TIM, PWM_REV_CH, HOMING_MOVE_AWAY_DUTY);
    PWM_SetDuty(&PWM_REV_TIM, PWM_FWD_CH, 0.0f);
    HAL_Delay(50);

    PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
    HAL_Delay(500);

    TIM4->CNT            = 0U;
    prevRawCount         = 0;
    absAngle             = 0.0f;
    actualAngle          = 0.0f;
    desiredAngle         = 0.0f;
    control_u            = 0.0f;
    duty_cmd             = 0.0f;
    pid.integrator       = 0.0f;
    pid.differentiator   = 0.0f;
    pid.prevError        = 0.0f;
    pid.prevMeasurement  = 0.0f;
    dir_state            = 0;
    applied_dir          = 0;
    dir                  = 0;
    deadtimeTicks        = 0U;
    limitSwitchRight     = Read_Right_Limit_Switch();
    Solenoid_Set(0U);

    HAL_Delay(100);
}

static void home_motor2(void)
{
    uint32_t startTick;
    uint32_t windowStart;
    uint32_t stallStartTick;

    int32_t rawCount2;
    int32_t delta2;
    int32_t absDelta2;
    int32_t windowMotionCounts;

    float backoffTargetAngle;

    PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);

    pid2.integrator      = 0.0f;
    pid2.differentiator  = 0.0f;
    pid2.prevError       = 0.0f;
    pid2.prevMeasurement = actualAngle2;
    control_u2           = 0.0f;
    duty_cmd2            = 0.0f;
    dir_state2           = 0;
    applied_dir2         = 0;
    deadtimeTicks2       = 0U;

    prevRawCount2 = (int32_t)TIM3->CNT;

    /* decreasing actualAngle2 means going toward home */
    dir2 = -1;
    startTick = HAL_GetTick();
		windowStart = startTick;
		stallStartTick = 0U;
		windowMotionCounts = 0;

		while (1)
		{
				if ((HAL_GetTick() - startTick) >= HOMING2_MAX_SEEK_MS)
				{
						break;
				}

				PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, HOMING2_SEEK_DUTY);
				PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);

				rawCount2 = (int32_t)TIM3->CNT;

				delta2 = rawCount2 - prevRawCount2;
				if (delta2 > 32767)  delta2 -= 65536;
				if (delta2 < -32767) delta2 += 65536;
				prevRawCount2 = rawCount2;

				absAngle2 += (360.0f / COUNTS_PER_REV_M2) * (float)delta2;
				actualAngle2 = absAngle2;

				absDelta2 = delta2;
				if (absDelta2 < 0) absDelta2 = -absDelta2;
				windowMotionCounts += absDelta2;

				if ((HAL_GetTick() - windowStart) >= HOMING2_WINDOW_MS)
				{
						if ((HAL_GetTick() - startTick) >= HOMING2_STARTUP_IGNORE_MS)
						{
								if (windowMotionCounts <= HOMING2_STALL_COUNTS)
								{
										if (stallStartTick == 0U)
										{
												stallStartTick = HAL_GetTick();
										}

										if ((HAL_GetTick() - stallStartTick) >= HOMING2_STALL_CONFIRM_MS)
										{
												break;
										}
								}
								else
								{
										stallStartTick = 0U;
								}
						}

						windowMotionCounts = 0;
						windowStart = HAL_GetTick();
				}

				HAL_Delay(1);
		}

    PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);
    dir2 = 0;

    HAL_Delay(HOMING2_SETTLE_MS);

    /* Back off slightly from the wall */
    backoffTargetAngle = actualAngle2 +
        ((360.0f / COUNTS_PER_REV_M2) * (float)HOMING2_BACKOFF_COUNTS);

    prevRawCount2 = (int32_t)TIM3->CNT;
    dir2 = +1;
		uint32_t backoffStartTick = HAL_GetTick();

		while (actualAngle2 < backoffTargetAngle)
		{
				if ((HAL_GetTick() - backoffStartTick) >= HOMING2_MAX_BACKOFF_MS)
				{
						break;
				}

				PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);
				PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, HOMING2_BACKOFF_DUTY);

				rawCount2 = (int32_t)TIM3->CNT;

				delta2 = rawCount2 - prevRawCount2;
				if (delta2 > 32767)  delta2 -= 65536;
				if (delta2 < -32767) delta2 += 65536;
				prevRawCount2 = rawCount2;

				absAngle2 += (360.0f / COUNTS_PER_REV_M2) * (float)delta2;
				actualAngle2 = absAngle2;

				HAL_Delay(1);
		}

    PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);
    PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);
    dir2 = 0;

    HAL_Delay(HOMING2_SETTLE_MS);

    TIM3->CNT            = 0U;
    prevRawCount2        = 0;
    absAngle2            = 0.0f;
    actualAngle2         = 0.0f;
    desiredAngle2        = 0.0f;
    control_u2           = 0.0f;
    duty_cmd2            = 0.0f;
    pid2.integrator      = 0.0f;
    pid2.differentiator  = 0.0f;
    pid2.prevError       = 0.0f;
    pid2.prevMeasurement = 0.0f;
    dir_state2           = 0;
    applied_dir2         = 0;
    dir2                 = 0;
    deadtimeTicks2       = 0U;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  Start_Peripherals();
  Run_Homing_Routine();

  uint32_t nextCtrlTick  = HAL_GetTick();
  uint32_t nextPrintTick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (homingRequest != 0U)
    {
      homingRequest = 0U;
      Run_Homing_Routine();
    }

    if ((uint32_t)(now - nextCtrlTick) >= 1U)
    {
      nextCtrlTick += 1U;
      Control_Loop_1kHz();
    }

    if ((uint32_t)(now - nextPrintTick) >= TELEMETRY_MS)
    {
      nextPrintTick += TELEMETRY_MS;
      Send_Telemetry();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

        if ((limitSwitchRight != 0U) && (dir < 0))
        {
            PWM_SetDuty(&PWM_FWD_TIM, PWM_FWD_CH, 0.0f);
            PWM_SetDuty(&PWM_REV_TIM, PWM_REV_CH, 0.0f);
            applied_dir = 0;
            dir_state = 0;
            dir = 0;
            duty_cmd = 0.0f;
            control_u = 0.0f;
            deadtimeTicks = 0U;
            pid.integrator = 0.0f;
            pid.differentiator = 0.0f;
            pid.prevError = 0.0f;
            pid.prevMeasurement = actualAngle;
            absAngle = 0.0f;
            actualAngle = 0.0f;
            desiredAngle = 0.0f;
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
    /* =========================
       Motor 1
       ========================= */
    int32_t rawCount;
    int32_t delta;
    int8_t requested_dir = 0;

    if (controlEnabled == 0U)
    {
        return;
    }

    rawCount = (int32_t)TIM4->CNT;

    delta = rawCount - prevRawCount;
    if (delta > 32767)  delta -= 65536;
    if (delta < -32767) delta += 65536;
    prevRawCount = rawCount;

    absAngle += (360.0f / COUNTS_PER_REV) * (float)delta;
    actualAngle = absAngle;

    control_u = PIDController_Update(&pid, desiredAngle, actualAngle);

    duty_cmd = fabsf(control_u) / pid.limMax;
    if (duty_cmd > DUTY_MAX) duty_cmd = DUTY_MAX;
    if (duty_cmd < DUTY_MIN_ACTIVE) duty_cmd = 0.0f;

    if (dir_state == 0)
    {
        if (control_u > U_ON)       requested_dir = +1;
        else if (control_u < -U_ON) requested_dir = -1;
    }
    else if (dir_state == +1)
    {
        if (control_u < U_OFF)      requested_dir = 0;
        else if (control_u < -U_ON) requested_dir = -1;
        else                        requested_dir = +1;
    }
    else
    {
        if (control_u > -U_OFF)     requested_dir = 0;
        else if (control_u > U_ON)  requested_dir = +1;
        else                        requested_dir = -1;
    }

    dir_state = requested_dir;
    limitSwitchRight = Read_Right_Limit_Switch();

    if ((limitSwitchRight != 0U) && (requested_dir < 0))
    {
        requested_dir       = 0;
        dir_state           = 0;
        applied_dir         = 0;
        deadtimeTicks       = 0U;
        duty_cmd            = 0.0f;
        control_u           = 0.0f;
        pid.integrator      = 0.0f;
        pid.differentiator  = 0.0f;
        pid.prevError       = 0.0f;
        pid.prevMeasurement = actualAngle;
        absAngle            = 0.0f;
        actualAngle         = 0.0f;
        desiredAngle        = 0.0f;
        Solenoid_Set(0U);
    }

    if (deadtimeTicks > 0U)
    {
        deadtimeTicks--;
        applied_dir = 0;
    }
    else if (requested_dir != applied_dir)
    {
        if ((requested_dir != 0) && (applied_dir != 0))
        {
            applied_dir = 0;
            deadtimeTicks = DEADTIME_MS;
        }
        else
        {
            applied_dir = requested_dir;
        }
    }

    dir = applied_dir;

    if (dir == 0)
    {
        duty_cmd = 0.0f;
    }

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

    /* =========================
       Motor 2
       TIM3 CH1/CH2 = encoder
       TIM2 CH1/CH3 = PWM
       ========================= */
    {
        int32_t rawCount2;
        int32_t delta2;
        int8_t requested_dir2 = 0;

        rawCount2 = (int32_t)TIM3->CNT;

        delta2 = rawCount2 - prevRawCount2;
        if (delta2 > 32767)  delta2 -= 65536;
        if (delta2 < -32767) delta2 += 65536;
        prevRawCount2 = rawCount2;

        absAngle2 += (360.0f / COUNTS_PER_REV_M2) * (float)delta2;
        actualAngle2 = absAngle2;

        control_u2 = PIDController_Update(&pid2, desiredAngle2, actualAngle2);

        duty_cmd2 = fabsf(control_u2) / pid2.limMax;
        if (duty_cmd2 > DUTY_MAX) duty_cmd2 = DUTY_MAX;
        if (duty_cmd2 < DUTY_MIN_ACTIVE) duty_cmd2 = 0.0f;

        if (dir_state2 == 0)
        {
            if (control_u2 > U_ON)       requested_dir2 = +1;
            else if (control_u2 < -U_ON) requested_dir2 = -1;
        }
        else if (dir_state2 == +1)
        {
            if (control_u2 < U_OFF)      requested_dir2 = 0;
            else if (control_u2 < -U_ON) requested_dir2 = -1;
            else                         requested_dir2 = +1;
        }
        else
        {
            if (control_u2 > -U_OFF)     requested_dir2 = 0;
            else if (control_u2 > U_ON)  requested_dir2 = +1;
            else                         requested_dir2 = -1;
        }

        dir_state2 = requested_dir2;

        if (deadtimeTicks2 > 0U)
        {
            deadtimeTicks2--;
            applied_dir2 = 0;
        }
        else if (requested_dir2 != applied_dir2)
        {
            if ((requested_dir2 != 0) && (applied_dir2 != 0))
            {
                applied_dir2 = 0;
                deadtimeTicks2 = DEADTIME_MS;
            }
            else
            {
                applied_dir2 = requested_dir2;
            }
        }

        dir2 = applied_dir2;

        if (dir2 == 0)
        {
            duty_cmd2 = 0.0f;
        }

        if (dir2 < 0)
        {
            PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, duty_cmd2);
            PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);
        }
        else if (dir2 > 0)
        {
            PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);
            PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, duty_cmd2);
        }
        else
        {
            PWM_SetDuty(&M2_PWM_FWD_TIM, M2_PWM_FWD_CH, 0.0f);
            PWM_SetDuty(&M2_PWM_REV_TIM, M2_PWM_REV_CH, 0.0f);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */