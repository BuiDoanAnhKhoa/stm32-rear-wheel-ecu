/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Rear STM32 ECU - Dual Rear Motor PID + CAN VCU Interface
  *
  * RTOS architecture:
  *   - CAN RX ISR only copies raw bxCAN FIFO0 frames into a static RTOS queue.
  *   - CanRxTask parses ID 0x040 steering angle commands and updates shared state.
  *   - MotorControlTask runs every 10 ms and owns encoder/PID/PWM control.
  *   - CanTxTask sends average rear speed feedback every 50 ms on CAN ID 0x020.
  *   - DebugTask prints compact USART2 telemetry every 500 ms.
  *
  * CAN protocol used by this file:
  *   RX 0x040, DLC=2: target_angle_cdeg, int16_t, little-endian
  *   TX 0x020, DLC=2: rear_avg_speed_mmps, int16_t, little-endian
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prev_meas;
  float output;
} PID_t;

typedef struct
{
  float angle_deg;
  uint8_t valid;
  uint32_t tick_ms;
} AngleCmdState_t;

typedef struct
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
} CanRxEntry_t;

typedef struct
{
  float angle_deg;
  uint8_t angle_fresh;
  float v_ref_base_mps;
  float v_ref_left_mps;
  float v_ref_right_mps;
  float v_left_mps;
  float v_right_mps;
  float pwm_left_pct;
  float pwm_right_pct;
} RearDebugSnapshot_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ========================= Mechanical constants ========================= */
#define ENC_PPR                         1560.0f
#define WHEEL_DIAMETER_M                0.065f
#define WHEEL_CIRCUMFERENCE_M           (3.1415926f * WHEEL_DIAMETER_M)

/* ========================= Timing ========================= */
#define SAMPLE_TIME_MS                  10U     /* 100 Hz control loop */
#define CAN_SEND_TIME_MS                50U     /* 20 Hz speed feedback */
#define DEBUG_TIME_MS                   500U    /* 2 Hz Hercules debug */
#define ANGLE_TIMEOUT_MS                200U    /* failsafe if VCU command lost */

#define TS_SEC                          (SAMPLE_TIME_MS / 1000.0f)
#define INV_TS_SEC                      (1000.0f / SAMPLE_TIME_MS)

/* ========================= CAN IDs ========================= */
#define CAN_ID_STEERING_CMD_FROM_VCU    0x040U  /* RX: target steering angle, cdeg */
#define CAN_ID_VAVG_FEEDBACK            0x020U  /* TX: rear average speed, mm/s */

/* ========================= Fixed-point CAN scaling ========================= */
#define ANGLE_SCALE_CDEG_PER_DEG        100.0f
#define SPEED_SCALE_MMPS_PER_MPS        1000.0f

/* ========================= Angle limits ========================= */
#define ANGLE_CENTER_DEG                90.0f
#define ANGLE_MIN_DEG                   50.0f
#define ANGLE_MAX_DEG                   130.0f
#define ANGLE_FULL_SCALE_DEG            40.0f
#define ANGLE_CENTER_DEADBAND_DEG       4.0f

/* ========================= Speed scheduling ========================= */
#define VREF_MAX_MPS                    0.30f
#define VREF_MIN_MPS                    0.10f
#define VREF_RAMP_UP_MPS_PER_S          2.0f
#define VREF_RAMP_DOWN_MPS_PER_S        4.0f
#define VREF_STOP_THRESHOLD             0.01f

/* ========================= Electronic differential ========================= */
#define DIFF_MAX_RATIO                  0.1f

/* ========================= Encoder filtering ========================= */
#define SPEED_FILTER_ALPHA              0.30f
#define SPEED_FILTER_BETA               (1.0f - SPEED_FILTER_ALPHA)
#define ENC_TO_MPS                      (WHEEL_CIRCUMFERENCE_M * INV_TS_SEC / ENC_PPR)

/* Change sign if one encoder reports reversed direction */
#define LEFT_SIGN                       (+1.0f)
#define RIGHT_SIGN                      (+1.0f)

/* ========================= PID ========================= */
#define PID_LEFT_KP                     150.0f
#define PID_LEFT_KI                     40.0f
#define PID_LEFT_KD                     0.0f

#define PID_RIGHT_KP                    180.0f
#define PID_RIGHT_KI                    55.0f
#define PID_RIGHT_KD                    0.0f

#define INTEGRAL_CLAMP                  2.0f
#define OUTPUT_CLAMP_PCT                100.0f

/* ========================= PWM ========================= */
#define PWM_TIMER_PERIOD                7199U
#define PWM_MAX_COUNTS                  ((float)PWM_TIMER_PERIOD)
#define MIN_EFFECTIVE_PWM_LEFT_PCT   12.0f
#define MIN_EFFECTIVE_PWM_RIGHT_PCT  28.0f
#define INV_100                         0.01f

/* ========================= RTOS ========================= */
#define CAN_RX_QUEUE_SIZE               4U

#define CAN_RX_TASK_STACK_SIZE          384U
#define MOTOR_TASK_STACK_SIZE           512U
#define CAN_TX_TASK_STACK_SIZE          384U
#define DEBUG_TASK_STACK_SIZE           1536U

/* ========================= Debug ========================= */
#define DEBUG_BUF_LEN                   96U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef CLAMP
#define CLAMP(x, lo, hi)   (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static PID_t pid_left =
{
  .Kp = PID_LEFT_KP,
  .Ki = PID_LEFT_KI,
  .Kd = PID_LEFT_KD,
  .integral = 0.0f,
  .prev_meas = 0.0f,
  .output = 0.0f
};

static PID_t pid_right =
{
  .Kp = PID_RIGHT_KP,
  .Ki = PID_RIGHT_KI,
  .Kd = PID_RIGHT_KD,
  .integral = 0.0f,
  .prev_meas = 0.0f,
  .output = 0.0f
};

static int16_t prev_cnt_left = 0;
static int16_t prev_cnt_right = 0;

static volatile float v_left_mps = 0.0f;
static volatile float v_right_mps = 0.0f;
static volatile float v_avg_mps = 0.0f;

static volatile float v_ref_target_mps = 0.0f;
static volatile float v_ref_mps = 0.0f;
static volatile float v_ref_left_mps = 0.0f;
static volatile float v_ref_right_mps = 0.0f;

static volatile AngleCmdState_t g_angle_cmd =
{
  .angle_deg = ANGLE_CENTER_DEG,
  .valid = 0U,
  .tick_ms = 0U
};

static volatile uint8_t g_angle_cmd_reacquired = 0U;

static volatile uint32_t can_rx_ok_count             = 0U;
static volatile uint32_t can_rx_invalid_count        = 0U;
static volatile uint32_t can_rx_dlc_error_count      = 0U;
static volatile uint32_t can_rx_queue_overflow_count = 0U;

static volatile uint32_t can_tx_req_count            = 0U;
static volatile uint32_t can_tx_ok_count             = 0U;
static volatile uint32_t can_tx_err_count            = 0U;
static volatile uint32_t can_busoff_count            = 0U;
static volatile uint32_t can_last_error_code         = 0U;
static volatile uint32_t can_severe_error_count      = 0U;
static volatile uint8_t can_busoff_fault_pending     = 0U;

static char debug_buf[DEBUG_BUF_LEN];

static osMessageQueueId_t can_rx_queue_id = NULL;

static StaticQueue_t can_rx_queue_cb;
static uint8_t can_rx_queue_storage[CAN_RX_QUEUE_SIZE * sizeof(CanRxEntry_t)];

static osThreadId_t can_rx_task_id = NULL;
static osThreadId_t motor_task_id = NULL;
static osThreadId_t can_tx_task_id = NULL;
static osThreadId_t debug_task_id = NULL;

static StaticTask_t can_rx_task_cb;
static StaticTask_t motor_task_cb;
static StaticTask_t can_tx_task_cb;
static StaticTask_t debug_task_cb;

static StackType_t can_rx_task_stack[CAN_RX_TASK_STACK_SIZE / sizeof(StackType_t)];
static StackType_t motor_task_stack[MOTOR_TASK_STACK_SIZE / sizeof(StackType_t)];
static StackType_t can_tx_task_stack[CAN_TX_TASK_STACK_SIZE / sizeof(StackType_t)];
static StackType_t debug_task_stack[DEBUG_TASK_STACK_SIZE / sizeof(StackType_t)];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
static void CAN_Filter_Config(void);
static void CAN_SendSpeedMmps(uint16_t std_id, float speed_mps);
static void RearEcu_RtosInit(void);

static void CanRxTask(void *argument);
static void MotorControlTask(void *argument);
static void CanTxTask(void *argument);
static void DebugTask(void *argument);

static void UART2_Print(const char *s);
static void UART2_PrintStatus(const RearDebugSnapshot_t *snapshot);

static int16_t UnpackI16LE(const uint8_t *src);
static void PackI16LE(uint8_t *dst, int16_t value);
static int16_t ClampI16FromFloat(float x);

static float ComputeSpeedTargetFromAngle(float angle_deg);
static float ApplyRateLimiter(float current, float target, float up_rate, float down_rate, float Ts);
static void ApplyElectronicDifferential(float angle_deg, float base_ref_mps, volatile float *left_ref_mps, volatile float *right_ref_mps);
static void SetEqualWheelRefs(float base_ref_mps, volatile float *left_ref_mps, volatile float *right_ref_mps);
static void EncoderSpeed_Update(void);

static void PID_Reset(PID_t *pid);
static void PID_ResetAll(void);
static float PID_Update(PID_t *pid, float ref, float meas, float Ts);
static void SpeedControl_Reset(void);

static void Motor_SetCommand(TIM_HandleTypeDef *htim,uint32_t ch_fwd,uint32_t ch_rev,float cmd_percent,float min_pwm_pct);
static void Motor_All_Stop(void);

static void AngleCmd_Set(float angle_deg, uint32_t tick_ms);
static AngleCmdState_t AngleCmd_GetSnapshot(void);
static uint8_t AngleCmd_TakeReacquiredFlag(void);
static uint8_t IsAngleCmdFresh(uint32_t now, float *angle_out);
static RearDebugSnapshot_t DebugSnapshot_Get(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int16_t UnpackI16LE(const uint8_t *src)
{
  uint16_t raw = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
  return (int16_t)raw;
}

static void PackI16LE(uint8_t *dst, int16_t value)
{
  uint16_t raw = (uint16_t)value;
  dst[0] = (uint8_t)(raw & 0xFFU);
  dst[1] = (uint8_t)((raw >> 8) & 0xFFU);
}

static int16_t ClampI16FromFloat(float x)
{
  if (x > 32767.0f)
  {
    return 32767;
  }

  if (x < -32768.0f)
  {
    return -32768;
  }

  return (int16_t)lroundf(x);
}

static void UART2_Print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 10U);
}

static void UART2_PrintStatus(const RearDebugSnapshot_t *snapshot)
{
  snprintf(
      debug_buf,
      sizeof(debug_buf),
      "A=%.1f V=%.2f L=%.2f R=%.2f P=%.0f/%.0f\r\n",
      (double)snapshot->angle_deg,
      (double)snapshot->v_ref_base_mps,
      (double)snapshot->v_left_mps,
      (double)snapshot->v_right_mps,
      (double)snapshot->pwm_left_pct,
      (double)snapshot->pwm_right_pct);

  UART2_Print(debug_buf);
}

static void SpeedControl_Reset(void)
{
  PID_ResetAll();
  v_ref_target_mps = 0.0f;
  v_ref_mps        = 0.0f;
  v_ref_left_mps   = 0.0f;
  v_ref_right_mps  = 0.0f;
}

static void AngleCmd_Set(float angle_deg, uint32_t tick_ms)
{
  uint8_t was_invalid;

  taskENTER_CRITICAL();
  was_invalid = (g_angle_cmd.valid == 0U) ? 1U : 0U;
  g_angle_cmd.angle_deg = angle_deg;
  g_angle_cmd.valid     = 1U;
  g_angle_cmd.tick_ms   = tick_ms;

  if (was_invalid != 0U)
  {
    g_angle_cmd_reacquired = 1U;
  }
  taskEXIT_CRITICAL();
}

static AngleCmdState_t AngleCmd_GetSnapshot(void)
{
  AngleCmdState_t snap;

  taskENTER_CRITICAL();
  snap = g_angle_cmd;
  taskEXIT_CRITICAL();

  return snap;
}

static uint8_t AngleCmd_TakeReacquiredFlag(void)
{
  uint8_t flag;

  taskENTER_CRITICAL();
  flag = g_angle_cmd_reacquired;
  g_angle_cmd_reacquired = 0U;
  taskEXIT_CRITICAL();

  return flag;
}

static uint8_t IsAngleCmdFresh(uint32_t now, float *angle_out)
{
  AngleCmdState_t snap = AngleCmd_GetSnapshot();

  if (angle_out != NULL)
  {
    *angle_out = snap.angle_deg;
  }

  return ((snap.valid != 0U) && ((uint32_t)(now - snap.tick_ms) <= ANGLE_TIMEOUT_MS)) ? 1U : 0U;
}

static RearDebugSnapshot_t DebugSnapshot_Get(void)
{
  RearDebugSnapshot_t snapshot;

  taskENTER_CRITICAL();
  snapshot.angle_deg      = g_angle_cmd.angle_deg;
  snapshot.angle_fresh    = 0U;
  snapshot.v_ref_base_mps = v_ref_mps;
  snapshot.v_ref_left_mps = v_ref_left_mps;
  snapshot.v_ref_right_mps = v_ref_right_mps;
  snapshot.v_left_mps     = v_left_mps;
  snapshot.v_right_mps    = v_right_mps;
  snapshot.pwm_left_pct   = pid_left.output;
  snapshot.pwm_right_pct  = pid_right.output;
  taskEXIT_CRITICAL();

  snapshot.angle_fresh = IsAngleCmdFresh(HAL_GetTick(), NULL);

  return snapshot;
}

static void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  /* Accept only standard CAN ID 0x040 from VCU */
  filter.FilterIdHigh     = (CAN_ID_STEERING_CMD_FROM_VCU << 5);
  filter.FilterIdLow      = 0x0000U;
  filter.FilterMaskIdHigh = (0x7FFU << 5);
  filter.FilterMaskIdLow  = 0x0000U;

  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;

#if defined(CAN2)
  filter.SlaveStartFilterBank = 14;
#endif

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }
}

static void CAN_SendSpeedMmps(uint16_t std_id, float speed_mps)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8] = {0};
  uint32_t txMailbox;
  int16_t speed_mmps;

  speed_mmps = ClampI16FromFloat(speed_mps * SPEED_SCALE_MMPS_PER_MPS);
  PackI16LE(&txData[0], speed_mmps);

  txHeader.StdId = std_id;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 2U;
  txHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
  {
    can_tx_err_count++;
    return;
  }

  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK)
  {
    can_tx_err_count++;
    return;
  }

  can_tx_req_count++;
}

static float ComputeSpeedTargetFromAngle(float angle_deg)
{
  float delta;
  float normalized;
  float target;

  angle_deg = CLAMP(angle_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
  delta = fabsf(angle_deg - ANGLE_CENTER_DEG);
  normalized = CLAMP(delta / ANGLE_FULL_SCALE_DEG, 0.0f, 1.0f);

  target = VREF_MAX_MPS - (VREF_MAX_MPS - VREF_MIN_MPS) * normalized;

  return CLAMP(target, VREF_MIN_MPS, VREF_MAX_MPS);
}

static float ApplyRateLimiter(float current, float target, float up_rate, float down_rate, float Ts)
{
  float delta = target - current;
  float max_up = up_rate * Ts;
  float max_down = down_rate * Ts;

  if (delta > max_up)
  {
    delta = max_up;
  }
  else if (delta < -max_down)
  {
    delta = -max_down;
  }

  return current + delta;
}

static void SetEqualWheelRefs(float base_ref_mps, volatile float *left_ref_mps, volatile float *right_ref_mps)
{
  if (left_ref_mps != NULL)
  {
    *left_ref_mps = base_ref_mps;
  }

  if (right_ref_mps != NULL)
  {
    *right_ref_mps = base_ref_mps;
  }
}

static void ApplyElectronicDifferential(float angle_deg, float base_ref_mps, volatile float *left_ref_mps, volatile float *right_ref_mps)
{
  float turn_delta;
  float turn_mag;
  float speed_delta;

  if ((left_ref_mps == NULL) || (right_ref_mps == NULL))
  {
    return;
  }

  turn_delta = CLAMP(angle_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG) - ANGLE_CENTER_DEG;

  if (fabsf(turn_delta) <= ANGLE_CENTER_DEADBAND_DEG)
  {
    SetEqualWheelRefs(base_ref_mps, left_ref_mps, right_ref_mps);
    return;
  }

  turn_mag = CLAMP(fabsf(turn_delta) / ANGLE_FULL_SCALE_DEG, 0.0f, 1.0f);
  speed_delta = base_ref_mps * DIFF_MAX_RATIO * turn_mag;

  if (turn_delta < 0.0f)
  {
    *left_ref_mps  = CLAMP(base_ref_mps - speed_delta, 0.0f, VREF_MAX_MPS);
    *right_ref_mps = CLAMP(base_ref_mps + speed_delta, 0.0f, VREF_MAX_MPS);
  }
  else
  {
    *left_ref_mps  = CLAMP(base_ref_mps + speed_delta, 0.0f, VREF_MAX_MPS);
    *right_ref_mps = CLAMP(base_ref_mps - speed_delta, 0.0f, VREF_MAX_MPS);
  }
}

static void EncoderSpeed_Update(void)
{
  int16_t cnt_left;
  int16_t cnt_right;
  int16_t delta_left;
  int16_t delta_right;
  float raw_left;
  float raw_right;

  cnt_left  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  cnt_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

  delta_left  = (int16_t)(cnt_left  - prev_cnt_left);
  delta_right = (int16_t)(cnt_right - prev_cnt_right);

  prev_cnt_left  = cnt_left;
  prev_cnt_right = cnt_right;

  raw_left  = LEFT_SIGN  * ((float)delta_left)  * ENC_TO_MPS;
  raw_right = RIGHT_SIGN * ((float)delta_right) * ENC_TO_MPS;

  v_left_mps  = SPEED_FILTER_ALPHA * raw_left  + SPEED_FILTER_BETA * v_left_mps;
  v_right_mps = SPEED_FILTER_ALPHA * raw_right + SPEED_FILTER_BETA * v_right_mps;
  v_avg_mps   = 0.5f * (v_left_mps + v_right_mps);
}

static void PID_Reset(PID_t *pid)
{
  pid->integral = 0.0f;
  pid->prev_meas = 0.0f;
  pid->output = 0.0f;
}

static void PID_ResetAll(void)
{
  PID_Reset(&pid_left);
  PID_Reset(&pid_right);
}

static float PID_Update(PID_t *pid, float ref, float meas, float Ts)
{
  float error = ref - meas;
  float derivative = -(meas - pid->prev_meas) / Ts;
  float integral_candidate;
  float output_unsat;

  integral_candidate = pid->integral + error * Ts;
  integral_candidate = CLAMP(integral_candidate, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

  output_unsat = (pid->Kp * error)
               + (pid->Ki * integral_candidate)
               + (pid->Kd * derivative);

  /* Anti-windup: integrate only when not saturated, or when error helps desaturate. */
  if ((output_unsat >= -OUTPUT_CLAMP_PCT && output_unsat <= OUTPUT_CLAMP_PCT) ||
      ((output_unsat >  OUTPUT_CLAMP_PCT) && (error < 0.0f)) ||
      ((output_unsat < -OUTPUT_CLAMP_PCT) && (error > 0.0f)))
  {
    pid->integral = integral_candidate;
  }
  else
  {
    output_unsat = (pid->Kp * error)
                 + (pid->Ki * pid->integral)
                 + (pid->Kd * derivative);
  }

  pid->output    = CLAMP(output_unsat, -OUTPUT_CLAMP_PCT, OUTPUT_CLAMP_PCT);
  pid->prev_meas = meas;

  return pid->output;
}

static void Motor_SetCommand(
    TIM_HandleTypeDef *htim,
    uint32_t ch_fwd,
    uint32_t ch_rev,
    float cmd_percent,
    float min_pwm_pct)
{
  float mag = fabsf(cmd_percent);
  uint16_t pwm;

  mag = CLAMP(mag, 0.0f, 100.0f);

  if ((mag > 0.0f) && (mag < min_pwm_pct))
  {
    mag = min_pwm_pct;
  }

  pwm = (uint16_t)(mag * INV_100 * PWM_MAX_COUNTS);

  __HAL_TIM_SET_COMPARE(htim, ch_fwd, 0U);
  __HAL_TIM_SET_COMPARE(htim, ch_rev, 0U);

  if (cmd_percent > 0.0f)
  {
    __HAL_TIM_SET_COMPARE(htim, ch_fwd, pwm);
  }
  else if (cmd_percent < 0.0f)
  {
    __HAL_TIM_SET_COMPARE(htim, ch_rev, pwm);
  }
}
static void Motor_All_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0U);
}

static void CanRxTask(void *argument)
{
  CanRxEntry_t rx_entry;

  (void)argument;

  for (;;)
  {
    if (osMessageQueueGet(can_rx_queue_id, &rx_entry, NULL, osWaitForever) != osOK)
    {
      continue;
    }

    if (rx_entry.header.IDE != CAN_ID_STD)
    {
      can_rx_invalid_count++;
      continue;
    }

    if (rx_entry.header.StdId != CAN_ID_STEERING_CMD_FROM_VCU)
    {
      continue;
    }

    if (rx_entry.header.RTR != CAN_RTR_DATA)
    {
      can_rx_invalid_count++;
      continue;
    }

    if (rx_entry.header.DLC < 2U)
    {
      can_rx_dlc_error_count++;
      continue;
    }

    {
      int16_t parsed_angle_cdeg = UnpackI16LE(&rx_entry.data[0]);
      float parsed_angle_deg = ((float)parsed_angle_cdeg) / ANGLE_SCALE_CDEG_PER_DEG;

      if ((parsed_angle_deg < ANGLE_MIN_DEG) || (parsed_angle_deg > ANGLE_MAX_DEG))
      {
        can_rx_invalid_count++;
        continue;
      }

      if (fabsf(parsed_angle_deg - ANGLE_CENTER_DEG) <= ANGLE_CENTER_DEADBAND_DEG)
      {
        parsed_angle_deg = ANGLE_CENTER_DEG;
      }

      AngleCmd_Set(parsed_angle_deg, HAL_GetTick());
      can_rx_ok_count++;
    }
  }
}

static void MotorControlTask(void *argument)
{
  TickType_t last_wake_tick;
  CanRxEntry_t dropped_entry;

  (void)argument;

  last_wake_tick = xTaskGetTickCount();

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();
    float local_angle = ANGLE_CENTER_DEG;
    uint8_t angle_fresh;
    uint8_t busoff_fault_pending_local = 0U;

    taskENTER_CRITICAL();
    if (can_busoff_fault_pending != 0U)
    {
      busoff_fault_pending_local = 1U;
      can_busoff_fault_pending = 0U;
      g_angle_cmd.valid = 0U;
      g_angle_cmd.tick_ms = 0U;
      g_angle_cmd_reacquired = 0U;
    }
    taskEXIT_CRITICAL();

    while ((busoff_fault_pending_local != 0U) &&
           (can_rx_queue_id != NULL) &&
           (osMessageQueueGet(can_rx_queue_id, &dropped_entry, NULL, 0U) == osOK))
    {
    }

    if (AngleCmd_TakeReacquiredFlag() != 0U)
    {
      SpeedControl_Reset();
    }

    EncoderSpeed_Update();

    angle_fresh = IsAngleCmdFresh(now, &local_angle);

    if (angle_fresh == 0U)
    {
      v_ref_target_mps = 0.0f;
      v_ref_mps = ApplyRateLimiter(
          v_ref_mps,
          0.0f,
          VREF_RAMP_DOWN_MPS_PER_S,
          VREF_RAMP_DOWN_MPS_PER_S,
          TS_SEC);

      if (v_ref_mps <= VREF_STOP_THRESHOLD)
      {
        SpeedControl_Reset();
        SetEqualWheelRefs(0.0f, &v_ref_left_mps, &v_ref_right_mps);
        Motor_All_Stop();
      }
      else
      {
        SetEqualWheelRefs(v_ref_mps, &v_ref_left_mps, &v_ref_right_mps);

        PID_Update(&pid_left,  v_ref_left_mps,  v_left_mps,  TS_SEC);
        PID_Update(&pid_right, v_ref_right_mps, v_right_mps, TS_SEC);

        Motor_SetCommand(&htim4, TIM_CHANNEL_2, TIM_CHANNEL_1,
                         pid_left.output, MIN_EFFECTIVE_PWM_LEFT_PCT);

        Motor_SetCommand(&htim4, TIM_CHANNEL_4, TIM_CHANNEL_3,
                         pid_right.output, MIN_EFFECTIVE_PWM_RIGHT_PCT);
      }
    }
    else
    {
      v_ref_target_mps = ComputeSpeedTargetFromAngle(local_angle);
      v_ref_mps = ApplyRateLimiter(
          v_ref_mps,
          v_ref_target_mps,
          VREF_RAMP_UP_MPS_PER_S,
          VREF_RAMP_DOWN_MPS_PER_S,
          TS_SEC);

      ApplyElectronicDifferential(local_angle, v_ref_mps, &v_ref_left_mps, &v_ref_right_mps);

      PID_Update(&pid_left,  v_ref_left_mps,  v_left_mps,  TS_SEC);
      PID_Update(&pid_right, v_ref_right_mps, v_right_mps, TS_SEC);

      Motor_SetCommand(&htim4, TIM_CHANNEL_2, TIM_CHANNEL_1,
                       pid_left.output, MIN_EFFECTIVE_PWM_LEFT_PCT);

      Motor_SetCommand(&htim4, TIM_CHANNEL_4, TIM_CHANNEL_3,
                       pid_right.output, MIN_EFFECTIVE_PWM_RIGHT_PCT);
    }

    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(SAMPLE_TIME_MS));
  }
}

static void CanTxTask(void *argument)
{
  TickType_t last_wake_tick;

  (void)argument;

  last_wake_tick = xTaskGetTickCount();

  for (;;)
  {
    float speed_snapshot;

    taskENTER_CRITICAL();
    speed_snapshot = v_avg_mps;
    taskEXIT_CRITICAL();

    CAN_SendSpeedMmps(CAN_ID_VAVG_FEEDBACK, speed_snapshot);

    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(CAN_SEND_TIME_MS));
  }
}

static void DebugTask(void *argument)
{
  TickType_t last_wake_tick;

  (void)argument;

  last_wake_tick = xTaskGetTickCount();

  for (;;)
  {
    RearDebugSnapshot_t snapshot = DebugSnapshot_Get();
    UART2_PrintStatus(&snapshot);

    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(DEBUG_TIME_MS));
  }
}

static void RearEcu_RtosInit(void)
{
  static const osMessageQueueAttr_t can_rx_queue_attr =
  {
    .name = "canRxQueue",
    .cb_mem = &can_rx_queue_cb,
    .cb_size = sizeof(can_rx_queue_cb),
    .mq_mem = can_rx_queue_storage,
    .mq_size = sizeof(can_rx_queue_storage)
  };

  static const osThreadAttr_t can_rx_task_attr =
  {
    .name = "CanRxTask",
    .priority = osPriorityHigh,
    .stack_mem = can_rx_task_stack,
    .stack_size = sizeof(can_rx_task_stack),
    .cb_mem = &can_rx_task_cb,
    .cb_size = sizeof(can_rx_task_cb)
  };

  static const osThreadAttr_t motor_task_attr =
  {
    .name = "MotorCtrlTask",
    .priority = osPriorityRealtime,
    .stack_mem = motor_task_stack,
    .stack_size = sizeof(motor_task_stack),
    .cb_mem = &motor_task_cb,
    .cb_size = sizeof(motor_task_cb)
  };

  static const osThreadAttr_t can_tx_task_attr =
  {
    .name = "CanTxTask",
    .priority = osPriorityNormal,
    .stack_mem = can_tx_task_stack,
    .stack_size = sizeof(can_tx_task_stack),
    .cb_mem = &can_tx_task_cb,
    .cb_size = sizeof(can_tx_task_cb)
  };

  static const osThreadAttr_t debug_task_attr =
  {
    .name = "DebugTask",
    .priority = osPriorityLow,
    .stack_mem = debug_task_stack,
    .stack_size = sizeof(debug_task_stack),
    .cb_mem = &debug_task_cb,
    .cb_size = sizeof(debug_task_cb)
  };

  can_rx_queue_id = osMessageQueueNew(CAN_RX_QUEUE_SIZE, sizeof(CanRxEntry_t), &can_rx_queue_attr);
  if (can_rx_queue_id == NULL)
  {
    Error_Handler();
  }

  can_rx_task_id = osThreadNew(CanRxTask, NULL, &can_rx_task_attr);
  motor_task_id = osThreadNew(MotorControlTask, NULL, &motor_task_attr);
  can_tx_task_id = osThreadNew(CanTxTask, NULL, &can_tx_task_attr);
  debug_task_id = osThreadNew(DebugTask, NULL, &debug_task_attr);

  if ((can_rx_task_id == NULL) ||
      (motor_task_id == NULL) ||
      (can_tx_task_id == NULL) ||
      (debug_task_id == NULL))
  {
    Error_Handler();
  }
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  Motor_All_Stop();
  PID_ResetAll();
  CAN_Filter_Config();

  prev_cnt_left  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  prev_cnt_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

  __disable_irq();
  g_angle_cmd.angle_deg = ANGLE_CENTER_DEG;
  g_angle_cmd.valid = 0U;
  g_angle_cmd.tick_ms = 0U;
  g_angle_cmd_reacquired = 0U;
  __enable_irq();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  RearEcu_RtosInit();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  if (HAL_CAN_Start(&hcan) != HAL_OK) Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan,
      CAN_IT_RX_FIFO0_MSG_PENDING |
      CAN_IT_TX_MAILBOX_EMPTY     |
      CAN_IT_BUSOFF               |
      CAN_IT_ERROR) != HAL_OK) Error_Handler();

  UART2_Print("\r\n=== Rear STM32 ECU READY (FreeRTOS) ===\r\n");
  UART2_Print("RX 0x040: target steering angle, int16 centi-degree\r\n");
  UART2_Print("TX 0x020: rear average speed, int16 mm/s\r\n");
  UART2_Print("Tasks: RX queue / 10 ms control / 50 ms CAN TX / 500 ms debug\r\n");
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
  CanRxEntry_t rx_entry;
  CanRxEntry_t dropped_entry;

  if (hcan_ptr->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx_entry.header, rx_entry.data) != HAL_OK)
  {
    can_rx_invalid_count++;
    return;
  }

  if ((can_rx_queue_id == NULL) || (osKernelGetState() != osKernelRunning))
  {
    can_rx_queue_overflow_count++;
    return;
  }

  if (osMessageQueuePut(can_rx_queue_id, &rx_entry, 0U, 0U) != osOK)
  {
    can_rx_queue_overflow_count++;

    /* Keep the newest steering command and discard the oldest queued frame. */
    if (osMessageQueueGet(can_rx_queue_id, &dropped_entry, NULL, 0U) == osOK)
    {
      (void)osMessageQueuePut(can_rx_queue_id, &rx_entry, 0U, 0U);
    }
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan_ptr)
{
  if (hcan_ptr->Instance == CAN1)
  {
    can_tx_ok_count++;
  }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan_ptr)
{
  if (hcan_ptr->Instance == CAN1)
  {
    can_tx_ok_count++;
  }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan_ptr)
{
  if (hcan_ptr->Instance == CAN1)
  {
    can_tx_ok_count++;
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan_ptr)
{
  if (hcan_ptr->Instance == CAN1)
  {
    uint32_t err = HAL_CAN_GetError(hcan_ptr);

    can_last_error_code = err;
    can_tx_err_count++;

    if ((err & HAL_CAN_ERROR_BOF) != 0U)
    {
      can_busoff_count++;
      can_severe_error_count++;
      can_busoff_fault_pending = 1U;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  Motor_All_Stop();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
