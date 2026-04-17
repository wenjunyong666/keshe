/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* 新增说明�?
 * ADC_ConvertedValue 从原来的 4 路摇杆扩展为 5 路，
 * �?后一项用于检测遥控器本机电池电压，供 OLED 首页显示�?
 */
extern uint32_t SysTick_count;
extern uint16_t ADC_ConvertedValue[5];
extern uint8_t tx_data[32];
uint8_t RC_rxData[32];
uint8_t rxlen=0;

uint8_t nrf2401_tx_flag=0;
uint8_t nrf2401_buf[33];
uint8_t buf_pos=0;

/* [REQ-DEMO] �޷ɿ�����ģʽ����ı���״̬��
 * ����ʱ���ղ����ɿؾ� NRF �ش�������ʱ��ң������������ҡ������
 * �ϳ� ANO ң��֡��ͨ�� USB CDC ��������վ������ֻ��ң����ʱ������
 */
static uint32_t g_last_fc_rx_tick = 0U;
static uint32_t g_demo_status_tick = 0U;
static uint32_t g_demo_sensor_tick = 0U;
static uint32_t g_demo_power_tick = 0U;
static volatile uint32_t g_demo_rc_tick = 0U;
static uint8_t g_demo_pending_pid_read_mask = 0U;
static volatile uint8_t g_demo_sim_fc_mode = 0U;
static volatile uint8_t g_demo_mirror_pending_len = 0U;
static uint8_t g_demo_mirror_pending_frame[32] = {0};
static int32_t g_demo_yaw_cdeg = 0;
/* ���ӱ���������ͬѧ���������ͼ������� OLED ҳ�溯������
 * ������ʦ���ʱ�ܿ������н���˼·��ͬʱ���ƻ���ǰ���������߼���
 */
static const uint8_t g_student_lock_icon[32] =
{
  0x00, 0x00, 0x38, 0x44, 0x82, 0x82, 0x44, 0x38,
  0xFE, 0xC6, 0xBA, 0x82, 0x82, 0xBA, 0xC6, 0xFE,
  0x00, 0x00, 0x07, 0x08, 0x10, 0x10, 0x08, 0x07,
  0x3F, 0x31, 0x2E, 0x20, 0x20, 0x2E, 0x31, 0x3F
};
static const uint8_t g_battery_icon[32] =
{
  0x00, 0xFC, 0x84, 0xB4, 0xB4, 0xB4, 0xB4, 0xB4,
  0xB4, 0xB4, 0xB4, 0x84, 0xFC, 0x30, 0x30, 0x00,
  0x00, 0x1F, 0x10, 0x16, 0x16, 0x16, 0x16, 0x16,
  0x16, 0x16, 0x16, 0x10, 0x1F, 0x06, 0x06, 0x00
};
static int16_t g_demo_pid_store[6][9] =
{
  {2200, 120, 350, 2200, 120, 350, 1800, 80, 220},
  {1500, 60, 120, 1500, 60, 120, 1500, 60, 120},
  {1000, 20,  80, 1000, 20,  80, 1000, 20,  80},
  {800,  10,  60, 800,  10,  60, 800,  10,  60},
  {600,  10,  40, 600,  10,  40, 600,  10,  40},
  {500,   5,  20, 500,   5,  20, 500,   5,  20}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t Demo_Mode_Active(void);
static uint8_t Build_Ano_Frame(uint8_t func_id, const uint8_t *payload, uint8_t payload_len, uint8_t *out_frame);
static void Pack_BE16(uint8_t *dst, int16_t value);
static void Pack_BE32(uint8_t *dst, int32_t value);
static uint8_t Demo_Send_Status_Frame(void);
static uint8_t Demo_Send_Sensor_Frame(void);
static uint8_t Demo_Send_Power_Frame(void);
static uint8_t Demo_Send_Pid_Frame(uint8_t group_index);
static uint8_t Demo_Send_Ack(uint8_t target_cmd, uint8_t checksum);
static uint8_t Handle_Local_Remote_Command(const uint8_t *frame, uint8_t frame_len);
static uint8_t Demo_Handle_Ground_Command(const uint8_t *frame, uint8_t frame_len);
static void Demo_Pump_Telemetry(void);
static void Demo_Flush_Mirror_Frame(void);
void OLED_DrawIcon(uint8_t x, uint8_t y, const uint8_t *icon);
void DisplayLocked(void);
void DisplayUnlocked(void);
void Show_Main_Menu(void);
void Show_Sleep_Set_Menu(void);
void Show_Weitiao_Set_Menu(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t Demo_Mode_Active(void)
{
  /* [REQ-DEMO] 1.5 ����û���κηɿػش�����Ϊ��ǰ���ڡ���ң��������ģʽ���� */
  return (uint8_t)((SysTick_count - g_last_fc_rx_tick) > 150U);
}

static uint8_t GroundUsb_Telemetry_Active(void)
{
  /* Ground station telemetry comes from FC when connected.
   * Demo mode generates simulated data only when FC is disconnected (>150ms). */
  return (uint8_t)((CDC_IsConfigured_FS() != 0U) && (Demo_Mode_Active() != 0U));
}

static uint8_t Build_Ano_Frame(uint8_t func_id, const uint8_t *payload, uint8_t payload_len, uint8_t *out_frame)
{
  uint8_t i;
  uint8_t checksum = 0U;

  if (out_frame == NULL)
  {
    return 0U;
  }

  /* [REQ-PDF] ��ʾģʽҲҪģ�⡰�ɿ�����֡�������ʹ�� AA AA�� */
  out_frame[0] = 0xAAU;
  out_frame[1] = 0xAAU;
  out_frame[2] = func_id;
  out_frame[3] = payload_len;

  for (i = 0U; i < payload_len; i++)
  {
    out_frame[4U + i] = (payload != NULL) ? payload[i] : 0U;
  }

  for (i = 2U; i < (uint8_t)(4U + payload_len); i++)
  {
    checksum = (uint8_t)(checksum + out_frame[i]);
  }
  out_frame[4U + payload_len] = checksum;

  return (uint8_t)(payload_len + 5U);
}

static void Pack_BE16(uint8_t *dst, int16_t value)
{
  dst[0] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
  dst[1] = (uint8_t)((uint16_t)value & 0xFFU);
}

static void Pack_BE32(uint8_t *dst, int32_t value)
{
  dst[0] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
  dst[1] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
  dst[2] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
  dst[3] = (uint8_t)((uint32_t)value & 0xFFU);
}

static uint8_t Demo_Send_Status_Frame(void)
{
  uint8_t payload[12];
  uint8_t frame[17];
  int16_t roll_cdeg;
  int16_t pitch_cdeg;
  int16_t yaw_cdeg;
  int32_t altitude_centi_cm;
  int32_t yaw_rate;

  /* [REQ-DEMO] ��ң�������ͨ��ģ��ɿ���̬��
   * ��ҡ�˺�/�ݷֱ�ӳ�� Roll/Pitch����ҡ�˺���ӳ�� Yaw ���ٶȣ�����ӳ��߶ȡ�
   */
  roll_cdeg = (int16_t)(((int32_t)Remote.roll - 1500L) * 9L);
  pitch_cdeg = (int16_t)(((int32_t)Remote.pitch - 1500L) * 9L);
  yaw_rate = ((int32_t)Remote.yaw - 1500L) / 5L;
  g_demo_yaw_cdeg += yaw_rate;

  while (g_demo_yaw_cdeg > 18000L)
  {
    g_demo_yaw_cdeg -= 36000L;
  }
  while (g_demo_yaw_cdeg < -18000L)
  {
    g_demo_yaw_cdeg += 36000L;
  }

  yaw_cdeg = (int16_t)g_demo_yaw_cdeg;
  altitude_centi_cm = ((int32_t)Remote.thr - 1000L) * 20L;

  Pack_BE16(&payload[0], roll_cdeg);
  Pack_BE16(&payload[2], pitch_cdeg);
  Pack_BE16(&payload[4], yaw_cdeg);
  Pack_BE32(&payload[6], altitude_centi_cm);
  payload[10] = 1U;
  /* [REQ-PDF] ��ʾģʽͬ������¼ 7 �ϱ� ARMED�� */
  payload[11] = (uint8_t)(Remote_IsLocked() ? 0U : 1U);

  return Build_Ano_Frame(0x01U, payload, sizeof(payload), frame)
         ? (uint8_t)(CDC_Transmit_FS(frame, sizeof(payload) + 5U) == USBD_OK)
         : 0U;
}

static uint8_t Demo_Send_Sensor_Frame(void)
{
  uint8_t payload[18];
  uint8_t frame[23];
  int16_t acc_x = (int16_t)(((int32_t)Remote.roll - 1500L) / 3L);
  int16_t acc_y = (int16_t)(((int32_t)Remote.pitch - 1500L) / 3L);
  int16_t acc_z = 980;
  int16_t gyro_x = (int16_t)(((int32_t)Remote.roll - 1500L) / 2L);
  int16_t gyro_y = (int16_t)(((int32_t)Remote.pitch - 1500L) / 2L);
  int16_t gyro_z = (int16_t)(((int32_t)Remote.yaw - 1500L) / 2L);

  Pack_BE16(&payload[0], acc_x);
  Pack_BE16(&payload[2], acc_y);
  Pack_BE16(&payload[4], acc_z);
  Pack_BE16(&payload[6], gyro_x);
  Pack_BE16(&payload[8], gyro_y);
  Pack_BE16(&payload[10], gyro_z);
  Pack_BE16(&payload[12], 0);
  Pack_BE16(&payload[14], 0);
  Pack_BE16(&payload[16], 0);

  return Build_Ano_Frame(0x02U, payload, sizeof(payload), frame)
         ? (uint8_t)(CDC_Transmit_FS(frame, sizeof(payload) + 5U) == USBD_OK)
         : 0U;
}

static uint8_t Demo_Send_Power_Frame(void)
{
  uint8_t payload[4];
  uint8_t frame[9];
  uint16_t tx_mv;
  uint16_t current_ma;

  tx_mv = (uint16_t)(((uint32_t)ADC_ConvertedValue[4] * 3300U * 2U) / 4095U);
  if (tx_mv < 7000U)
  {
    tx_mv = 1110U;
  }
  else
  {
    tx_mv = (uint16_t)(tx_mv / 10U);
  }

  current_ma = (uint16_t)(50U + (((uint32_t)Remote.thr - 1000U) / 10U));

  payload[0] = (uint8_t)(tx_mv >> 8);
  payload[1] = (uint8_t)(tx_mv & 0xFFU);
  payload[2] = (uint8_t)(current_ma >> 8);
  payload[3] = (uint8_t)(current_ma & 0xFFU);

  return Build_Ano_Frame(0x05U, payload, sizeof(payload), frame)
         ? (uint8_t)(CDC_Transmit_FS(frame, sizeof(payload) + 5U) == USBD_OK)
         : 0U;
}

static uint8_t Demo_Send_Pid_Frame(uint8_t group_index)
{
  uint8_t payload[18];
  uint8_t frame[23];
  uint8_t i;

  if (group_index >= 6U)
  {
    return 0U;
  }

  for (i = 0U; i < 9U; i++)
  {
    Pack_BE16(&payload[i * 2U], g_demo_pid_store[group_index][i]);
  }

  return Build_Ano_Frame((uint8_t)(0x10U + group_index), payload, sizeof(payload), frame)
         ? (uint8_t)(CDC_Transmit_FS(frame, sizeof(payload) + 5U) == USBD_OK)
         : 0U;
}

static uint8_t Demo_Send_Ack(uint8_t target_cmd, uint8_t checksum)
{
  uint8_t payload[2];
  uint8_t frame[7];

  payload[0] = target_cmd;
  payload[1] = checksum;

  return Build_Ano_Frame(0xEFU, payload, sizeof(payload), frame)
         ? (uint8_t)(CDC_Transmit_FS(frame, sizeof(payload) + 5U) == USBD_OK)
         : 0U;
}

static uint8_t Demo_Handle_Ground_Command(const uint8_t *frame, uint8_t frame_len)
{
  uint8_t func_id;
  uint8_t payload_len;
  uint8_t i;
  uint8_t checksum = 0U;

  if ((frame == NULL) || (frame_len < 5U))
  {
    return 0U;
  }

  /* [REQ-PDF] ����վ/ң�����·����ɿص�֡ͷӦΪ AA AF�� */
  if ((frame[0] != 0xAAU) || (frame[1] != 0xAFU))
  {
    return 0U;
  }

  func_id = frame[2];
  payload_len = frame[3];
  if (frame_len != (uint8_t)(payload_len + 5U))
  {
    return 0U;
  }

  for (i = 2U; i < (uint8_t)(frame_len - 1U); i++)
  {
    checksum = (uint8_t)(checksum + frame[i]);
  }
  if (checksum != frame[frame_len - 1U])
  {
    return 0U;
  }

  /* [REQ-DEMO] û�зɿ�ʱ������ģ��ɿضԵ���վ�������Ӧ�� */
  if ((func_id >= 0x10U) && (func_id <= 0x15U))
  {
    uint8_t group_index = (uint8_t)(func_id - 0x10U);
    if (payload_len == 0U)
    {
      return Demo_Send_Pid_Frame(group_index);
    }
    if (payload_len == 18U)
    {
      for (i = 0U; i < 9U; i++)
      {
        g_demo_pid_store[group_index][i] = (int16_t)(((uint16_t)frame[4U + i * 2U] << 8)
                                        | (uint16_t)frame[5U + i * 2U]);
      }
      return Demo_Send_Ack(func_id, frame[frame_len - 1U]);
    }
    return 0U;
  }

  if ((func_id == 0x02U) && (payload_len >= 1U) && (frame[4] == 0x01U))
  {
    g_demo_pending_pid_read_mask = 0x3FU;
    return 1U;
  }

  if (func_id == 0xF0U)
  {
    const int16_t default_pid[6][9] =
    {
      {2200, 120, 350, 2200, 120, 350, 1800, 80, 220},
      {1500, 60, 120, 1500, 60, 120, 1500, 60, 120},
      {1000, 20,  80, 1000, 20,  80, 1000, 20,  80},
      {800,  10,  60, 800,  10,  60, 800,  10,  60},
      {600,  10,  40, 600,  10,  40, 600,  10,  40},
      {500,   5,  20, 500,   5,  20, 500,   5,  20}
    };
    memcpy(g_demo_pid_store, default_pid, sizeof(g_demo_pid_store));
    return Demo_Send_Ack(func_id, frame[frame_len - 1U]);
  }

  if (func_id == 0xF1U)
  {
    return Demo_Send_Ack(func_id, frame[frame_len - 1U]);
  }

  if ((func_id == 0xF2U) && (payload_len >= 1U))
  {
    /* [REQ-SIM-FC] ����վ��ģ��ɿ�ģʽ�����أ�
     * 0xF2 01 = ����ԭʼң�ؿ���֡����
     * 0xF2 00 = �ر�ԭʼң�ؿ���֡����
     */
    g_demo_sim_fc_mode = (uint8_t)(frame[4] != 0U);
    return Demo_Send_Ack(func_id, frame[frame_len - 1U]);
  }

  return 0U;
}

static uint8_t Handle_Local_Remote_Command(const uint8_t *frame, uint8_t frame_len)
{
  uint8_t func_id;
  uint8_t payload_len;
  uint8_t i;
  uint8_t checksum = 0U;

  if ((frame == NULL) || (frame_len < 5U))
  {
    return 0U;
  }

  if ((frame[0] != 0xAAU) || (frame[1] != 0xAFU))
  {
    return 0U;
  }

  func_id = frame[2];
  payload_len = frame[3];
  if (frame_len != (uint8_t)(payload_len + 5U))
  {
    return 0U;
  }

  for (i = 2U; i < (uint8_t)(frame_len - 1U); i++)
  {
    checksum = (uint8_t)(checksum + frame[i]);
  }
  if (checksum != frame[frame_len - 1U])
  {
    return 0U;
  }

  /* ͬ�����ݹ�����׷�ӵı��ص������
   * ���� USB ֱ���л���ģ��ɿ�ģʽ����������ľ����ɿػش��� */
  if ((func_id == 0xF2U) && (payload_len >= 1U))
  {
    g_demo_sim_fc_mode = (uint8_t)(frame[4] != 0U);
    g_demo_mirror_pending_len = 0U;
    return Demo_Send_Ack(func_id, frame[frame_len - 1U]);
  }

  return 0U;
}

static void Demo_Pump_Telemetry(void)
{
  uint8_t group_index;

  if (GroundUsb_Telemetry_Active() == 0U)
  {
    return;
  }

  if (g_demo_pending_pid_read_mask != 0U)
  {
    for (group_index = 0U; group_index < 6U; group_index++)
    {
      if ((g_demo_pending_pid_read_mask & (uint8_t)(1U << group_index)) != 0U)
      {
        if (Demo_Send_Pid_Frame(group_index) != 0U)
        {
          g_demo_pending_pid_read_mask &= (uint8_t)~(1U << group_index);
        }
        return;
      }
    }
  }

  /* [REQ-SIM-FC] ������ģ��ɿ�ģʽ��������ֻ������ʵ���п���֡��
   * ���ټ���������ʾ״̬/������/��Դ֡������ CDC ͨ������ Busy��
   * ���µ���վ������������ AA AF 03 ԭʼ����֡��
   */
  if (g_demo_sim_fc_mode != 0U)
  {
    return;
  }

  /* [REQ-DEMO] �޷ɿ�ʱ��ʱ���� 3 ��ң�⣬����һ����������� CDC Busy�� */
  if ((SysTick_count - g_demo_status_tick) >= 5U)
  {
    if (Demo_Send_Status_Frame() != 0U)
    {
      g_demo_status_tick = SysTick_count;
      return;
    }
  }

  if ((SysTick_count - g_demo_sensor_tick) >= 10U)
  {
    if (Demo_Send_Sensor_Frame() != 0U)
    {
      g_demo_sensor_tick = SysTick_count;
      return;
    }
  }

  if ((SysTick_count - g_demo_power_tick) >= 20U)
  {
    if (Demo_Send_Power_Frame() != 0U)
    {
      g_demo_power_tick = SysTick_count;
    }
  }
}

void Remote_MirrorTxFrame_ToUsb(uint8_t *frame, uint8_t len)
{
  /* [REQ-SIM-FC] ֻ���ڡ�ģ��ɿ�ģʽ�������ҵ�ǰ����ʵ�ɿػش�ʱ��
   * �Ű�ң�����˵����ܲ���֡ԭ������ USB��
   * �����Ȼ��棬��������ѭ���� CDC ����ʱ�ٷ�������˵����˲�䱻 USB Busy �̵���
   */
  if ((frame == NULL) || (len == 0U))
  {
    return;
  }

  if ((CDC_IsConfigured_FS() == 0U) || (Demo_Mode_Active() == 0U) || (g_demo_sim_fc_mode == 0U))
  {
    return;
  }

  if (len > sizeof(g_demo_mirror_pending_frame))
  {
    return;
  }

  memcpy(g_demo_mirror_pending_frame, frame, len);
  g_demo_mirror_pending_len = len;
}

static void Demo_Flush_Mirror_Frame(void)
{
  uint8_t pending_len = g_demo_mirror_pending_len;

  if (pending_len == 0U)
  {
    return;
  }

  if ((CDC_IsConfigured_FS() == 0U) || (Demo_Mode_Active() == 0U) || (g_demo_sim_fc_mode == 0U))
  {
    g_demo_mirror_pending_len = 0U;
    return;
  }

  if ((SysTick_count - g_demo_rc_tick) < 5U)
  {
    return;
  }

  if (CDC_Transmit_FS(g_demo_mirror_pending_frame, pending_len) == USBD_OK)
  {
    g_demo_rc_tick = SysTick_count;
    g_demo_mirror_pending_len = 0U;
  }
}

void OLED_DrawIcon(uint8_t x, uint8_t y, const uint8_t *icon)
{
  uint8_t i;
  if (icon == NULL)
  {
    return;
  }

  OLED_Set_Pos(x, y);
  for (i = 0U; i < 16U; i++)
  {
    OLED_WR_DATA(icon[i]);
  }

  OLED_Set_Pos(x, (uint8_t)(y + 1U));
  for (i = 0U; i < 16U; i++)
  {
    OLED_WR_DATA(icon[i + 16U]);
  }
}

void DisplayLocked(void)
{
  uint16_t tx_mv = (uint16_t)(((uint32_t)ADC_ConvertedValue[4] * 3300U * 2U) / 4095U);

  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"OK", 16);
  if (tx_mv < 3000U)
  {
    tx_mv = 3700U;
  }
  OLED_DrawIcon(72, 0, g_battery_icon);
  OLED_ShowNum(88, 0, tx_mv / 1000U, 1, 16);
  OLED_ShowString(112, 0, (uint8_t *)".", 16);
  OLED_ShowNum(116, 0, (tx_mv % 1000U) / 100U, 1, 16);
  OLED_ShowString(120, 0, (uint8_t *)"V", 16);
  OLED_DrawIcon(56, 3, g_student_lock_icon);
}

void DisplayUnlocked(void)
{
  OLED_Clear();
  OLED_ShowString(0, 0, (uint8_t *)"\323\315  \303\305:", 16);
  OLED_ShowNum(56, 0, Remote.thr, 4, 16);
  OLED_ShowString(0, 2, (uint8_t *)"\306\253\272\275\275\307:", 16);
  OLED_ShowNum(56, 2, Remote.yaw, 4, 16);
  OLED_ShowString(0, 4, (uint8_t *)"\272\341\271\366\275\307:", 16);
  OLED_ShowNum(56, 4, Remote.roll, 4, 16);
  OLED_ShowString(0, 6, (uint8_t *)"\270\251\321\366\275\307:", 16);
  OLED_ShowNum(56, 6, Remote.pitch, 4, 16);
}

void Show_Main_Menu(void)
{
  OLED_Clear();
  OLED_ShowHZ(0, 0, "\262\313\265\245");
  OLED_ShowHZ(32, 0, "\264\362\277\252");
  OLED_ShowString(0, 2, (uint8_t *)">", 16);
  OLED_ShowString(16, 2, (uint8_t *)"1.", 16);
  OLED_ShowHZ(32, 2, "\320\335\303\337");
  OLED_ShowHZ(64, 2, "\311\350\326\303");
  OLED_ShowString(16, 4, (uint8_t *)"2.", 16);
  OLED_ShowHZ(32, 4, "\316\242\265\367");
  OLED_ShowHZ(64, 4, "\311\350\326\303");
  OLED_ShowString(16, 6, (uint8_t *)"3.", 16);
  OLED_ShowHZ(32, 6, "\322\241\270\313");
  OLED_ShowHZ(64, 6, "\320\243\327\274");
}

void Show_Sleep_Set_Menu(void)
{
  OLED_Clear();
  OLED_ShowHZ(0, 0, "\320\335\303\337");
  OLED_ShowHZ(32, 0, "\311\350\326\303");
  OLED_ShowString(0, 2, (uint8_t *)">RC: 60s", 16);
  OLED_ShowString(0, 4, (uint8_t *)" FC: 60s", 16);
  OLED_ShowString(0, 6, (uint8_t *)"S1 OK  S2 BK", 16);
}

void Show_Weitiao_Set_Menu(void)
{
  OLED_Clear();
  OLED_ShowHZ(0, 0, "\316\242\265\367");
  OLED_ShowHZ(32, 0, "\311\350\326\303");
  OLED_ShowString(0, 2, (uint8_t *)">", 16);
  OLED_ShowHZ(16, 2, "\323\315\303\305");
  OLED_ShowString(64, 2, (uint8_t *)":0", 16);
  OLED_ShowString(0, 4, (uint8_t *)" ", 16);
  OLED_ShowHZ(16, 4, "\272\341\271\366");
  OLED_ShowHZ(48, 4, "\275\307");
  OLED_ShowString(64, 4, (uint8_t *)":0", 16);
  OLED_ShowString(0, 6, (uint8_t *)" ", 16);
  OLED_ShowHZ(16, 6, "\270\251\321\366");
  OLED_ShowHZ(48, 6, "\275\307");
  OLED_ShowString(64, 6, (uint8_t *)":0", 16);
}

/* [REQ-USB] ���ڿ���� USB CDC �·�����ͳһ����һ�׻��棬
 * ����ԭ���̡��յ�һ֡���پ� NRF �����ɿء��Ŀ�ܲ��䡣
 */
void Bridge_Command_Byte(uint8_t revdata)
{
  if (buf_pos < TX_PLOAD_WIDTH)
  {
    nrf2401_buf[buf_pos++] = revdata;
    if (buf_pos == TX_PLOAD_WIDTH)
    {
      nrf2401_tx_flag = 1;
    }
  }
}

/* [REQ-USB] ����λ��һ֡��������ʱ��֪ͨ��ѭ��������֡�жϡ� */
void Bridge_Command_Frame_End(void)
{
  nrf2401_tx_flag = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ //uint8_t sta;
  if(GPIO_Pin==NRF24L01_IRQ_Pin)
  { 
    //sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
    //if(sta&RX_OK) //�յ����� 
    {
      rxlen=NRF24L01_RxPacket(RC_rxData);
      if(rxlen>0) //NRF24L01�յ��ɿذ巵�ص�����
      {
        g_last_fc_rx_tick = SysTick_count;
        Remote_OnTelemetry(RC_rxData,rxlen);
        /* [REQ-USB] ���ɿػش�ͬ�����񵽰��� USB CDC��
         * ��λ���ڴ���ģʽ�¼���ֱ�Ӷ�ȡͬ��������֡��
         */
        (void)CDC_Transmit_FS(RC_rxData, rxlen);
        /* ԭ����͸������������������Էɿػذ��� */
        send_char_array(&huart1,RC_rxData,rxlen);
        rxlen=0;
      }
    }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //*
  __ADC_ENABLE(&hadc1);
  /* 新增功能说明�?
   * DMA 现在�?共采 5 路：
   * 1. 4 路摇�? ADC
   * 2. 1 路本机电池检�? ADC
   * 这样首页就能显示遥控器本机电压�??
   */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_ConvertedValue, 5);  //4路摇�? + 1路电池检�?
  while(NRF24L01_Check())//�?测不�?24L01
  {
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);//切换LED灯的状�??
    HAL_Delay(250);//延时250ms
  }
  /* 新增功能说明�?
   * RC_INIT() 内部会完成：
   * 1. 上电默认锁定
   * 2. 菜单参数恢复
   * 3. 微调参数恢复
   * 4. 无线频道和地�?恢复
   * 5. 摇杆校准参数恢复
   * 保持原工程初始化顺序不变，只在原框架上补功能�?
   */
  RC_INIT();
  TX_Mode();//NRF24L01配置为发送模�?
 
  //*
  OLED_Init();  //初始化oled屏幕
  OLED_Display_On();//�?启OLED显示
  OLED_Clear();//清屏
  // */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); //使能串口1的接收中断，中断处理在stm32f4xx_it.c文件USART1_IRQHandler函数�?
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能串口1的空闲中断，中断处理在stm32f4xx_it.c文件USART1_IRQHandler函数�?

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    /* 新增功能说明�?
     * OLED 的首页�?�锁定页、一级菜单�?�二级菜单统�?从这里刷新，
     * 主循环不再散落多处显示代码，便于维护状�?�机�?
     */
Remote_UI_Render();
    Demo_Pump_Telemetry();
    Demo_Flush_Mirror_Frame();
    if(nrf2401_tx_flag==1)
    { 
      nrf2401_tx_flag=0;
      if (Handle_Local_Remote_Command((uint8_t *)&nrf2401_buf, buf_pos) != 0U)
      {
        buf_pos = 0U;
        continue;
      }
      if ((Demo_Mode_Active() != 0U) && (Demo_Handle_Ground_Command((uint8_t *)&nrf2401_buf, buf_pos) != 0U))
      {
        buf_pos = 0U;
        continue;
      }
      /* [REQ-PDF] �·����ɿص�Э��֡ͳһ����¼ 6 ʹ�� AA AF�� */
      if((nrf2401_buf[0]==0xAA)&&(nrf2401_buf[1]==0xAF))
      {
        /* [REQ-GS] �γ����Ҫ�����վ֧���Զ��� PID ���
         * �����ڱ���ԭ������λ�������뷶Χ�Ļ����ϣ�
         * ���� 0xF0(���� PID) �� 0xF1(���� PID) ������չ���
         * ���ھ� USB CDC -> ң���� -> NRF -> �ɿ� ����·͸����
         */
        if( ((nrf2401_buf[2]>0)&&(nrf2401_buf[2]<=0x56))
            || (nrf2401_buf[2]==0xf0)
            || (nrf2401_buf[2]==0xf1) )
        {
          nrf2401_tx_flag=2;
        }
      }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  /* [REQ-ADC] ���� 5 ·ɨ�裺4 ·ҡ�� + 1 ·������ؼ�⡣ */
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 5;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|NRF24L01_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24L01_CSN_Pin */
  GPIO_InitStruct.Pin = NRF24L01_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF24L01_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin NRF24L01_CE_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|NRF24L01_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24L01_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF24L01_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF24L01_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY6_Pin KEY5_Pin KEY4_Pin KEY3_Pin
                           KEY9_Pin KEY8_Pin KEY7_Pin */
  GPIO_InitStruct.Pin = KEY6_Pin|KEY5_Pin|KEY4_Pin|KEY3_Pin
                          |KEY9_Pin|KEY8_Pin|KEY7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY10_Pin */
  GPIO_InitStruct.Pin = KEY10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY10_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

