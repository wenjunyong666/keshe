#include "remote.h"
#include "main.h"
#include "nrf24l01.h"
#include "uart.h"
#include "flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Added feature overview: keep the original HAL + SysTick + main-loop framework.
 * 1. Lock/unlock and auto-lock after idle.
 * 2. OLED home/menu UI and S1/S2 menu interaction.
 * 3. Soft sleep for the remote and flight controller while keeping NRF online.
 * 4. Trim, stick calibration, FC calibration trigger and RF reset/pair commands.
 */

#define REMOTE_CFG_MAGIC         ((uint16_t)0x5A67)
#define REMOTE_RF_BUILD_ID       ((uint16_t)((((uint16_t)__TIME__[0]) << 8) ^ (((uint16_t)__TIME__[1]) << 4) ^ (((uint16_t)__TIME__[3]) << 12) ^ (((uint16_t)__TIME__[4]) << 2) ^ ((uint16_t)__TIME__[6]) ^ (((uint16_t)__TIME__[7]) << 1)))
#define REMOTE_UNLOCK_TIMEOUT    ((uint32_t)1000)
#define REMOTE_LONG_PRESS_TICK   ((uint16_t)200)
#define REMOTE_LOCK_LOW_TH       ((uint16_t)1120)
#define REMOTE_LOCK_HIGH_TH      ((uint16_t)1880)
#define REMOTE_NAV_LOW_TH        ((uint16_t)1200)
#define REMOTE_NAV_HIGH_TH       ((uint16_t)1800)
#define REMOTE_NAV_NEUTRAL_LOW   ((uint16_t)1450)
#define REMOTE_NAV_NEUTRAL_HIGH  ((uint16_t)1550)
#define REMOTE_NAV_CONFIRM_TICK  ((uint8_t)3)
#define REMOTE_MENU_ADC_UD_INDEX ((uint8_t)3)
#define REMOTE_MENU_ADC_LR_INDEX ((uint8_t)2)
#define REMOTE_MENU_RAW_LOW_TH   ((uint16_t)1300)
#define REMOTE_MENU_RAW_HIGH_TH  ((uint16_t)2800)
#define REMOTE_MENU_RAW_NL_TH    ((uint16_t)1700)
#define REMOTE_MENU_RAW_NH_TH    ((uint16_t)2400)
#define REMOTE_LEFT_UD_INDEX     ((uint8_t)3)
#define REMOTE_LEFT_LR_INDEX     ((uint8_t)2)
#define REMOTE_RIGHT_UD_INDEX    ((uint8_t)1)
#define REMOTE_RIGHT_LR_INDEX    ((uint8_t)0)
#define REMOTE_MENU_STEP         1
#define REMOTE_SLEEP_STEP        10
#define REMOTE_SLEEP_MAX         600
#define REMOTE_LOCAL_STANDBY_SECONDS 60U
#define REMOTE_IDLE_ADC_DELTA    30U
#define REMOTE_RF_SWITCH_DELAY_TICKS 20U
#define REMOTE_STICK_AXIS_COUNT  4
#define REMOTE_ADC_YAW_INDEX     0
#define REMOTE_ADC_THR_INDEX     1
#define REMOTE_ADC_ROL_INDEX     2
#define REMOTE_ADC_PIT_INDEX     3
#define REMOTE_ADC_BAT_INDEX     4
#define REMOTE_BAT_REF_MV        3300U
#define REMOTE_BAT_DIV_NUM       2U
#define REMOTE_BAT_DIV_DEN       1U
/* Logical key remap used by the current board wiring.
 * S1/S2 are kept as logical names so the UI code does not depend on board silk-screen names.
 */
#define REMOTE_S1_PIN            KEY5_Pin
#define REMOTE_S2_PIN            KEY4_Pin

/* OLED Chinese labels use explicit GB2312 byte strings.
 * This avoids editor-encoding changes breaking the glyph lookup table.
 */
#define ZH_SIGNAL_OK      "\xD0\xC5\xBA\xC5\xBA\xC3"
#define ZH_SIGNAL_LOST    "\xD0\xC5\xBA\xC5\xB6\xCF"
#define ZH_HOLD_MENU      "\xB3\xA4\xB0\xB4S1\xB2\xCB\xB5\xA5"
#define ZH_REMOTE_READY   "\xD2\xA3\xBF\xD8\xBE\xCD\xD0\xF7"
#define ZH_FC_READY       "\xB7\xC9\xBF\xD8\xBE\xCD\xD0\xF7"
#define ZH_HINT_LOCK      "\xD2\xA3\xBF\xD8\xC9\xCF\xCB\xF8"
#define ZH_HINT_UNLOCK    "\xD2\xA3\xBF\xD8\xBD\xE2\xCB\xF8"
#define ZH_HINT_RF_RESET  "\xCE\xDE\xCF\xDF\xD6\xD8\xD6\xC3"
#define ZH_HINT_RF_PAIR   "\xCE\xDE\xCF\xDF\xC5\xE4\xB6\xD4"
#define ZH_HINT_RF_FIRST  "\xCF\xC8\xD6\xD8\xD6\xC3"
#define ZH_HINT_SAVE_OK   "\xB1\xA3\xB4\xE6\xB3\xC9\xB9\xA6"
#define ZH_HINT_CAL_OK    "\xD0\xA3\xD7\xBC\xB3\xC9\xB9\xA6"
#define ZH_HINT_FC_CAL    "\xD2\xD1\xB7\xA2\xD0\xA3\xD7\xBC"
#define ZH_HINT_MENU      "\xB2\xCB\xB5\xA5\xB4\xF2\xBF\xAA"
#define ZH_MENU_LIST      "\xB2\xCB\xB5\xA5\xC1\xD0\xB1\xED"
#define ZH_MENU_SLEEP     "\xD0\xDD\xC3\xDF\xC9\xE8\xD6\xC3"
#define ZH_MENU_TRIM      "\xCE\xA2\xB5\xF7\xC9\xE8\xD6\xC3"
#define ZH_MENU_STICK     "\xD2\xA1\xB8\xCB\xD0\xA3\xD7\xBC"
#define ZH_MENU_FC        "\xB7\xC9\xBF\xD8\xD0\xA3\xD7\xBC"
#define ZH_MENU_RF        "\xCE\xDE\xCF\xDF\xC9\xE8\xD6\xC3"
#define ZH_FC_SLEEP       "\xB7\xC9\xBF\xD8\xD0\xDD\xC3\xDF"
#define ZH_TIME_FMT       "\xCA\xB1\xBC\xE4:%3u\xC3\xEB"
#define ZH_NEVER_LOCK     "0=\xD3\xC0\xB2\xBB\xC9\xCF\xCB\xF8"
#define ZH_SAVE_BACK      "S1\xB1\xA3\xB4\xE6S2\xB7\xB5"
#define ZH_CONFIRM_BACK   "S1\xC8\xB7\xC8\xCFS2\xB7\xB5"
#define ZH_TRIM_THR_FMT   "%c\xD3\xCD\xC3\xC5:%5d"
#define ZH_TRIM_ROL_FMT   "%c\xBA\xE1\xB9\xF6:%5d"
#define ZH_TRIM_PIT_FMT   "%c\xB8\xA9\xD1\xF6:%5d"
#define ZH_STICK_PT_FMT   "\xB8\xA9%4u \xD3\xCD%4u"
#define ZH_STICK_RY_FMT   "\xB9\xF6%4u \xBA\xBD%4u"
#define ZH_STICK_TIP1     "\xC7\xEB\xD7\xAA\xB6\xAF\xD2\xA1\xB8\xCB"
#define ZH_STICK_TIP2     "\xD2\xBB\xD6\xDC\xD6\xB4\xD0\xD0\xD0\xA3\xD7\xBC"
#define ZH_FC_TIP1        "\xC7\xEB\xCF\xC8\xBD\xAB\xB7\xC9\xBF\xD8\xB7\xC5\xD6\xC3"
#define ZH_FC_TIP2        "\xCB\xAE\xC6\xBD\xD4\xD9\xD6\xB4\xD0\xD0\xD0\xA3\xD7\xBC"
#define ZH_CONFIRM_CANCEL "\xC8\xB7\xC8\xCF  \xC8\xA1\xCF\xFB"
#define ZH_KEEP_LEVEL     "\xB1\xA3\xB3\xD6\xCB\xAE\xC6\xBD"
#define ZH_SEND_CAL       "S1\xB7\xA2\xCB\xCD\xD0\xA3\xD7\xBC"
#define ZH_BACK           "S2\xB7\xB5\xBB\xD8"
#define ZH_RF_RESET_FMT   "%c1\xD6\xD8\xD6\xC3"
#define ZH_RF_PAIR_FMT    "%c2\xC5\xE4\xB6\xD4"

typedef struct
{
  uint16_t magic;
  uint16_t rf_build_id;
  int16_t trim_thr;
  int16_t trim_yaw;
  int16_t trim_roll;
  int16_t trim_pitch;
  uint16_t sleep_seconds;
  uint16_t fc_sleep_seconds;
  uint16_t rf_channel;
  uint16_t rf_addr_01;
  uint16_t rf_addr_23;
  uint16_t rf_addr_4;
  uint16_t adc_min[REMOTE_STICK_AXIS_COUNT];
  uint16_t adc_mid[REMOTE_STICK_AXIS_COUNT];
  uint16_t adc_max[REMOTE_STICK_AXIS_COUNT];
} _st_RemoteConfig;

typedef enum
{
  REMOTE_STATE_LOCKED = 0,
  REMOTE_STATE_UNLOCKED,
  REMOTE_STATE_MENU_ROOT,
  REMOTE_STATE_MENU_SLEEP,
  REMOTE_STATE_MENU_TRIM,
  REMOTE_STATE_MENU_STICK_CAL,
  REMOTE_STATE_MENU_FC_CAL,
  REMOTE_STATE_MENU_RF
} remote_state_t;

typedef enum
{
  MENU_ITEM_SLEEP = 0,
  MENU_ITEM_TRIM,
  MENU_ITEM_STICK_CAL,
  MENU_ITEM_FC_CAL,
  MENU_ITEM_RF,
  MENU_ITEM_COUNT
} remote_menu_item_t;

_st_Remote Remote = {1000, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
__IO uint16_t ADC_ConvertedValue[5];
uint32_t SysTick_count = 0;
_st_Offset offset;

static _st_RemoteConfig g_cfg;
static remote_state_t g_state = REMOTE_STATE_LOCKED;
static uint16_t g_live_thr = 1000;
static uint16_t g_live_yaw = 1500;
static uint16_t g_live_roll = 1500;
static uint16_t g_live_pitch = 1500;
static uint16_t g_tx_battery_mv = 0;
static uint16_t g_fc_battery_mv = 0;
static uint8_t g_menu_index = 0;
static uint8_t g_sleep_index = 0;
static uint8_t g_trim_index = 0;
static uint8_t g_rf_index = 0;
static uint8_t g_fc_cal_ticks = 0;
static uint8_t g_rf_reset_ready = 0;
static uint8_t g_rf_switch_delay_ticks = 0;
static uint8_t g_rf_pending_pair_hint = 0;
volatile uint8_t g_remote_soft_sleeping = 0;
static uint16_t g_cal_min[REMOTE_STICK_AXIS_COUNT];
static uint16_t g_cal_max[REMOTE_STICK_AXIS_COUNT];
static uint32_t g_last_input_tick = 0;
static uint32_t g_last_telemetry_tick = 0;
static uint32_t g_last_rf_ack_tick = 0;
static uint32_t g_hint_until_tick = 0;
static char g_hint_line[17] = "";
static uint8_t g_oled_cache_valid = 0;
static uint8_t g_oled_line_cache[4][17];
static uint8_t g_home_icon_cache = 0xFFU;
static uint16_t g_home_display_mv_cache = 0xFFFFU;
static uint8_t g_home_rf_cache = 0xFFU;
static uint16_t g_home_channel_cache = 0xFFFFU;

static void load_default_config(void);
static void load_config_from_flash(void);
static void save_config_to_flash(void);
static void apply_config(void);
static void center_trim_calibration(void);
static uint16_t map_linear_channel(uint16_t raw, uint16_t min_value, uint16_t max_value, uint8_t invert);
static uint16_t map_center_channel(uint16_t raw, uint16_t min_value, uint16_t mid_value, uint16_t max_value, uint8_t invert);
static uint16_t adc_to_battery_mv(uint16_t raw);
static void init_stick_calibration_capture(void);
static void update_stick_calibration_capture(void);
static void save_stick_calibration_capture(void);
static void sample_live_channels(void);
static void update_remote_output(void);
static void process_input_events(void);
static void set_remote_locked(uint8_t locked);
static void enter_menu(void);
static void exit_menu(void);
static void handle_nav_up(void);
static void handle_nav_down(void);
static void handle_nav_left(void);
static void handle_nav_right(void);
static void handle_s1_short(void);
static void handle_s2_short(void);
static void draw_ascii_line(uint8_t row, const char *text);
static void draw_menu_line(uint8_t row, const char *text);
static void draw_lock_status_icon(uint8_t locked);
static void clear_status_icon_area(void);
static void draw_home_top_line(const char *text);
static void show_voltage(uint8_t x, uint8_t row, uint16_t mv);
static void render_hint_line(void);
static void set_hint(const char *text, uint16_t duration_tick);
static void draw_home_icon16(uint8_t x, uint8_t page, const uint8_t *icon);
static void draw_signal_icon(uint8_t strength_level);
static void draw_battery_icon(void);
static void render_home(void);
static void render_menu_root(void);
static void render_menu_sleep(void);
static void render_menu_trim(void);
static void render_menu_stick_cal(void);
static void render_menu_fc_cal(void);
static void render_menu_rf(void);
static void randomize_rf_pair(void);
static void apply_rf_defaults(void);
static void schedule_rf_switch(uint8_t pair_hint);
static uint8_t service_rf_switch(void);
static void send_fc_sleep_command(void);
static void send_rf_pair_command(void);
static void send_remote_command_frame(uint8_t func_id, const uint8_t *payload, uint8_t payload_len, uint8_t repeat);
static void enter_standby_if_needed(void);
static void set_safe_channels(void);
static uint8_t button_is_pressed_port(GPIO_TypeDef *port, uint16_t pin);
static void invalidate_oled_cache(void);
void NRF_SEND(void);

static void load_default_config(void)
{
  uint8_t i;

  /* Default values used after factory reset.
   * Keep RF reset behavior consistent with the task document.
   */
  g_cfg.magic = REMOTE_CFG_MAGIC;
  g_cfg.rf_build_id = REMOTE_RF_BUILD_ID;
  g_cfg.trim_thr = 0;
  g_cfg.trim_yaw = 0;
  g_cfg.trim_roll = 0;
  g_cfg.trim_pitch = 0;
  /* Default local soft-sleep timeout is 60 seconds; setting 0 disables it. */
  g_cfg.sleep_seconds = REMOTE_LOCAL_STANDBY_SECONDS;
  g_cfg.fc_sleep_seconds = 60;
  g_cfg.rf_channel = 0;
  g_cfg.rf_addr_01 = 0x2211;
  g_cfg.rf_addr_23 = 0x4433;
  g_cfg.rf_addr_4 = 0x0055;
  for (i = 0; i < REMOTE_STICK_AXIS_COUNT; i++)
  {
    g_cfg.adc_min[i] = 0;
    g_cfg.adc_mid[i] = 2048;
    g_cfg.adc_max[i] = 4095;
  }
}

static void load_config_from_flash(void)
{
  const _st_RemoteConfig *flash_cfg = (const _st_RemoteConfig *)FLASH_Start_Addr;

  if (flash_cfg->magic != REMOTE_CFG_MAGIC)
  {
    load_default_config();
    center_trim_calibration();
    g_cfg.trim_thr = offset.thr;
    g_cfg.trim_yaw = offset.yaw;
    g_cfg.trim_roll = offset.roll;
    g_cfg.trim_pitch = offset.pitch;
    save_config_to_flash();
    return;
  }

  memcpy(&g_cfg, flash_cfg, sizeof(g_cfg));

  if (g_cfg.rf_build_id != REMOTE_RF_BUILD_ID)
  {
    /* Build id changed after flashing: reset both sides to default RF config. */
    g_cfg.rf_build_id = REMOTE_RF_BUILD_ID;
    g_cfg.rf_channel = 0;
    g_cfg.rf_addr_01 = 0x2211;
    g_cfg.rf_addr_23 = 0x4433;
    g_cfg.rf_addr_4 = 0x0055;
    save_config_to_flash();
  }

  if ((g_cfg.sleep_seconds > REMOTE_SLEEP_MAX) ||
      (g_cfg.fc_sleep_seconds > REMOTE_SLEEP_MAX) ||
      (g_cfg.adc_mid[0] > 4095U) || (g_cfg.adc_mid[1] > 4095U) ||
      (g_cfg.adc_mid[2] > 4095U) || (g_cfg.adc_mid[3] > 4095U))
  {
    int16_t keep_thr = g_cfg.trim_thr;
    int16_t keep_yaw = g_cfg.trim_yaw;
    int16_t keep_roll = g_cfg.trim_roll;
    int16_t keep_pitch = g_cfg.trim_pitch;
    uint16_t keep_channel = g_cfg.rf_channel;
    uint16_t keep_addr_01 = g_cfg.rf_addr_01;
    uint16_t keep_addr_23 = g_cfg.rf_addr_23;
    uint16_t keep_addr_4 = g_cfg.rf_addr_4;

    load_default_config();
    g_cfg.sleep_seconds = REMOTE_LOCAL_STANDBY_SECONDS;
    g_cfg.trim_thr = keep_thr;
    g_cfg.trim_yaw = keep_yaw;
    g_cfg.trim_roll = keep_roll;
    g_cfg.trim_pitch = keep_pitch;
    g_cfg.rf_channel = keep_channel;
    g_cfg.rf_addr_01 = keep_addr_01;
    g_cfg.rf_addr_23 = keep_addr_23;
    g_cfg.rf_addr_4 = keep_addr_4;
  }
}

static void save_config_to_flash(void)
{
  g_cfg.magic = REMOTE_CFG_MAGIC;
  g_cfg.rf_build_id = REMOTE_RF_BUILD_ID;
  writedata_to_flash((int16_t *)&g_cfg, sizeof(g_cfg) / 2U, FLASH_Start_Addr);
}

static void apply_config(void)
{
  uint8_t rf_addr[5];

  offset.flag = 0x0066;
  offset.thr = g_cfg.trim_thr;
  offset.yaw = g_cfg.trim_yaw;
  offset.roll = g_cfg.trim_roll;
  offset.pitch = g_cfg.trim_pitch;

  rf_addr[0] = (uint8_t)(g_cfg.rf_addr_01 & 0xFF);
  rf_addr[1] = (uint8_t)(g_cfg.rf_addr_01 >> 8);
  rf_addr[2] = (uint8_t)(g_cfg.rf_addr_23 & 0xFF);
  rf_addr[3] = (uint8_t)(g_cfg.rf_addr_23 >> 8);
  rf_addr[4] = (uint8_t)(g_cfg.rf_addr_4 & 0xFF);
  NRF24L01_SetChannelAddress((uint8_t)g_cfg.rf_channel, rf_addr);
  /* Pairing is allowed only after the RF reset step required by the task document. */
  g_rf_reset_ready = (uint8_t)((g_cfg.rf_channel == 0U) &&
                               (g_cfg.rf_addr_01 == 0x2211U) &&
                               (g_cfg.rf_addr_23 == 0x4433U) &&
                               (g_cfg.rf_addr_4 == 0x0055U));
}

static void send_remote_command_frame(uint8_t func_id, const uint8_t *payload, uint8_t payload_len, uint8_t repeat)
{
  uint8_t frame[32];
  uint8_t i;
  uint8_t frame_len;
  uint8_t checksum = 0U;

  if ((payload_len + 5U) > sizeof(frame))
  {
    return;
  }

  frame[0] = 0xAAU;
  frame[1] = 0xAFU;
  frame[2] = func_id;
  frame[3] = payload_len;
  for (i = 0U; i < payload_len; i++)
  {
    frame[4U + i] = (payload != NULL) ? payload[i] : 0U;
  }

  frame_len = (uint8_t)(payload_len + 4U);
  for (i = 0U; i < frame_len; i++)
  {
    checksum = (uint8_t)(checksum + frame[i]);
  }
  frame[frame_len] = checksum;
  frame_len = (uint8_t)(frame_len + 1U);

  /* Menu command frames are not periodic, so send them a few times to improve delivery. */
  if (repeat == 0U)
  {
    repeat = 1U;
  }
  /* This path may run from SysTick-driven RC_Analy(), so do not use HAL_Delay here.
   * Send repeated command frames quickly without blocking the interrupt context.
   */
  for (i = 0U; i < repeat; i++)
  {
    /* Mirror the outgoing command frame to USB before NRF TX for bench testing. */
    Remote_MirrorTxFrame_ToUsb(frame, frame_len);
    NRF24L01_TxPacket(frame, frame_len);
  }
}

static void send_fc_sleep_command(void)
{
  uint8_t payload[2];

  /* Remote -> FC sleep command frame: 0x05 + uint16 seconds. */
  payload[0] = (uint8_t)((g_cfg.fc_sleep_seconds >> 8) & 0xFFU);
  payload[1] = (uint8_t)(g_cfg.fc_sleep_seconds & 0xFFU);
  send_remote_command_frame(0x05U, payload, sizeof(payload), 3U);
}

static void send_rf_pair_command(void)
{
  uint8_t payload[6];

  /* Remote -> FC RF command frame: 0x04 + channel + 5-byte address.
   * Reset and pair share the same frame format; only the payload differs.
   */
  payload[0] = (uint8_t)g_cfg.rf_channel;
  payload[1] = (uint8_t)(g_cfg.rf_addr_01 & 0xFFU);
  payload[2] = (uint8_t)(g_cfg.rf_addr_01 >> 8);
  payload[3] = (uint8_t)(g_cfg.rf_addr_23 & 0xFFU);
  payload[4] = (uint8_t)(g_cfg.rf_addr_23 >> 8);
  payload[5] = (uint8_t)(g_cfg.rf_addr_4 & 0xFFU);
  send_remote_command_frame(0x04U, payload, sizeof(payload), 5U);
}

static void center_trim_calibration(void)
{
  uint8_t i;
  int32_t sum_thr = 0;
  int32_t sum_yaw = 0;
  int32_t sum_roll = 0;
  int32_t sum_pitch = 0;

  /* This calibration can be triggered by menu S1 from the SysTick-driven flow.
   * Avoid HAL_Delay and average several immediate ADC samples instead.
   */
  for (i = 0; i < 20; i++)
  {
    sum_thr += ADC_ConvertedValue[REMOTE_ADC_THR_INDEX];
    sum_yaw += ADC_ConvertedValue[REMOTE_ADC_YAW_INDEX] - 2000;
    sum_roll += ADC_ConvertedValue[REMOTE_ADC_ROL_INDEX] - 2000;
    sum_pitch += ADC_ConvertedValue[REMOTE_ADC_PIT_INDEX] - 2000;
  }

  offset.thr = (int16_t)(-(sum_thr + 40) / 80);
  offset.yaw = (int16_t)((sum_yaw + 40) / 80);
  offset.roll = (int16_t)((sum_roll + 40) / 80);
  offset.pitch = (int16_t)(-(sum_pitch + 40) / 80);

  g_cfg.adc_mid[REMOTE_ADC_YAW_INDEX] = ADC_ConvertedValue[REMOTE_ADC_YAW_INDEX];
  g_cfg.adc_mid[REMOTE_ADC_THR_INDEX] = ADC_ConvertedValue[REMOTE_ADC_THR_INDEX];
  g_cfg.adc_mid[REMOTE_ADC_ROL_INDEX] = ADC_ConvertedValue[REMOTE_ADC_ROL_INDEX];
  g_cfg.adc_mid[REMOTE_ADC_PIT_INDEX] = ADC_ConvertedValue[REMOTE_ADC_PIT_INDEX];
}

void RC_INIT(void)
{
  /* Boot-time remote initialization.
   * Restore saved menu, trim, calibration and RF settings from Flash.
   * The default state stays locked, so no unsafe channel output is sent on power-up.
   */
  load_config_from_flash();
  apply_config();
  g_last_input_tick = SysTick_count;
}

static uint16_t map_linear_channel(uint16_t raw, uint16_t min_value, uint16_t max_value, uint8_t invert)
{
  uint32_t value;

  if (max_value <= min_value)
  {
    value = 1500U;
  }
  else if (raw <= min_value)
  {
    value = 1000U;
  }
  else if (raw >= max_value)
  {
    value = 2000U;
  }
  else
  {
    value = 1000U + ((uint32_t)(raw - min_value) * 1000U) / (uint32_t)(max_value - min_value);
  }

  if (invert != 0U)
  {
    value = 3000U - value;
  }
  return (uint16_t)value;
}

static uint16_t map_center_channel(uint16_t raw, uint16_t min_value, uint16_t mid_value, uint16_t max_value, uint8_t invert)
{
  uint32_t value;

  if ((mid_value <= min_value) || (max_value <= mid_value))
  {
    value = 1500U;
  }
  else if (raw <= mid_value)
  {
    if (raw <= min_value)
    {
      value = 1000U;
    }
    else
    {
      value = 1000U + ((uint32_t)(raw - min_value) * 500U) / (uint32_t)(mid_value - min_value);
    }
  }
  else
  {
    if (raw >= max_value)
    {
      value = 2000U;
    }
    else
    {
      value = 1500U + ((uint32_t)(raw - mid_value) * 500U) / (uint32_t)(max_value - mid_value);
    }
  }

  if (invert != 0U)
  {
    value = 3000U - value;
  }
  return (uint16_t)value;
}

static uint16_t adc_to_battery_mv(uint16_t raw)
{
  uint32_t mv = (uint32_t)raw * REMOTE_BAT_REF_MV * REMOTE_BAT_DIV_NUM;

  mv /= (4095U * REMOTE_BAT_DIV_DEN);
  return (uint16_t)mv;
}

static void init_stick_calibration_capture(void)
{
  uint8_t i;

  for (i = 0; i < REMOTE_STICK_AXIS_COUNT; i++)
  {
    g_cal_min[i] = ADC_ConvertedValue[i];
    g_cal_max[i] = ADC_ConvertedValue[i];
  }
}

static void update_stick_calibration_capture(void)
{
  uint8_t i;

  for (i = 0; i < REMOTE_STICK_AXIS_COUNT; i++)
  {
    if (ADC_ConvertedValue[i] < g_cal_min[i])
    {
      g_cal_min[i] = ADC_ConvertedValue[i];
    }
    if (ADC_ConvertedValue[i] > g_cal_max[i])
    {
      g_cal_max[i] = ADC_ConvertedValue[i];
    }
  }
}

static void save_stick_calibration_capture(void)
{
  uint8_t i;

  for (i = 0; i < REMOTE_STICK_AXIS_COUNT; i++)
  {
    if ((g_cal_max[i] > g_cal_min[i]) && ((g_cal_max[i] - g_cal_min[i]) > 500U))
    {
      g_cfg.adc_min[i] = g_cal_min[i];
      g_cfg.adc_max[i] = g_cal_max[i];
    }
    g_cfg.adc_mid[i] = ADC_ConvertedValue[i];
  }
}

static void sample_live_channels(void)
{
  static uint16_t last_thr = 1000;
  static uint16_t last_yaw = 1500;
  static uint16_t last_roll = 1500;
  static uint16_t last_pitch = 1500;
  uint16_t thr;
  uint16_t yaw;
  uint16_t roll;
  uint16_t pitch;

  /* Convert raw ADC values to 1000-2000 remote-control channels.
   * This also applies calibration, trim, simple filtering and local battery sampling.
   */
  thr = (uint16_t)(map_linear_channel(
      ADC_ConvertedValue[REMOTE_ADC_THR_INDEX],
      g_cfg.adc_min[REMOTE_ADC_THR_INDEX],
      g_cfg.adc_max[REMOTE_ADC_THR_INDEX],
      0U) + offset.thr);
  yaw = (uint16_t)(map_center_channel(
      ADC_ConvertedValue[REMOTE_ADC_YAW_INDEX],
      g_cfg.adc_min[REMOTE_ADC_YAW_INDEX],
      g_cfg.adc_mid[REMOTE_ADC_YAW_INDEX],
      g_cfg.adc_max[REMOTE_ADC_YAW_INDEX],
      1U) + offset.yaw);
  roll = (uint16_t)(map_center_channel(
      ADC_ConvertedValue[REMOTE_ADC_ROL_INDEX],
      g_cfg.adc_min[REMOTE_ADC_ROL_INDEX],
      g_cfg.adc_mid[REMOTE_ADC_ROL_INDEX],
      g_cfg.adc_max[REMOTE_ADC_ROL_INDEX],
      1U) + offset.roll);
  pitch = (uint16_t)(map_center_channel(
      ADC_ConvertedValue[REMOTE_ADC_PIT_INDEX],
      g_cfg.adc_min[REMOTE_ADC_PIT_INDEX],
      g_cfg.adc_mid[REMOTE_ADC_PIT_INDEX],
      g_cfg.adc_max[REMOTE_ADC_PIT_INDEX],
      0U) + offset.pitch);

  if (thr < 1000U) thr = 1000U;
  if (thr > 2000U) thr = 2000U;
  if (yaw < 1000U) yaw = 1000U;
  if (yaw > 2000U) yaw = 2000U;
  if (roll < 1000U) roll = 1000U;
  if (roll > 2000U) roll = 2000U;
  if (pitch < 1000U) pitch = 1000U;
  if (pitch > 2000U) pitch = 2000U;

  g_live_thr = (uint16_t)((thr + 3U * last_thr + 2U) >> 2);
  g_live_yaw = (uint16_t)((yaw + 3U * last_yaw + 2U) >> 2);
  g_live_roll = (uint16_t)((roll + 3U * last_roll + 2U) >> 2);
  g_live_pitch = (uint16_t)((pitch + 3U * last_pitch + 2U) >> 2);

  last_thr = g_live_thr;
  last_yaw = g_live_yaw;
  last_roll = g_live_roll;
  last_pitch = g_live_pitch;

  /* Local battery voltage comes from BAT_DET.
   * Low-pass filtering avoids OLED flicker caused by tiny voltage changes.
   */
  {
    uint16_t bat_mv = adc_to_battery_mv(ADC_ConvertedValue[REMOTE_ADC_BAT_INDEX]);
    if (g_tx_battery_mv == 0U)
    {
      g_tx_battery_mv = bat_mv;
    }
    else
    {
      g_tx_battery_mv = (uint16_t)((3U * g_tx_battery_mv + bat_mv + 2U) >> 2);
    }
  }

  if (g_state == REMOTE_STATE_MENU_STICK_CAL)
  {
    update_stick_calibration_capture();
  }
}

static void set_safe_channels(void)
{
  Remote.thr = 1000;
  Remote.yaw = 1500;
  Remote.roll = 1500;
  Remote.pitch = 1500;
}

static void update_remote_output(void)
{
  /* When unlocked, send live stick channels; when locked, send safe neutral values.
   * The original NRF packet layout is preserved.
   */
  if (g_state == REMOTE_STATE_UNLOCKED)
  {
    Remote.thr = g_live_thr;
    Remote.yaw = g_live_yaw;
    Remote.roll = g_live_roll;
    Remote.pitch = g_live_pitch;
  }
  else
  {
    set_safe_channels();
  }

  /* FC calibration reuses AUX7 as a one-shot trigger.
   * After user confirmation, AUX7 is raised briefly so the FC can detect it.
   */
  if (g_fc_cal_ticks > 0U)
  {
    Remote.AUX7 = 2000;
    g_fc_cal_ticks--;
  }
  else
  {
    Remote.AUX7 = 1000;
  }
}

static void set_remote_locked(uint8_t locked)
{
  /* Central lock/unlock transition handler.
   * This keeps safe output, hints and idle timer reset in one place.
   */
  if (locked != 0U)
  {
    g_state = REMOTE_STATE_LOCKED;
    g_last_input_tick = SysTick_count;
    set_safe_channels();
    set_hint(ZH_HINT_LOCK, 120);
  }
  else
  {
    g_state = REMOTE_STATE_UNLOCKED;
    g_last_input_tick = SysTick_count;
    set_hint(ZH_HINT_UNLOCK, 120);
  }
}

static void enter_menu(void)
{
  /* Long-press S1 while locked to enter the menu.
   * Reset menu cursors so the previous menu position does not leak into the next session.
   */
  g_state = REMOTE_STATE_MENU_ROOT;
  g_menu_index = 0;
  g_sleep_index = 0;
  g_trim_index = 0;
  g_rf_index = 0;
}

static void exit_menu(void)
{
  g_state = REMOTE_STATE_LOCKED;
}

static uint8_t button_is_pressed_port(GPIO_TypeDef *port, uint16_t pin)
{
  return (uint8_t)((port->IDR & pin) == 0U);
}

static void invalidate_oled_cache(void)
{
  uint8_t i;
  uint8_t j;

  g_oled_cache_valid = 0;
  g_home_icon_cache = 0xFFU;
  g_home_display_mv_cache = 0xFFFFU;
  g_home_rf_cache = 0xFFU;
  g_home_channel_cache = 0xFFFFU;
  for (i = 0; i < 4U; i++)
  {
    for (j = 0; j < 17U; j++)
    {
      g_oled_line_cache[i][j] = 0U;
    }
  }
}

static void handle_nav_up(void)
{
  /* [REQ-3] Left stick up/down selects different menu entries. */
  if (g_state == REMOTE_STATE_MENU_ROOT)
  {
    if (g_menu_index > 0U)
    {
      g_menu_index--;
    }
  }
  else if (g_state == REMOTE_STATE_MENU_SLEEP)
  {
    if (g_sleep_index > 0U)
    {
      g_sleep_index--;
    }
  }
  else if (g_state == REMOTE_STATE_MENU_TRIM)
  {
    if (g_trim_index > 0U)
    {
      g_trim_index--;
    }
  }
  else if (g_state == REMOTE_STATE_MENU_RF)
  {
    g_rf_index = 0;
  }
}

static void handle_nav_down(void)
{
  if (g_state == REMOTE_STATE_MENU_ROOT)
  {
    if (g_menu_index + 1U < MENU_ITEM_COUNT)
    {
      g_menu_index++;
    }
  }
  else if (g_state == REMOTE_STATE_MENU_SLEEP)
  {
    if (g_sleep_index < 1U)
    {
      g_sleep_index++;
    }
  }
  else if (g_state == REMOTE_STATE_MENU_TRIM)
  {
    if (g_trim_index < 2U)
    {
      g_trim_index++;
    }
  }
  else if (g_state == REMOTE_STATE_MENU_RF)
  {
    g_rf_index = 1;
  }
}

static void handle_nav_left(void)
{
  /* [REQ-3][REQ-4][REQ-5] Left stick left/right adjusts parameter values. */
  if (g_state == REMOTE_STATE_MENU_SLEEP)
  {
    uint16_t *target = (g_sleep_index == 0U) ? &g_cfg.sleep_seconds : &g_cfg.fc_sleep_seconds;

    if (*target >= REMOTE_SLEEP_STEP)
    {
      *target = (uint16_t)(*target - REMOTE_SLEEP_STEP);
    }
  }
  else if (g_state == REMOTE_STATE_MENU_TRIM)
  {
    if (g_trim_index == 0U)
    {
      g_cfg.trim_thr -= REMOTE_MENU_STEP;
    }
    else if (g_trim_index == 1U)
    {
      g_cfg.trim_roll -= REMOTE_MENU_STEP;
    }
    else
    {
      g_cfg.trim_pitch -= REMOTE_MENU_STEP;
    }
    apply_config();
  }
}

static void handle_nav_right(void)
{
  if (g_state == REMOTE_STATE_MENU_SLEEP)
  {
    uint16_t *target = (g_sleep_index == 0U) ? &g_cfg.sleep_seconds : &g_cfg.fc_sleep_seconds;

    if ((*target + REMOTE_SLEEP_STEP) <= REMOTE_SLEEP_MAX)
    {
      *target = (uint16_t)(*target + REMOTE_SLEEP_STEP);
    }
  }
  else if (g_state == REMOTE_STATE_MENU_TRIM)
  {
    if (g_trim_index == 0U)
    {
      g_cfg.trim_thr += REMOTE_MENU_STEP;
    }
    else if (g_trim_index == 1U)
    {
      g_cfg.trim_roll += REMOTE_MENU_STEP;
    }
    else
    {
      g_cfg.trim_pitch += REMOTE_MENU_STEP;
    }
    apply_config();
  }
}

static void schedule_rf_switch(uint8_t pair_hint)
{
  g_rf_pending_pair_hint = pair_hint;
  g_rf_switch_delay_ticks = REMOTE_RF_SWITCH_DELAY_TICKS;
  g_last_input_tick = SysTick_count;
}

static uint8_t service_rf_switch(void)
{
  if (g_rf_switch_delay_ticks == 0U)
  {
    return 0U;
  }

  /* Keep sending the pair frame on the old channel first, then switch locally. */
  send_rf_pair_command();
  g_last_input_tick = SysTick_count;
  g_rf_switch_delay_ticks--;
  if (g_rf_switch_delay_ticks == 0U)
  {
    save_config_to_flash();
    apply_config();
    TX_Mode();
    set_hint((g_rf_pending_pair_hint != 0U) ? ZH_HINT_RF_PAIR : ZH_HINT_RF_RESET, 150);
  }
  return 1U;
}
static void apply_rf_defaults(void)
{
  /* Reset uses the task-book default channel/address, then switches after the FC has time to store it. */
  g_cfg.rf_channel = 0;
  g_cfg.rf_addr_01 = 0x2211;
  g_cfg.rf_addr_23 = 0x4433;
  g_cfg.rf_addr_4 = 0x0055;
  schedule_rf_switch(0U);
}

static void randomize_rf_pair(void)
{
  /* [REQ-8 pair] Generate random RF channel/address and store them. */
  uint32_t seed = SysTick_count + ADC_ConvertedValue[0] + (ADC_ConvertedValue[1] << 3) +
                  (ADC_ConvertedValue[2] << 7) + (ADC_ConvertedValue[3] << 11);
  uint8_t addr[5];
  uint8_t i;

  srand(seed);
  g_cfg.rf_channel = (uint16_t)((rand() % 125) + 1);
  for (i = 0; i < 5; i++)
  {
    addr[i] = (uint8_t)((rand() % 0xFE) + 1);
  }

  g_cfg.rf_addr_01 = (uint16_t)(addr[0] | ((uint16_t)addr[1] << 8));
  g_cfg.rf_addr_23 = (uint16_t)(addr[2] | ((uint16_t)addr[3] << 8));
  g_cfg.rf_addr_4 = addr[4];
  schedule_rf_switch(1U);
}

static void handle_s1_short(void)
{
  /* S1 short press means confirm / enter / execute.
   * Page-specific confirm actions are handled here.
   */
  if (g_state == REMOTE_STATE_MENU_ROOT)
  {
    switch (g_menu_index)
    {
      case MENU_ITEM_SLEEP:
        g_state = REMOTE_STATE_MENU_SLEEP;
        g_sleep_index = 0;
        break;
      case MENU_ITEM_TRIM:
        g_state = REMOTE_STATE_MENU_TRIM;
        g_trim_index = 0;
        break;
      case MENU_ITEM_STICK_CAL:
        g_state = REMOTE_STATE_MENU_STICK_CAL;
        init_stick_calibration_capture();
        break;
      case MENU_ITEM_FC_CAL:
        g_state = REMOTE_STATE_MENU_FC_CAL;
        break;
      case MENU_ITEM_RF:
        g_state = REMOTE_STATE_MENU_RF;
        g_rf_index = 0;
        break;
      default:
        break;
    }
  }
  else if ((g_state == REMOTE_STATE_MENU_SLEEP) || (g_state == REMOTE_STATE_MENU_TRIM))
  {
    save_config_to_flash();
    apply_config();
    g_last_input_tick = SysTick_count;
    if (g_state == REMOTE_STATE_MENU_SLEEP)
    {
      send_fc_sleep_command();
    }
    g_state = REMOTE_STATE_MENU_ROOT;
    set_hint(ZH_HINT_SAVE_OK, 120);
  }
  else if (g_state == REMOTE_STATE_MENU_STICK_CAL)
  {
    /* [REQ-6] Save full min/max stick calibration after one full stick rotation. */
    save_stick_calibration_capture();
    center_trim_calibration();
    g_cfg.trim_thr = offset.thr;
    g_cfg.trim_yaw = offset.yaw;
    g_cfg.trim_roll = offset.roll;
    g_cfg.trim_pitch = offset.pitch;
    save_config_to_flash();
    apply_config();
    g_state = REMOTE_STATE_MENU_ROOT;
    set_hint(ZH_HINT_CAL_OK, 150);
  }
  else if (g_state == REMOTE_STATE_MENU_FC_CAL)
  {
    /* [REQ-7] Send a short calibration trigger window to the flight controller. */
    g_fc_cal_ticks = 100;
    g_state = REMOTE_STATE_MENU_ROOT;
    set_hint(ZH_HINT_FC_CAL, 150);
  }
  else if (g_state == REMOTE_STATE_MENU_RF)
  {
    if (g_rf_index == 0U)
    {
      apply_rf_defaults();
    }
    else
    {
      /* Do not allow direct pairing here.
       * The user must run RF reset first, matching the task-document flow.
       */
      if (g_rf_reset_ready != 0U)
      {
        randomize_rf_pair();
      }
      else
      {
        set_hint(ZH_HINT_RF_FIRST, 150);
      }
    }
    g_state = REMOTE_STATE_MENU_ROOT;
  }
}

static void handle_s2_short(void)
{
  /* S2 short press means cancel / go back.
   * Root menu returns to the locked home page; submenus return to the root menu.
   */
  if (g_state == REMOTE_STATE_MENU_ROOT)
  {
    exit_menu();
  }
  else if ((g_state != REMOTE_STATE_LOCKED) && (g_state != REMOTE_STATE_UNLOCKED))
  {
    g_state = REMOTE_STATE_MENU_ROOT;
  }
}

static void process_input_events(void)
{
  /* Main input state machine.
   * Handles lock/unlock gestures, S1/S2 actions, stick menu navigation,
   * auto-lock and soft-sleep wake-up.
   */
  static uint8_t s1_prev = 0;
  static uint8_t s2_prev = 0;
  static uint8_t s1_stable = 0;
  static uint8_t s2_stable = 0;
  static uint8_t s1_debounce = 0;
  static uint8_t s2_debounce = 0;
  static uint16_t s1_hold = 0;
  static uint8_t up_armed = 1;
  static uint8_t down_armed = 1;
  static uint8_t left_armed = 1;
  static uint8_t right_armed = 1;
  static uint8_t up_count = 0;
  static uint8_t down_count = 0;
  static uint8_t left_count = 0;
  static uint8_t right_count = 0;
  static uint16_t prev_thr = 1000;
  static uint16_t prev_yaw = 1500;
  static uint16_t prev_roll = 1500;
  static uint16_t prev_pitch = 1500;
  uint16_t menu_ud_raw = ADC_ConvertedValue[REMOTE_MENU_ADC_UD_INDEX];
  uint16_t menu_lr_raw = ADC_ConvertedValue[REMOTE_MENU_ADC_LR_INDEX];
  uint16_t left_ud_raw = ADC_ConvertedValue[REMOTE_LEFT_UD_INDEX];
  uint16_t left_lr_raw = ADC_ConvertedValue[REMOTE_LEFT_LR_INDEX];
  uint16_t right_ud_raw = ADC_ConvertedValue[REMOTE_RIGHT_UD_INDEX];
  uint16_t right_lr_raw = ADC_ConvertedValue[REMOTE_RIGHT_LR_INDEX];
  uint8_t s1_raw = button_is_pressed_port(KEY5_GPIO_Port, REMOTE_S1_PIN);
  uint8_t s2_raw = button_is_pressed_port(KEY4_GPIO_Port, REMOTE_S2_PIN);
  uint8_t s1_now;
  uint8_t s2_now;
  uint8_t nav_up = (uint8_t)(menu_ud_raw > REMOTE_MENU_RAW_HIGH_TH);
  uint8_t nav_down = (uint8_t)(menu_ud_raw < REMOTE_MENU_RAW_LOW_TH);
  uint8_t nav_left = (uint8_t)(menu_lr_raw < REMOTE_MENU_RAW_LOW_TH);
  uint8_t nav_right = (uint8_t)(menu_lr_raw > REMOTE_MENU_RAW_HIGH_TH);
  uint8_t unlock_combo = (uint8_t)((left_ud_raw < REMOTE_MENU_RAW_LOW_TH) &&
                                   (left_lr_raw < REMOTE_MENU_RAW_LOW_TH) &&
                                   (right_ud_raw < REMOTE_MENU_RAW_LOW_TH) &&
                                   (right_lr_raw > REMOTE_MENU_RAW_HIGH_TH));
  uint8_t relock_combo = (uint8_t)((left_ud_raw < REMOTE_MENU_RAW_LOW_TH) &&
                                   (left_lr_raw > REMOTE_MENU_RAW_HIGH_TH) &&
                                   (right_ud_raw < REMOTE_MENU_RAW_LOW_TH) &&
                                   (right_lr_raw < REMOTE_MENU_RAW_LOW_TH));
  uint8_t allow_nav = (uint8_t)((g_state != REMOTE_STATE_LOCKED) && (g_state != REMOTE_STATE_UNLOCKED));

  if (s1_raw == s1_stable)
  {
    s1_debounce = 0U;
  }
  else
  {
    if (s1_debounce < 2U)
    {
      s1_debounce++;
    }
    if (s1_debounce >= 2U)
    {
      s1_stable = s1_raw;
      s1_debounce = 0U;
    }
  }

  if (s2_raw == s2_stable)
  {
    s2_debounce = 0U;
  }
  else
  {
    if (s2_debounce < 2U)
    {
      s2_debounce++;
    }
    if (s2_debounce >= 2U)
    {
      s2_stable = s2_raw;
      s2_debounce = 0U;
    }
  }

  s1_now = s1_stable;
  s2_now = s2_stable;

  if ((abs((int)g_live_thr - (int)prev_thr) > REMOTE_IDLE_ADC_DELTA) ||
      (abs((int)g_live_yaw - (int)prev_yaw) > REMOTE_IDLE_ADC_DELTA) ||
      (abs((int)g_live_roll - (int)prev_roll) > REMOTE_IDLE_ADC_DELTA) ||
      (abs((int)g_live_pitch - (int)prev_pitch) > REMOTE_IDLE_ADC_DELTA) ||
      (s1_now != 0U) || (s2_now != 0U))
  {
    g_last_input_tick = SysTick_count;
    if (g_remote_soft_sleeping != 0U)
    {
      g_remote_soft_sleeping = 0U;
      OLED_Display_On();
      invalidate_oled_cache();
    }
  }

  prev_thr = g_live_thr;
  prev_yaw = g_live_yaw;
  prev_roll = g_live_roll;
  prev_pitch = g_live_pitch;

  /* Lock/unlock gestures use raw ADC corners instead of mapped channels,
   * so stick remapping will not break the safety gesture.
   */
  if ((g_state == REMOTE_STATE_LOCKED) && (unlock_combo != 0U))
  {
    set_remote_locked(0);
  }
  else if ((g_state == REMOTE_STATE_UNLOCKED) && (relock_combo != 0U))
  {
    set_remote_locked(1);
  }

  if ((g_state == REMOTE_STATE_UNLOCKED) &&
      ((SysTick_count - g_last_input_tick) >= REMOTE_UNLOCK_TIMEOUT))
  {
    set_remote_locked(1);
  }

  if (s1_now != 0U)
  {
    if (s1_hold < 0xFFFFU)
    {
      s1_hold++;
    }
    if ((g_state == REMOTE_STATE_LOCKED) && (s1_hold == REMOTE_LONG_PRESS_TICK))
    {
      enter_menu();
      set_hint(ZH_HINT_MENU, 120);
    }

    if ((s1_prev == 0U) &&
        (g_state != REMOTE_STATE_LOCKED) &&
        (g_state != REMOTE_STATE_UNLOCKED))
    {
      /* Menu S1 confirms on press instead of release.
       * This is more stable with the current key scan timing.
       */
      handle_s1_short();
    }
  }
  else
  {
    s1_hold = 0;
  }

  /* S2 returns on press and is mapped to the requested board key. */
  if ((s2_prev == 0U) && (s2_now != 0U))
  {
    handle_s2_short();
  }

  if (allow_nav != 0U)
  {
    /* Stick menu navigation is edge-triggered after returning to center.
     * This prevents one held stick direction from skipping across the menu.
     */
    if ((menu_ud_raw >= REMOTE_MENU_RAW_NL_TH) && (menu_ud_raw <= REMOTE_MENU_RAW_NH_TH))
    {
      up_armed = 1U;
      down_armed = 1U;
      up_count = 0U;
      down_count = 0U;
    }
    if ((menu_lr_raw >= REMOTE_MENU_RAW_NL_TH) && (menu_lr_raw <= REMOTE_MENU_RAW_NH_TH))
    {
      left_armed = 1U;
      right_armed = 1U;
      left_count = 0U;
      right_count = 0U;
    }

    if ((nav_up != 0U) && (up_armed != 0U))
    {
      if (up_count < REMOTE_NAV_CONFIRM_TICK)
      {
        up_count++;
      }
      if (up_count >= REMOTE_NAV_CONFIRM_TICK)
      {
        handle_nav_up();
        up_armed = 0U;
        up_count = 0U;
      }
    }
    else
    {
      up_count = 0U;
    }

    if ((nav_down != 0U) && (down_armed != 0U))
    {
      if (down_count < REMOTE_NAV_CONFIRM_TICK)
      {
        down_count++;
      }
      if (down_count >= REMOTE_NAV_CONFIRM_TICK)
      {
        handle_nav_down();
        down_armed = 0U;
        down_count = 0U;
      }
    }
    else
    {
      down_count = 0U;
    }

    if ((nav_left != 0U) && (left_armed != 0U))
    {
      if (left_count < REMOTE_NAV_CONFIRM_TICK)
      {
        left_count++;
      }
      if (left_count >= REMOTE_NAV_CONFIRM_TICK)
      {
        handle_nav_left();
        left_armed = 0U;
        left_count = 0U;
      }
    }
    else
    {
      left_count = 0U;
    }

    if ((nav_right != 0U) && (right_armed != 0U))
    {
      if (right_count < REMOTE_NAV_CONFIRM_TICK)
      {
        right_count++;
      }
      if (right_count >= REMOTE_NAV_CONFIRM_TICK)
      {
        handle_nav_right();
        right_armed = 0U;
        right_count = 0U;
      }
    }
    else
    {
      right_count = 0U;
    }
  }
  else
  {
    up_armed = 1U;
    down_armed = 1U;
    left_armed = 1U;
    right_armed = 1U;
    up_count = 0U;
    down_count = 0U;
    left_count = 0U;
    right_count = 0U;
  }

  s1_prev = s1_now;
  s2_prev = s2_now;
}

void RC_Analy(void)
{
  /* [REQ-BASE] Keep 10ms periodic workflow and extend it with new features. */
  sample_live_channels();
  process_input_events();
  update_remote_output();
  if (service_rf_switch() == 0U)
  {
    NRF_SEND();
    enter_standby_if_needed();
  }
}

void Remote_OnTelemetry(uint8_t *data, uint8_t len)
{
  /* [REQ-3] Try to extract FC voltage from telemetry if the packet already carries it. */
  g_last_telemetry_tick = SysTick_count;
  g_last_rf_ack_tick = SysTick_count;

  if ((len >= 6U) && (data[0] == 0xAAU) && (data[1] == 0xAFU))
  {
    uint16_t maybe_mv = (uint16_t)((data[4] << 8) | data[5]);
    if ((maybe_mv >= 3000U) && (maybe_mv <= 26000U))
    {
      g_fc_battery_mv = maybe_mv;
    }
  }
}

uint8_t Remote_IsLocked(void)
{
  return (uint8_t)(g_state == REMOTE_STATE_LOCKED);
}

static void draw_ascii_line(uint8_t row, const char *text)
{
  uint8_t buf[17];
  uint8_t cache_index = (uint8_t)(row >> 1);
  memset(buf, ' ', 16U);
  if (text != NULL)
  {
    strncpy((char *)buf, text, 16U);
  }
  buf[16] = '\0';

  /* [REQ-3 oled-opt] Only redraw a line when content actually changes. */
  if ((g_oled_cache_valid != 0U) &&
      (cache_index < 4U) &&
      (memcmp(g_oled_line_cache[cache_index], buf, 17U) == 0))
  {
    return;
  }

  /* [REQ-3][OLED-FIX] Clear both 8-pixel pages before redrawing a 16-pixel line.
   * This removes stale pixels on the right side when line lengths shrink.
   */
  OLED_Clearrow(row);
  OLED_Clearrow((uint8_t)(row + 1U));
  OLED_ShowHZ(0, row, (char *)buf);
  if (cache_index < 4U)
  {
    memcpy(g_oled_line_cache[cache_index], buf, 17U);
    g_oled_cache_valid = 1U;
  }
}

static void draw_menu_line(uint8_t row, const char *text)
{
  draw_ascii_line(row, text);
}

static void draw_lock_status_icon(uint8_t locked)
{
  static const uint8_t lock_top[8] =
  {
    0x00,0x0E,0x11,0x21,0x21,0x11,0x0E,0x00
  };
  static const uint8_t lock_bottom[8] =
  {
    0x7F,0x63,0x5D,0x41,0x41,0x5D,0x63,0x7F
  };
  static const uint8_t unlock_top[8] =
  {
    0x00,0x06,0x09,0x10,0x20,0x21,0x1E,0x00
  };
  static const uint8_t unlock_bottom[8] =
  {
    0x7F,0x63,0x5D,0x41,0x41,0x5D,0x63,0x7F
  };
  const uint8_t *top = (locked != 0U) ? lock_top : unlock_top;
  const uint8_t *bottom = (locked != 0U) ? lock_bottom : unlock_bottom;
  uint8_t i;

  if (g_home_icon_cache == locked)
  {
    return;
  }

  /* [REQ-2][REQ-3][ICON] Bottom-center lock status icon required by the PDF. */
  OLED_Clearrow(6);
  OLED_Clearrow(7);
  OLED_Set_Pos(60, 6);
  for (i = 0; i < 8U; i++)
  {
    OLED_WR_DATA(top[i]);
  }
  OLED_Set_Pos(60, 7);
  for (i = 0; i < 8U; i++)
  {
    OLED_WR_DATA(bottom[i]);
  }
  g_home_icon_cache = locked;
}

static void clear_status_icon_area(void)
{
  /* [REQ-3][HOME-UI] Unlocked page should not show a bottom lock icon. */
  if (g_home_icon_cache != 2U)
  {
    OLED_Clearrow(6);
    OLED_Clearrow(7);
    g_home_icon_cache = 2U;
  }
}

static void draw_home_top_line(const char *text)
{
  draw_ascii_line(0, text);
}

static void draw_home_icon16(uint8_t x, uint8_t page, const uint8_t *icon)
{
  uint8_t i;

  if (icon == NULL)
  {
    return;
  }

  OLED_Set_Pos(x, page);
  for (i = 0U; i < 16U; i++)
  {
    OLED_WR_DATA(icon[i]);
  }

  OLED_Set_Pos(x, (uint8_t)(page + 1U));
  for (i = 0U; i < 16U; i++)
  {
    OLED_WR_DATA(icon[i + 16U]);
  }
}

static void draw_signal_icon(uint8_t strength_level)
{
  static const uint8_t signal_bar_mask[5] =
  {
    0x03, 0x07, 0x0F, 0x1F, 0x3F
  };
  uint8_t i;

  /* 首页左上角信号图标按“最近一次飞控回传的时间”做 0~5 档显示，
   * It avoids fake RSSI math and is more stable on a monochrome OLED.
   */
  OLED_Clearrow(0);
  OLED_Clearrow(1);
  for (i = 0U; i < 5U; i++)
  {
    uint8_t top = (strength_level > i) ? signal_bar_mask[i] : 0x01U;
    uint8_t bottom = (strength_level > i) ? 0xFFU : 0x80U;
    uint8_t x = (uint8_t)(2U + i * 3U);

    OLED_Set_Pos(x, 0);
    OLED_WR_DATA(0x00U);
    OLED_WR_DATA(top);
    OLED_Set_Pos(x, 1);
    OLED_WR_DATA(0x00U);
    OLED_WR_DATA(bottom);
  }
}

static void draw_battery_icon(void)
{
  static const uint8_t battery[32] =
  {
    0x00,0xFC,0x84,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0x84,0xFC,0x30,0x30,0x00,
    0x00,0x1F,0x10,0x16,0x16,0x16,0x16,0x16,0x16,0x16,0x16,0x10,0x1F,0x06,0x06,0x00
  };

  draw_home_icon16(72U, 0U, battery);
}

static void show_voltage(uint8_t x, uint8_t row, uint16_t mv)
{
  char buf[8];

  if (mv == 0U)
  {
    OLED_ShowString(x, row, (uint8_t *)"--.-V", 16);
    return;
  }

  snprintf(buf, sizeof(buf), "%2u.%1uV", mv / 1000U, (mv % 1000U) / 100U);
  OLED_ShowString(x, row, (uint8_t *)buf, 16);
}

static void render_hint_line(void)
{
  if ((g_hint_line[0] != '\0') && (SysTick_count <= g_hint_until_tick))
  {
    draw_ascii_line(6, g_hint_line);
  }
}

static void set_hint(const char *text, uint16_t duration_tick)
{
  strncpy(g_hint_line, text, sizeof(g_hint_line) - 1U);
  g_hint_line[sizeof(g_hint_line) - 1U] = '\0';
  g_hint_until_tick = SysTick_count + duration_tick;
}

static void render_home(void)
{
  uint32_t link_tick = (g_last_rf_ack_tick > g_last_telemetry_tick) ? g_last_rf_ack_tick : g_last_telemetry_tick;
  uint32_t telemetry_age = (link_tick == 0U) ? 0xFFFFFFFFU : (SysTick_count - link_tick);
  uint8_t signal_level = 0U;
  /* 任务书要求：
   * Locked page shows remote battery; unlocked page shows flight-controller battery.
   * If FC telemetry has not arrived yet, show the placeholder instead of falling back.
   */
  uint16_t display_mv = (g_state == REMOTE_STATE_LOCKED) ? g_tx_battery_mv : g_fc_battery_mv;
  uint16_t rounded_mv = display_mv;
  char line[17];

  if (telemetry_age < 100U)
  {
    signal_level = 5U;
  }
  else if (telemetry_age < 200U)
  {
    signal_level = 4U;
  }
  else if (telemetry_age < 300U)
  {
    signal_level = 3U;
  }
  else if (telemetry_age < 400U)
  {
    signal_level = 2U;
  }
  else if (telemetry_age < 500U)
  {
    signal_level = 1U;
  }

  if (rounded_mv != 0U)
  {
    rounded_mv = (uint16_t)(((uint32_t)rounded_mv + 50U) / 100U * 100U);
  }

  if ((g_home_rf_cache != signal_level) ||
      (g_home_display_mv_cache != rounded_mv) ||
      (g_home_channel_cache != g_cfg.rf_channel))
  {
    char ch_line[8];
    draw_signal_icon(signal_level);
    OLED_ShowString(30, 0, (uint8_t *)"     ", 16);
    snprintf(ch_line, sizeof(ch_line), "CH%03u", g_cfg.rf_channel);
    OLED_ShowString(30, 0, (uint8_t *)ch_line, 16);
    draw_battery_icon();
    show_voltage(88, 0, rounded_mv);
    g_home_rf_cache = signal_level;
    g_home_display_mv_cache = rounded_mv;
    g_home_channel_cache = g_cfg.rf_channel;
  }

  if (g_state == REMOTE_STATE_LOCKED)
  {
    draw_ascii_line(2, "                ");
    draw_ascii_line(4, "                ");
  }
  else
  {
    /* Keep the task-document home layout and add the four ADC channel values in the middle. */
    snprintf(line, sizeof(line), "P:%4u T:%4u", g_live_pitch, g_live_thr);
    draw_ascii_line(2, line);
    snprintf(line, sizeof(line), "R:%4u Y:%4u", g_live_roll, g_live_yaw);
    draw_ascii_line(4, line);
  }

  if ((g_hint_line[0] != '\0') && (SysTick_count <= g_hint_until_tick))
  {
    render_hint_line();
  }
  else
  {
    if (g_state == REMOTE_STATE_LOCKED)
    {
      draw_lock_status_icon(1U);
    }
    else
    {
      draw_lock_status_icon(0U);
    }
  }
}

static void render_menu_root(void)
{
  static const char *menu_name[MENU_ITEM_COUNT] =
  {
    "1." ZH_MENU_SLEEP,
    "2." ZH_MENU_TRIM,
    "3." ZH_MENU_STICK,
    "4." ZH_MENU_FC,
    "5." ZH_MENU_RF
  };
  char line[17];
  uint8_t first_visible = (g_menu_index > (MENU_ITEM_COUNT - 3U)) ? (MENU_ITEM_COUNT - 3U) : g_menu_index;
  uint8_t item0 = first_visible;
  uint8_t item1 = (uint8_t)(first_visible + 1U);
  uint8_t item2 = (uint8_t)(first_visible + 2U);

  draw_menu_line(0, "  " ZH_MENU_LIST);
  snprintf(line, sizeof(line), "%c%s", (g_menu_index == item0) ? '>' : ' ', menu_name[item0]);
  draw_menu_line(2, line);
  snprintf(line, sizeof(line), "%c%s", (g_menu_index == item1) ? '>' : ' ', menu_name[item1]);
  draw_menu_line(4, line);
  snprintf(line, sizeof(line), "%c%s", (g_menu_index == item2) ? '>' : ' ', menu_name[item2]);
  draw_menu_line(6, line);
}

static void render_menu_sleep(void)
{
  char line[17];

  draw_menu_line(0, "  " ZH_MENU_SLEEP);
  snprintf(line, sizeof(line), "%cRC:%3us", (g_sleep_index == 0U) ? '>' : ' ', g_cfg.sleep_seconds);
  draw_menu_line(2, line);
  snprintf(line, sizeof(line), "%cFC:%3us", (g_sleep_index == 1U) ? '>' : ' ', g_cfg.fc_sleep_seconds);
  draw_menu_line(4, line);
  draw_menu_line(6, ZH_SAVE_BACK);
}

static void render_menu_trim(void)
{
  char line[17];

  draw_menu_line(0, "  " ZH_MENU_TRIM);
  snprintf(line, sizeof(line), ZH_TRIM_THR_FMT, (g_trim_index == 0U) ? '>' : ' ', g_cfg.trim_thr);
  draw_menu_line(2, line);
  snprintf(line, sizeof(line), ZH_TRIM_ROL_FMT, (g_trim_index == 1U) ? '>' : ' ', g_cfg.trim_roll);
  draw_menu_line(4, line);
  snprintf(line, sizeof(line), ZH_TRIM_PIT_FMT, (g_trim_index == 2U) ? '>' : ' ', g_cfg.trim_pitch);
  draw_menu_line(6, line);
}

static void render_menu_stick_cal(void)
{
  char line[17];

  draw_menu_line(0, "  " ZH_MENU_STICK);
  snprintf(line, sizeof(line), ZH_STICK_PT_FMT, g_live_pitch, g_live_thr);
  draw_menu_line(2, line);
  snprintf(line, sizeof(line), ZH_STICK_RY_FMT, g_live_roll, g_live_yaw);
  draw_menu_line(4, line);
  draw_menu_line(6, ZH_STICK_TIP2);
}

static void render_menu_fc_cal(void)
{
  draw_menu_line(0, "  " ZH_MENU_FC);
  draw_menu_line(2, ZH_FC_TIP1);
  draw_menu_line(4, ZH_FC_TIP2);
  draw_menu_line(6, ZH_CONFIRM_CANCEL);
}

static void render_menu_rf(void)
{
  char line[17];

  draw_menu_line(0, "  " ZH_MENU_RF);
  snprintf(line, sizeof(line), "%c1." ZH_HINT_RF_RESET, (g_rf_index == 0U) ? '>' : ' ');
  draw_menu_line(2, line);
  snprintf(line, sizeof(line), "%c2." ZH_HINT_RF_PAIR, (g_rf_index == 1U) ? '>' : ' ');
  draw_menu_line(4, line);
  draw_menu_line(6, ZH_CONFIRM_BACK);
}

void Remote_UI_Render(void)
{
  if (g_remote_soft_sleeping != 0U)
  {
    return;
  }
  /* Unified OLED render entry.
   * Choose the home/menu page by state and avoid unnecessary full-screen redraws.
   */
  static uint32_t last_render_tick = 0xFFFFFFFFU;
  static remote_state_t last_render_state = (remote_state_t)0xFF;
  static uint8_t last_menu_index = 0xFFU;
  static uint8_t last_sleep_index = 0xFFU;
  static uint8_t last_trim_index = 0xFFU;
  static uint8_t last_rf_index = 0xFFU;
  static uint16_t last_sleep_seconds = 0xFFFFU;
  static uint16_t last_fc_sleep_seconds = 0xFFFFU;
  static int16_t last_trim_thr = 0x7FFF;
  static int16_t last_trim_roll = 0x7FFF;
  static int16_t last_trim_pitch = 0x7FFF;
  uint8_t menu_dirty = 0U;

  if ((last_render_tick != 0xFFFFFFFFU) && ((SysTick_count - last_render_tick) < 8U))
  {
    return;
  }
  last_render_tick = SysTick_count;

  /* Clear the full screen only on page changes; normal refreshes update by line. */
  if (last_render_state != g_state)
  {
    OLED_Clear();
    invalidate_oled_cache();
    last_render_state = g_state;
    menu_dirty = 1U;
  }

  if (g_state == REMOTE_STATE_MENU_ROOT)
  {
    menu_dirty = (uint8_t)(menu_dirty || (last_menu_index != g_menu_index));
    last_menu_index = g_menu_index;
  }
  else if (g_state == REMOTE_STATE_MENU_SLEEP)
  {
    menu_dirty = (uint8_t)(menu_dirty ||
                           (last_sleep_index != g_sleep_index) ||
                           (last_sleep_seconds != g_cfg.sleep_seconds) ||
                           (last_fc_sleep_seconds != g_cfg.fc_sleep_seconds));
    last_sleep_index = g_sleep_index;
    last_sleep_seconds = g_cfg.sleep_seconds;
    last_fc_sleep_seconds = g_cfg.fc_sleep_seconds;
  }
  else if (g_state == REMOTE_STATE_MENU_TRIM)
  {
    menu_dirty = (uint8_t)(menu_dirty ||
                           (last_trim_index != g_trim_index) ||
                           (last_trim_thr != g_cfg.trim_thr) ||
                           (last_trim_roll != g_cfg.trim_roll) ||
                           (last_trim_pitch != g_cfg.trim_pitch));
    last_trim_index = g_trim_index;
    last_trim_thr = g_cfg.trim_thr;
    last_trim_roll = g_cfg.trim_roll;
    last_trim_pitch = g_cfg.trim_pitch;
  }
  else if (g_state == REMOTE_STATE_MENU_RF)
  {
    menu_dirty = (uint8_t)(menu_dirty || (last_rf_index != g_rf_index));
    last_rf_index = g_rf_index;
  }

  switch (g_state)
  {
    case REMOTE_STATE_LOCKED:
    case REMOTE_STATE_UNLOCKED:
      render_home();
      break;
    case REMOTE_STATE_MENU_ROOT:
      if (menu_dirty != 0U)
      {
        render_menu_root();
      }
      break;
    case REMOTE_STATE_MENU_SLEEP:
      if (menu_dirty != 0U)
      {
        render_menu_sleep();
      }
      break;
    case REMOTE_STATE_MENU_TRIM:
      if (menu_dirty != 0U)
      {
        render_menu_trim();
      }
      break;
    case REMOTE_STATE_MENU_STICK_CAL:
      render_menu_stick_cal();
      break;
    case REMOTE_STATE_MENU_FC_CAL:
      if (menu_dirty != 0U)
      {
        render_menu_fc_cal();
      }
      break;
    case REMOTE_STATE_MENU_RF:
      if (menu_dirty != 0U)
      {
        render_menu_rf();
      }
      break;
    default:
      render_home();
      break;
  }
}

static void enter_standby_if_needed(void)
{
  /* Soft sleep starts only when the remote is locked and idle for the configured time.
   * OLED and LEDs are turned off, but MCU/SysTick/NRF keep running so communication stays alive.
   */
  if ((g_state != REMOTE_STATE_LOCKED) ||
      (g_cfg.sleep_seconds == 0U) ||
      (g_remote_soft_sleeping != 0U) ||
      (g_rf_switch_delay_ticks != 0U) ||
      ((SysTick_count - g_last_input_tick) < ((uint32_t)g_cfg.sleep_seconds * 100U)))
  {
    return;
  }

  g_remote_soft_sleeping = 1U;
  set_remote_locked(1U);
  OLED_Display_Off();
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  /* Keep MCU, SysTick and NRF TX running so the radio link is not interrupted. */
}

uint8_t tx_data[32] = {0xAA, 0xAF, 0x03, 32 - 5};

void NRF_SEND(void)
{
  /* [REQ-BASE] Keep original anonymous-protocol TX frame builder. */
  uint8_t len = tx_data[3] + 4U;
  uint8_t i;

  for (i = 4U; i < 20U; i += 2U)
  {
    tx_data[i] = *((uint8_t *)&Remote + i - 3U);
    tx_data[i + 1U] = *((uint8_t *)&Remote + i - 4U);
  }

  /* Frame 0x03 carries only stick/aux channels.
   * FC sleep settings use frame 0x05; RF settings use frame 0x04.
   */
  for (i = 20U; i < len; i++)
  {
    tx_data[i] = 0U;
  }
  if (len > 20U)
  {
    uint8_t rc_sleep_idle = (uint8_t)((g_state == REMOTE_STATE_LOCKED) &&
                                      ((SysTick_count - g_last_input_tick) >= 10U));
    tx_data[20] = (uint8_t)(((g_state == REMOTE_STATE_LOCKED) ? 0x01U : 0x00U) |
                            ((rc_sleep_idle != 0U) ? 0x02U : 0x00U));
  }

  tx_data[len] = 0;
  for (i = 0U; i < len; i++)
  {
    tx_data[len] += tx_data[i];
  }

  if (NRF24L01_TxPacket((uint8_t *)&tx_data, len + 1U) == TX_OK)
  {
    g_last_rf_ack_tick = SysTick_count; // TX_OK说明飞控已经应答本次控制包。
  }
}
