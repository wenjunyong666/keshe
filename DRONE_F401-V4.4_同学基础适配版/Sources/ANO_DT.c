
#include <stdlib.h>
#include <string.h>
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "UART.h"
#include "main.h"
#include "myMath.h"
#include "Remote.h"
#include "NRF24L01.h"
#include "flash.h"
#include "ALL_DEFINE.h"

/******************************************************************/
//--------------------------
static uint8_t RatePID[19];
static uint8_t AnglePID[19];
static uint8_t HighPID[19];
extern uint8_t nrf2401_txbuf[];
extern uint8_t txbuf_pos;
extern uint8_t nrf2401_tx_flag;

// PID packet queue for sequential transmission
#define PID_QUEUE_SIZE 8
typedef struct {
  uint8_t func_id;  // ANTO_RATE_PID, ANTO_ANGLE_PID, etc.
  uint8_t data[32]; // Packet payload
  uint8_t len;      // Payload length
} PidPacket;

static struct {
  PidPacket packets[PID_QUEUE_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} pid_queue = {0};

static void PidQueue_Push(uint8_t func_id, const uint8_t *data, uint8_t len)
{
  if(pid_queue.count >= PID_QUEUE_SIZE) return;

  pid_queue.packets[pid_queue.tail].func_id = func_id;
  pid_queue.packets[pid_queue.tail].len = len;
  if(data != NULL && len > 0)
  {
    memcpy(pid_queue.packets[pid_queue.tail].data, data, len);
  }
  pid_queue.tail = (pid_queue.tail + 1) % PID_QUEUE_SIZE;
  pid_queue.count++;
}

static uint8_t PidQueue_Pop(PidPacket *pkt)
{
  if(pid_queue.count == 0) return 0;

  *pkt = pid_queue.packets[pid_queue.head];
  pid_queue.head = (pid_queue.head + 1) % PID_QUEUE_SIZE;
  pid_queue.count--;
  return 1;
}

extern UART_HandleTypeDef huart1;
extern uint32_t baro_height;
extern uint16_t voltage;
extern void Reset_Idle(void);
extern uint8_t fc_sleep_counting;

static struct{
  uint8_t PID1 :1;
  uint8_t PID2 :1;
  uint8_t PID3 :1;
  uint8_t PID4 :1;
  uint8_t PID5 :1;
  uint8_t PID6 :1;
  uint8_t CMD2_READ_PID:1;
}ANTO_Recived_flag;

uint16_t g_fc_idle_sleep_seconds = 60U;  // FC soft-sleep timeout in seconds, loaded from Flash at boot.
static volatile uint8_t g_pair_save_pending = 0U;  // Deferred Flash save flag for RF pair settings.
static volatile uint8_t g_pair_save_channel = 0U;
static volatile uint8_t g_pair_save_addr[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
static volatile uint8_t g_sleep_save_pending = 0U; // Deferred Flash save flag for FC sleep settings.
static volatile uint16_t g_sleep_save_seconds = 60U;
static volatile uint8_t g_fc_cal_pending = 0U;     // Deferred IMU calibration request from remote menu.
static uint8_t g_telemetry_round_robin = 0U;
static uint32_t g_last_status_tick = 0U;
static uint32_t g_last_sensor_tick = 0U;
static uint32_t g_last_power_tick = 0U;

static void FC_DefaultPair(uint8_t *channel, uint8_t *addr)
{
  *channel = 0U;
  addr[0] = 0x11U;
  addr[1] = 0x22U;
  addr[2] = 0x33U;
  addr[3] = 0x44U;
  addr[4] = 0x55U;
}

static void FC_ReadPairOrDefault(uint8_t *channel, uint8_t *addr)
{
  FC_DefaultPair(channel, addr);
  if(FLASH_ReadByte(PAIR_FLAG) == PAIR_MAGIC)
  {
    uint8_t saved_channel = FLASH_ReadByte(PAIR_ADDR);
    if(saved_channel <= 125U)
    {
      *channel = saved_channel;
      addr[0] = FLASH_ReadByte(PAIR_ADDR + 1U);
      addr[1] = FLASH_ReadByte(PAIR_ADDR + 2U);
      addr[2] = FLASH_ReadByte(PAIR_ADDR + 3U);
      addr[3] = FLASH_ReadByte(PAIR_ADDR + 4U);
      addr[4] = FLASH_ReadByte(PAIR_ADDR + 5U);
    }
  }
}

static void FC_SavePairAndSleep(uint8_t channel, const uint8_t *addr, uint16_t sleep_seconds)
{
  if(channel > 125U)
  {
    channel = 125U;
  }
  if(sleep_seconds > 600U)
  {
    sleep_seconds = 600U;
  }

  FLASH_EraseSectorByAddr(PAIR_ADDR);
  FLASH_WriteByte(PAIR_ADDR, channel);
  FLASH_WriteByte(PAIR_ADDR + 1U, addr[0]);
  FLASH_WriteByte(PAIR_ADDR + 2U, addr[1]);
  FLASH_WriteByte(PAIR_ADDR + 3U, addr[2]);
  FLASH_WriteByte(PAIR_ADDR + 4U, addr[3]);
  FLASH_WriteByte(PAIR_ADDR + 5U, addr[4]);
  FLASH_WriteByte(PAIR_FLAG, PAIR_MAGIC);
  FLASH_WriteByte(PAIR_BUILD_ADDR, (uint8_t)(PAIR_BUILD_ID & 0xFFU));
  FLASH_WriteByte(PAIR_BUILD_ADDR + 1U, (uint8_t)((PAIR_BUILD_ID >> 8) & 0xFFU));
  FLASH_WriteByte(FC_SLEEP_ADDR, (uint8_t)(sleep_seconds & 0xFFU));
  FLASH_WriteByte(FC_SLEEP_ADDR + 1U, (uint8_t)((sleep_seconds >> 8) & 0xFFU));
  FLASH_WriteByte(FC_SLEEP_MAGIC_ADDR, FC_SLEEP_MAGIC);
}

void FC_LoadSleepConfig(void)
{
  if(FLASH_ReadByte(FC_SLEEP_MAGIC_ADDR) == FC_SLEEP_MAGIC)
  {
    uint16_t saved_seconds = (uint16_t)FLASH_ReadByte(FC_SLEEP_ADDR) |
                             ((uint16_t)FLASH_ReadByte(FC_SLEEP_ADDR + 1U) << 8);
    if(saved_seconds <= 600U)
    {
      g_fc_idle_sleep_seconds = saved_seconds;
    }
  }
}

void FC_SaveSleepConfig(uint16_t seconds)
{
  uint8_t channel;
  uint8_t addr[5];

  FC_ReadPairOrDefault(&channel, addr);
  FC_SavePairAndSleep(channel, addr, seconds);
}
void ANO_ServicePairSave(void)
{
  uint8_t rf_addr[5];
  uint8_t rf_channel = 0U;
  uint8_t need_pair_save = 0U;
  uint8_t need_sleep_save = 0U;
  uint16_t sleep_seconds = g_fc_idle_sleep_seconds;

  if((g_pair_save_pending != 0U) || (g_sleep_save_pending != 0U))
  {
    __disable_irq();
    if(g_pair_save_pending != 0U)
    {
      rf_channel = g_pair_save_channel;
      rf_addr[0] = g_pair_save_addr[0];
      rf_addr[1] = g_pair_save_addr[1];
      rf_addr[2] = g_pair_save_addr[2];
      rf_addr[3] = g_pair_save_addr[3];
      rf_addr[4] = g_pair_save_addr[4];
      g_pair_save_pending = 0U;
      need_pair_save = 1U;
    }
    if(g_sleep_save_pending != 0U)
    {
      sleep_seconds = g_sleep_save_seconds;
      g_sleep_save_pending = 0U;
      need_sleep_save = 1U;
    }
    __enable_irq();
  }

  if((need_pair_save != 0U) || (need_sleep_save != 0U))
  {
    if(need_pair_save == 0U)
    {
      FC_ReadPairOrDefault(&rf_channel, rf_addr);
    }
    g_fc_idle_sleep_seconds = sleep_seconds;
    Reset_Idle();
    /* Defer Flash erase/write to the main loop, not the NRF IRQ path.
       This keeps the RF link alive while saving pair or sleep settings. */
    FC_SavePairAndSleep(rf_channel, rf_addr, sleep_seconds);
    if(need_pair_save != 0U)
    {
      NRF24L01_SetChannelAddress(rf_channel, rf_addr);
    }
    Reset_Idle();
  }

  if(g_fc_cal_pending != 0U)
  {
    uint8_t cal_type = g_fc_cal_pending;
    g_fc_cal_pending = 0U;

    if(cal_type == 1U)
    {
      // IMU calibration
      ALL_flag.unlock = 0;
      Reset_Idle();
      MPU6050_Calibrate();
      FLASH_WriteByte(CALIB_FLAG_ADDR, 0xAAU);
    }
    else if(cal_type == 2U)
    {
      // PID reset - reload from Flash or defaults
      // TODO: Implement PID reload from Flash
      Reset_Idle();
    }
    else if(cal_type == 3U)
    {
      // PID save - save current PID to Flash
      // TODO: Implement PID save to Flash
      Reset_Idle();
    }
  }
}
void ANO_Recive(uint8_t *pt)
{
  switch(pt[2])
  {
    case ANTO_CMD2:
      { 
        enum
        {
          READ_PID = 0x01,
          READ_MODE = 0x02,
          READ_ROUTE = 0x21,
          READ_VERSION = 0xA0,
          RETURN_DEFAULT_PID = 0xA1
         };  //CMD2;/

        switch(*(uint8_t*)&pt[4])
        {
          case READ_PID:
            ANTO_Recived_flag.CMD2_READ_PID = 1;
            break;
          case READ_MODE: 
            break;
          case READ_ROUTE: 
            break;          
          case READ_VERSION:  
            break;
          case RETURN_DEFAULT_PID:  
            break;          
          default: 
            break;          
        }
      
      }
      break;
    case ANTO_RCDATA: // 0x03: remote stick and auxiliary channel command
               Remote.thr  =((uint16_t)pt[4] <<8) | pt[5] ;  // channel 1: throttle
               Remote.yaw  =((uint16_t)pt[6] <<8) | pt[7] ;  // channel 2: yaw
               Remote.roll =((uint16_t)pt[8] <<8) | pt[9] ;  // channel 3: roll
               Remote.pitch=((uint16_t)pt[10]<<8) | pt[11];  // channel 4: pitch
               Remote.AUX1 =((uint16_t)pt[12]<<8) | pt[13];  // channel 5: aux key
               Remote.AUX2 =((uint16_t)pt[14]<<8) | pt[15];  // channel 6: aux key
               Remote.AUX3 =((uint16_t)pt[16]<<8) | pt[17];  // channel 7: aux key
               Remote.AUX4 =((uint16_t)pt[18]<<8) | pt[19];  // channel 8: aux key
               LIMIT(Remote.thr  ,1000,2000);
               LIMIT(Remote.yaw  ,1000,2000);
               LIMIT(Remote.roll ,1000,2000);
               LIMIT(Remote.pitch,1000,2000);
               LIMIT(Remote.AUX1 ,1000,2000);
               LIMIT(Remote.AUX2 ,1000,2000);
               LIMIT(Remote.AUX3 ,1000,2000);
               LIMIT(Remote.AUX4 ,1000,2000);    

               {
                 uint8_t rc_sleep_flags = (pt[3] >= 17U) ? pt[20] : 0U;
                 uint8_t rc_has_lock_flag = (uint8_t)((pt[3] >= 17U) ? 1U : 0U);
                 uint8_t rc_locked = (uint8_t)((rc_sleep_flags & 0x01U) ? 1U : 0U);
                 uint8_t rc_locked_idle = (uint8_t)(((rc_sleep_flags & 0x03U) == 0x03U) ? 1U : 0U);
                 if(rc_has_lock_flag != 0U)
                 {
                   ALL_flag.unlock = (rc_locked == 0U) ? 1U : 0U;
                 }
                  if(rc_locked_idle == 0U)
                  {
                    fc_sleep_counting = 0U;
                    Reset_Idle();
                  }
                  else
                  {
                    fc_sleep_counting = 1U;
                  }
                }

               {
                     const float roll_pitch_ratio = 0.02f;//0.04f
                     //const float yaw_ratio =  0.0015f;    
                 
                     pidPitch.desired =(1500-Remote.pitch)*roll_pitch_ratio;
                     pidRoll.desired = (1500-Remote.roll)*roll_pitch_ratio;
                     if(Remote.yaw>1820)
                     {
                       pidYaw.desired += 0.75f;  
                     }
                     else if(Remote.yaw <1180)
                     {
                       pidYaw.desired -= 0.75f;  
                     }            
               }
               if(pt[3] < 17U)
               {
                 remote_unlock(); // Legacy remotes still use the old throttle low-high-low unlock sequence.
               }
      break;
    case ANTO_GPSDATA: // 0x04: RF channel/address command from remote menu
      if(pt[3] >= 6U)
      {
        uint8_t rf_addr[5];
        uint8_t rf_channel = pt[4];
        if(rf_channel > 125U)
        {
          rf_channel = 125U;
        }
        rf_addr[0] = pt[5];
        rf_addr[1] = pt[6];
        rf_addr[2] = pt[7];
        rf_addr[3] = pt[8];
        rf_addr[4] = pt[9];
        Reset_Idle(); /* Pairing frame arrived: do not let standby interrupt RF switching. */
        NRF24L01_SetChannelAddress(rf_channel, rf_addr);
        g_pair_save_channel = rf_channel;
        g_pair_save_addr[0] = rf_addr[0];
        g_pair_save_addr[1] = rf_addr[1];
        g_pair_save_addr[2] = rf_addr[2];
        g_pair_save_addr[3] = rf_addr[3];
        g_pair_save_addr[4] = rf_addr[4];
        g_pair_save_pending = 1U; /* Save in main loop; do not erase Flash inside NRF IRQ. */
      }
      break;
    case ANTO_POWER: // 0x05: FC soft-sleep timeout from remote menu
      if(pt[3] >= 2U)
      {
        uint16_t requested_sleep_seconds = ((uint16_t)pt[4] << 8) | pt[5];
        if(requested_sleep_seconds > 600U)
        {
          requested_sleep_seconds = 600U;
        }
        g_fc_idle_sleep_seconds = requested_sleep_seconds;
        g_sleep_save_seconds = requested_sleep_seconds;
        g_sleep_save_pending = 1U;
      }
      break;
    case ANTO_MOTOR: // 0x06: remote menu requests IMU calibration
      if((pt[3] >= 1U) && (pt[4] == 0xA5U))
      {
        Reset_Idle();
        g_fc_cal_pending = 1U; /* Execute later in main loop, never inside NRF IRQ. */
      }
      break;
    case ANTO_RATE_PID:
      ANTO_Recived_flag.PID1 = 1;
      memcpy(RatePID,&pt[4],19);
      break;
    case ANTO_ANGLE_PID:
      memcpy(AnglePID,&pt[4],19);
      ANTO_Recived_flag.PID2 = 1;
      break;
    case ANTO_HEIGHT_PID:
      memcpy(HighPID,&pt[4],19);
      ANTO_Recived_flag.PID3 = 1;
      break;
    case ANTO_PID4:
      break;
    case ANTO_PID5:   
      break;
    case ANTO_PID6:
      break;
    case 0xF0:  // RESET_PID - reload PID from Flash or defaults
      g_fc_cal_pending = 2U;  // Use value 2 to indicate PID reset
      Reset_Idle();
      break;
    case 0xF1:  // SAVE_PID - save current PID to Flash
      g_fc_cal_pending = 3U;  // Use value 3 to indicate PID save
      Reset_Idle();
      break;
    case 0x01:

      break;
    default:
      break;      
  }
  return;
}
/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
static void ANTO_Send(const enum ANTO_SEND FUNCTION) // send data to ground station or NRF ACK payload
{
  uint8_t i;
  uint8_t len=2;
  int16_t Anto[12];
  int8_t *pt = (int8_t*)(Anto);
  PidObject *pidX=0;
  PidObject *pidY=0;
  PidObject *pidZ=0;

  switch(FUNCTION)
  {
    case ANTO_STATUS:  //0x01  send angle
      
         Anto[2] = (int16_t)(-Angle.roll*100);
         Anto[3] = (int16_t)(Angle.pitch*100);
         Anto[4] = (int16_t)(-Angle.yaw*100);
         Anto[5] = baro_height>>16;
         Anto[6] = baro_height;
         Anto[7] = (0x01<<8)|(ALL_flag.unlock);
         len = 12;
         break;
    case ANTO_MPU_MAGIC:
         memcpy(&Anto[2],(int8_t*)&MPU6050,sizeof(_st_Mpu));
         memcpy(&Anto[8],(int8_t*)&AK8975,sizeof(_st_Mag));
         len = 18;
         break;
    case ANTO_RATE_PID:      //0x10  PID1
         pidX = &pidRateX;
         pidY = &pidRateY;
         pidZ = &pidRateZ;
         goto send_pid;    
    case ANTO_ANGLE_PID:    //0x11  PID2
         pidX = &pidRoll;
         pidY = &pidPitch;
         pidZ = &pidYaw;
         goto send_pid;        
    case ANTO_HEIGHT_PID:   //0x12  PID3
         pidX = &pidHeightRate;
         pidY = &pidHeightHigh;
         goto send_pid;              
    case ANTO_PID4:           //PID4
    case ANTO_PID5:           //PID5
    case ANTO_PID6:           //PID6
send_pid:
         if(pidX!=NULL)
         {
           Anto[2] = (int16_t)(pidX->kp *1000);
           Anto[3] = (int16_t)(pidX->ki *1000);
           Anto[4] = (int16_t)(pidX->kd *1000);
         }
         if(pidY!=NULL)
         {
           Anto[5] = (int16_t)(pidY->kp *1000);
           Anto[6] = (int16_t)(pidY->ki *1000);
           Anto[7] = (int16_t)(pidY->kd *1000);
         }
         if(pidZ!=NULL)
         {
           Anto[8] = (int16_t)(pidZ->kp *1000);
           Anto[9] = (int16_t)(pidZ->ki *1000);
           Anto[10] = (int16_t)(pidZ->kd *1000);
         }
         len = 18;
         break;
    case ANTO_CHECK:
         if(ANTO_Recived_flag.PID1)
         { pt[5]=0x10;
           pt[4]=RatePID[18];
         }
         else if(ANTO_Recived_flag.PID2)
         { pt[5]=0x11; //
           pt[4]=AnglePID[18];
         }
         else if(ANTO_Recived_flag.PID3)
         { pt[5]=0x12; //
           pt[4]=HighPID[18];
         }
         else if(ANTO_Recived_flag.PID4)
           pt[5]=0x13; //
         else if(ANTO_Recived_flag.PID5)
           pt[5]=0x14; //
         else if(ANTO_Recived_flag.PID6)
           pt[5]=0x15; //
         len=2;
         break;
    case ANTO_RCDATA:  //0x03  send RC data

         break;
    case ANTO_POWER:  //0x05
         Anto[2] = (int16_t)voltage;
         Anto[3] = 0;
         len = 4;
        break;
    case ANTO_MOTOR:   //0x06  send motor

        break;  
    case ANTO_SENSER2: //0x07

        break;
    default:
        break;      
  }
  
  Anto[0] = 0xAAAA;
  Anto[1] =(FUNCTION<<8) | len;
  //Anto[1] =(len<<8) | FUNCTION;
  pt[len+4] = (int8_t)(0xAA+0xAA);
  for(i=2;i<len+4;i+=2)    //a swap with b;
  {
    pt[i] ^= pt[i+1];
    pt[i+1] ^= pt[i];
    pt[i] ^= pt[i+1];
    pt[len+4] += pt[i] + pt[i+1];
  }
  
  //send_char_array(&huart1,(uint8_t *)pt,len+5);
  //if(FUNCTION==ANTO_STATUS)
  //{ //NRF24L01_Write_Buf(0xa8, (uint8_t *)pt, len+5);
  
  /* Mirror the Fixed5 queue rule: only replace the NRF telemetry buffer when
     the previous frame has already been sent, otherwise keep the RF path stable. */
  if (nrf2401_tx_flag==1)
  { txbuf_pos=len+5;
    for(i=0;i<txbuf_pos;i++)
      nrf2401_txbuf[i]=pt[i];
    nrf2401_tx_flag=0;
  }

  //}
}
/***********************************************************************
 * polling  work.
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void ANTO_polling(void)
{
  volatile static uint8_t status = 1;
  uint32_t now = HAL_GetTick();
  uint32_t status_interval, sensor_interval, power_interval;

  /* Adaptive telemetry generation rate:
     - Locked: slow (500ms/1000ms/2000ms) - minimal keepalive
     - Unlocked: fast (50ms/100ms/200ms) - full ground station telemetry */
  if(ALL_flag.unlock == 0U)
  {
    status_interval = 500U;
    sensor_interval = 1000U;
    power_interval = 2000U;
  }
  else
  {
    status_interval = 50U;
    sensor_interval = 100U;
    power_interval = 200U;
  }

  switch(status)
  {
    case 1:
      /* Send telemetry in round-robin fashion. Check only the current frame type
         to ensure all three types get sent in order. */
      if(nrf2401_tx_flag==1)
      {
        // Priority 1: Process PID queue if not empty
        if(pid_queue.count > 0)
        {
          PidPacket pkt;
          if(PidQueue_Pop(&pkt))
          {
            ANTO_Send(pkt.func_id);
          }
          break; // Exit to allow next packet to be sent in next cycle
        }

        // Priority 2: Normal telemetry round-robin
        if(g_telemetry_round_robin == 0U)
        {
          if((now - g_last_status_tick) >= status_interval)
          {
            ANTO_Send(ANTO_STATUS);
            g_last_status_tick = now;
            g_telemetry_round_robin = 1U;
          }
        }
        else if(g_telemetry_round_robin == 1U)
        {
          if((now - g_last_sensor_tick) >= sensor_interval)
          {
            ANTO_Send(ANTO_MPU_MAGIC);
            g_last_sensor_tick = now;
            g_telemetry_round_robin = 2U;
          }
        }
        else
        {
          if((now - g_last_power_tick) >= power_interval)
          {
            ANTO_Send(ANTO_POWER);
            g_last_power_tick = now;
            g_telemetry_round_robin = 0U;
          }
        }
      }
      if(*(uint8_t*)&ANTO_Recived_flag != 0)
        status = 2;
      break;
    case 2:
      if(*(uint8_t*)&ANTO_Recived_flag == 0)
      {
        status = 1;
      }
  
      if(ANTO_Recived_flag.CMD2_READ_PID)
      {
        // Queue all PID packets for sequential transmission
        PidQueue_Push(ANTO_RATE_PID, NULL, 0);
        PidQueue_Push(ANTO_ANGLE_PID, NULL, 0);
        PidQueue_Push(ANTO_HEIGHT_PID, NULL, 0);
        ANTO_Recived_flag.CMD2_READ_PID = 0;
      }
      
      if(*(uint8_t*)&ANTO_Recived_flag & 0x3f)
      {
          PidObject *pidX=0;
          PidObject *pidY=0;
          PidObject *pidZ=0;
          uint8_t *P;

          // Queue ACK response instead of sending immediately
          PidQueue_Push(ANTO_CHECK, NULL, 0);

          if(ANTO_Recived_flag.PID1)
          {
             pidX = &pidRateX;
             pidY = &pidRateY;
             pidZ = &pidRateZ;
             P = RatePID;
             ANTO_Recived_flag.PID1 = 0;
          }
          else if(ANTO_Recived_flag.PID2)
          {
             pidX = &pidRoll;
             pidY = &pidPitch;
             pidZ = &pidYaw;
             P = AnglePID;  
             ANTO_Recived_flag.PID2 = 0;                             
          }
          else if(ANTO_Recived_flag.PID3)
          {
             pidX = &pidHeightRate;
             pidY = &pidHeightHigh;
             P = HighPID;  
             ANTO_Recived_flag.PID3 = 0;                 
          }
          else
          {
            break;
          }
          {
              union {
                uint16_t _16;
                uint8_t _u8[2];
              }data;
              
              if(pidX!=NULL)
              {
                data._u8[1] = P[0]; 
                data._u8[0] = P[1];
                pidX->kp =  data._16 /1000.0f;
                data._u8[1] = P[2]; 
                data._u8[0] = P[3];
                pidX->ki =  data._16 /1000.0f;
                data._u8[1] = P[4]; 
                data._u8[0] = P[5];
                pidX->kd =  data._16 /1000.0f;                
              }
              if(pidY!=NULL)
              {
                data._u8[1] = P[6]; 
                data._u8[0] = P[7];
                pidY->kp =  data._16 /1000.0f;
                data._u8[1] = P[8]; 
                data._u8[0] = P[9];
                pidY->ki =  data._16 /1000.0f;
                data._u8[1] = P[10]; 
                data._u8[0] = P[11];
                pidY->kd =  data._16 /1000.0f;    
              }
              if(pidZ!=NULL)
              {
                data._u8[1] = P[12]; 
                data._u8[0] = P[13];
                pidZ->kp =  data._16 /1000.0f;
                data._u8[1] = P[14]; 
                data._u8[0] = P[15];
                pidZ->ki =  data._16 /1000.0f;
                data._u8[1] = P[16]; 
                data._u8[0] = P[17];
                pidZ->kd =  data._16 /1000.0f;    
              }        
          }        
      }
      break;
    default:
      break;
  }

}

/************************END OF FILE********************/
