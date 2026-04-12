#ifndef _DATA_TRANSFER_H
#define  _DATA_TRANSFER_H


#include "main.h"


enum ANTO_SEND{         //飞控发的命令类型
  ANTO_VER       = 0x00,
  ANTO_STATUS    = 0x01,
  ANTO_MPU_MAGIC = 0X02,
  ANTO_RCDATA    = 0x03,
  ANTO_GPSDATA   = 0x04,
  ANTO_POWER     = 0x05,
  ANTO_MOTOR     = 0x06,
  ANTO_SENSER2   = 0x07,  
  ANTO_RESERD1   = 0x08,
  ANTO_RESERD2   = 0x09,  
  ANTO_FLY_MODE  = 0x0A,  
  ANTO_RATE_PID  = 0x10,  
  ANTO_ANGLE_PID = 0x11,
  ANTO_HEIGHT_PID= 0x12,    
  ANTO_PID4      = 0x13,  
  ANTO_PID5      = 0x14,  
  ANTO_PID6      = 0x15,   
  ANTO_CHECK     = 0xEF
};

enum ANTO_RECV{         //飞控收的命令类型,其他和飞控发的命令相同的未列出
  ANTO_CMD1      = 0x01,
  ANTO_CMD2      = 0X02,
};
extern void ANO_Recive(uint8_t *pt);
extern void ANO_ServicePairSave(void);
extern void FC_LoadSleepConfig(void);
extern void FC_SaveSleepConfig(uint16_t seconds);
extern void ANTO_polling(void);
extern uint16_t g_fc_idle_sleep_seconds;

#endif







