#ifndef __ALL_DEFINE_H__
#define __ALL_DEFINE_H__
//#include "stm32f10x.h"
#include "main.h"
// Calibration data exported by the calibration module.
extern uint8_t  Calib_Flag;        // calibration flag (0=invalid, 1=valid)
extern int16_t  Accel_Offset[3];  // accelerometer offset
extern int16_t  Gyro_Offset[3];   // gyroscope offset

// Pairing and sleep Flash storage.
#define PAIR_ADDR    0x0803F000  // F401CC pair config page
#define PAIR_FLAG    0x0803F010  // F401CC pair flag byte
#define PAIR_MAGIC   0xAA
#define PAIR_BUILD_ADDR 0x0803F012
#define PAIR_BUILD_ID   ((uint16_t)((((uint16_t)__TIME__[0]) << 8) ^ (((uint16_t)__TIME__[1]) << 4) ^ (((uint16_t)__TIME__[3]) << 12) ^ (((uint16_t)__TIME__[4]) << 2) ^ ((uint16_t)__TIME__[6]) ^ (((uint16_t)__TIME__[7]) << 1)))      // build id for flash reset on rebuild
#define FC_SLEEP_ADDR       0x0803F020
#define FC_SLEEP_MAGIC_ADDR 0x0803F022
#define FC_SLEEP_MAGIC      0x5C

// Pairing data exported for legacy modules.
extern uint8_t  Pair_Flag;
extern uint8_t  NRF24L01_Addr[5];
// Flash addresses for calibration data. Keep them in a valid F401 sector.
#define CALIB_FLAG_ADDR  0x0801F000
#define CALIB_DATA_ADDR  0x0801F004
#include "ALL_DATA.h"
#include "INIT.h"
#include "stdio.h"
//#include "sys.h"
//#include "I2C.h"
//#include "SPI.h"
#include "NRF24L01.h"
#include "UART.h"
//#include  "TIM.h"
#include "LED.h"
#include "mpu6050.h"
#include "SPL06_001.h"
#include "GL9306.h"
#include "imu.h"
#include "ANO_DT.h"
#include "Remote.h"
#include "control.h"
#include "myMath.h"
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1



#endif

