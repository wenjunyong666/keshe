#ifndef _REMOTE_H
#define _REMOTE_H

#include "main.h"


typedef volatile struct 
{
  uint16_t thr;
  uint16_t yaw;  
  uint16_t roll;
  uint16_t pitch;
  uint16_t AUX1;
  uint16_t AUX2;
  uint16_t AUX3;
  uint16_t AUX4;
  uint16_t AUX5;
  uint16_t AUX6;
  uint16_t AUX7;
}_st_Remote;

typedef volatile struct //校准数据
{
  int16_t flag;    //校准标志位
  int16_t thr;  
  int16_t yaw;  
  int16_t roll;
  int16_t pitch;  
}_st_Offset ;

extern _st_Remote Remote;
extern _st_Offset offset;
/* 原工程接口：开机初始化遥控器参数、校准参数、无线参数。 */
extern void  RC_INIT(void);
/* 原工程周期接口：保持 10ms 调度，内部会完成按键/菜单/发包/休眠处理。 */
extern void RC_Analy(void);

/* 新增接口：OLED 首页和菜单统一刷新入口，由 main.c 主循环调用。 */
extern void Remote_UI_Render(void);
/* 新增接口：处理飞控回传数据，目前主要用于首页飞控电压/链路状态显示。 */
extern void Remote_OnTelemetry(uint8_t *data, uint8_t len);
/* 新增接口：供外部查询当前是否仍处于锁定状态。 */
extern uint8_t Remote_IsLocked(void);
/* 新增接口：在无飞控联调时，将真实发给飞控的 NRF 控制帧镜像到 USB。 */
extern void Remote_MirrorTxFrame_ToUsb(uint8_t *frame, uint8_t len);
#endif


