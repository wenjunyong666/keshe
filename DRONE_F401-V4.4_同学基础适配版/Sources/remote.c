//========================================================================
//  爱好者电子工作室-淘宝 https://devotee.taobao.com/
//  STM32四轴爱好者QQ群: 810149456
//  作者：小刘
//  电话:13728698082
//  邮箱:1042763631@qq.com
//  日期：2018.05.17
//  版本：V1.0
//========================================================================
//套件购买地址：https://devotee.taobao.com/
//                 爱好者电子工作室
//特此声明：
//
//         此程序只能用作学习，如用商业用途。必追究责任！
//          
//
//
#include "ALL_DATA.h"
#include "NRF24L01.h"
#include "control.h"
#include <math.h>
#include "LED.h"
#include "Remote.h"
#include "ANO_DT.h"

//#define SUCCESS 0
//#undef FAILED
//#define FAILED  1
/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/  
uint16_t nrf_cnt=0;
uint8_t RC_rxData[32];
extern UART_HandleTypeDef huart6;
extern void Reset_Idle(void);

void remote_unlock(void);  
void RC_Analy(void)  
{ uint8_t rxlen;
  //Receive  and check RC data 
  rxlen=NRF24L01_RxPacket(RC_rxData);
  if(rxlen>0)
  { uint8_t i;
    uint8_t CheckSum=0;
    uint8_t CheckSumNoHeader=0;
    //send_char_array(&huart6,RC_rxData,rxlen);
    nrf_cnt = 0;
    for(i=0;i<rxlen-1;i++)
    {
      CheckSum +=  RC_rxData[i];
      if(i >= 2U)
      {
        CheckSumNoHeader += RC_rxData[i];
      }
    }
    if((RC_rxData[rxlen-1] == CheckSum) || (RC_rxData[rxlen-1] == CheckSumNoHeader))
    {
      /* Accept both direct RC packets (AA AF) and station packets relayed by RC (AA AA). */
      if(RC_rxData[0]==0xAA && ((RC_rxData[1]==0xAF) || (RC_rxData[1]==0xAA)))
      {
        ANO_Recive(RC_rxData);
      }
    }
  }
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] if
 * @return     
 ******************************************************************************************/  
void remote_unlock(void)
{
  static uint8_t status=WAITING_1;
  static uint16_t cnt=0;

  if(Remote.thr<1200 && Remote.yaw<1200)                         //油门遥杆左下角锁定
  {
    status = EXIT_255;
  }
  
  switch(status)
  {
    case WAITING_1://等待解锁
      if(Remote.thr<1150)           //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
      {       
        status = WAITING_2;
      }    
      break;
    case WAITING_2:
      if(Remote.thr>1800)          
      {    
        status = WAITING_3;
      }      
      break;
    case WAITING_3:
      if(Remote.thr<1150)          
      {       
        ALL_flag.unlock = 1;     //解锁前准备
        status = PROCESS_31;
        LED.status = AlwaysOn;                  
      }      
      break;      
    case PROCESS_31:  //进入解锁状态
      if(Remote.thr<1020)
      {
        if((g_fc_idle_sleep_seconds != 0U) && (cnt++ > 1500))    // 油门遥杆处于最低30S自动上锁
        { cnt=0;               
          status = EXIT_255;                
        }
        else if(g_fc_idle_sleep_seconds == 0U)
        {
          cnt = 0;
        }
      }
      else if(!ALL_flag.unlock)                           //Other conditions lock 
      {
        status = EXIT_255;        
      }
      else          
        cnt = 0;
      break;
    case EXIT_255: //进入锁定
      LED.status = AllFlashLight;                                   //exit
      cnt = 0;
      LED.FlashTime = 100; //100*3ms    
      ALL_flag.unlock = 0;
      status = WAITING_1;
      break;
    default:
      status = EXIT_255;
      break;
  }
}
/***********************END OF FILE*************************************/







