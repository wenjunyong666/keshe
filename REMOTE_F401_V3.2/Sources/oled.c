#include "oled.h"
#include "oledfont.h"
#include "main.h"
#include <string.h>
//几个变量声明
extern GB_16 HZ[];
extern I2C_HandleTypeDef hi2c1;
static uint8_t oled_zero_page[128] = {0};
//初始化命令
uint8_t CMD_Data[]={
0xAE, 0x00, 0x10, 0x40, 0xB0, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F,
0xC8, 0xD3, 0x00, 0xD5, 0x80, 0xD8, 0x05, 0xD9, 0xF1, 0xDA, 0x12,
0xD8, 0x30, 0x8D, 0x14, 0xAF};
void WriteCmd()
{
  uint8_t i = 0;
  for(i=0; i<27; i++)
  {
    HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,CMD_Data+i,1,0x100);
  }
}
//向设备写控制命令
void OLED_WR_CMD(uint8_t cmd)
{
  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x00,I2C_MEMADD_SIZE_8BIT,&cmd,1,0x100);
}
//向设备写数据
void OLED_WR_DATA(uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,0x100);
}
//初始化oled屏幕
void OLED_Init(void)
{   
  HAL_Delay(200);
  WriteCmd();
}
//清屏size12 size16要清两行，其他函数有类似情况
void OLED_Clear()
{
  uint8_t i;        
  for(i=0;i<8;i++)  
  {  
    OLED_WR_CMD(0xb0+i);
    OLED_WR_CMD (0x00); 
    OLED_WR_CMD (0x10); 
    HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,oled_zero_page,128,0x100);
  } 
}
//清行
void OLED_Clearrow(uint8_t i)
{
  OLED_WR_CMD(0xb0+i);
  OLED_WR_CMD(0x00); 
  OLED_WR_CMD(0x10); 
  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,oled_zero_page,128,0x100);
}
//开启OLED显示    
void OLED_Display_On(void)
{
  OLED_WR_CMD(0X8D);  //SET DCDC命令
  OLED_WR_CMD(0X14);  //DCDC ON
  OLED_WR_CMD(0XAF);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
  OLED_WR_CMD(0X8D);  //SET DCDC命令
  OLED_WR_CMD(0X10);  //DCDC OFF
  OLED_WR_CMD(0XAE);  //DISPLAY OFF
}              
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{   
  OLED_WR_CMD(0xb0+y);
  OLED_WR_CMD(((x&0xf0)>>4)|0x10);
  OLED_WR_CMD(x&0x0f);
} 
 
void OLED_On(void)  
{  
  uint8_t i,n;        
  for(i=0;i<8;i++)  
  {  
    OLED_WR_CMD(0xb0+i);    //设置页地址（0~7）
    OLED_WR_CMD(0x00);      //设置显示位置—列低地址
    OLED_WR_CMD(0x10);      //设置显示位置—列高地址   
    for(n=0;n<128;n++)
      OLED_WR_DATA(1); 
  } //更新显示
}
unsigned int oled_pow(uint8_t m,uint8_t n)
{
  unsigned int result=1;   
  while(n--)result*=m;    
  return result;
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示         
//size:选择字体 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size)
{        
  unsigned char c=0,i=0;  
  c=chr-' ';//得到偏移后的值      
  if(x>128-1){x=0;y=y+2;}
  if(Char_Size ==16)
  {
    OLED_Set_Pos(x,y);  
    for(i=0;i<8;i++)
    OLED_WR_DATA(F8x16[c*16+i]);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<8;i++)
    OLED_WR_DATA(F8x16[c*16+i+8]);
  }
  else 
  {  
    OLED_Set_Pos(x,y);
    for(i=0;i<6;i++)
    OLED_WR_DATA(F6x8[c][i]);
  }
}
 //显示2个数字
//x,y :起点坐标   
//len :数字的位数
//size:字体大小
//mode:模式  0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(uint8_t x,uint8_t y,int num,uint8_t len,uint8_t size2)
{         
  uint8_t str[7],i=0;
  str[6]=0;
  if(num<0)
  { num=-num;
    str[0]='-';
  }
  else
    str[0]=' ';
  
  do
  { str[5-i]=num%10+'0';
    num=num/10;
    i++;
  } while(num!=0);
  
  if(str[0]=='-')
    str[5-i++]='-';
  
  if(i<len)
  for(;i<len;i++)
  { str[5-i]=' ';
  }
  else if (i>len)  //超出len指定的长度时，低位不显示
  { str[9-i]=0;
  }    
  OLED_ShowString(x,y,str+6-i,size2);  
} 
//显示一个字符号串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size)
{
  unsigned char j=0;
  while (chr[j]!='\0')
  { OLED_ShowChar(x,y,chr[j],Char_Size);
    x+=8;
    if(x>120){x=0;y+=2;}
      j++;
  }
}

//显示汉字，hz为汉字的编码,可以是汉字与英文及数字的任意组合
void OLED_DrawPageText(uint8_t page, uint8_t *chr)
{
  uint8_t line[128];
  uint8_t ch;
  uint8_t i;
  uint8_t pos = 0;

  /* [REQ-3][OLED-ROOTFIX] Build and write a full page at once so menu updates
   * do not flicker from per-character I2C writes and do not leave right-edge
   * garbage pixels behind.
   */
  memset(line, 0, sizeof(line));
  while ((chr != 0) && (*chr != '\0') && (pos < 16U))
  {
    ch = *chr;
    if ((ch < ' ') || (ch > '~'))
    {
      ch = ' ';
    }
    for (i = 0; i < 6U; i++)
    {
      line[pos * 8U + i] = F6x8[ch - ' '][i];
    }
    line[pos * 8U + 6U] = 0x00U;
    line[pos * 8U + 7U] = 0x00U;
    pos++;
    chr++;
  }

  OLED_WR_CMD(0xb0 + page);
  OLED_WR_CMD(0x00);
  OLED_WR_CMD(0x10);
  HAL_I2C_Mem_Write(&hi2c1 ,0x78,0x40,I2C_MEMADD_SIZE_8BIT,line,128,0x100);
}

void OLED_ShowHZ(uint8_t x,uint8_t y,char hz[])
{                
  uint8_t t,no,p=0;
  while (hz[p]!=0)
  { if (((hz[p]&0x80)==0x80) && ((hz[p+1]&0x80)==0x80) )     //是汉字，一个汉字占两个字节，且每个字节的最高位都是1
    { for(t=0;t<lenHZ;t++)  //字库HZ中查找汉字hz
        if ((HZ[t].Index[0]==hz[p]) && (HZ[t].Index[1]==hz[p+1])) 
          break;
      if (t==lenHZ)  //字库中没有找到指定的汉字，则显示空白
      { 
        OLED_Set_Pos(x,y);  
        for(t=0;t<16;t++)
          OLED_WR_DATA(0x00);
        OLED_Set_Pos(x,y+1);  
        for(t=0;t<16;t++)
          OLED_WR_DATA(0x00);
      }
      else  //汉字库中找到指定的汉字
      {
        no=t;
        OLED_Set_Pos(x,y);  
        for(t=0;t<16;t++)
          OLED_WR_DATA(HZ[no].Msk[t]);
        OLED_Set_Pos(x,y+1);  
        for(t=0;t<16;t++)
          OLED_WR_DATA(HZ[no].Msk[t+16]);
      }
      x=x+16;
      p=p+2;
    }
    else  //是英文字母或数字等
    {
      OLED_ShowChar(x,y,hz[p],16);
      x=x+8;
      p++;
    }
    if (x>=128)  //达到指定行最右边，转下一行显示
    { x=x-128;
      y=y+2;
    }
  }
}

//显示小数，整数部分最多5位（加负号是6位），小数部分显示2位，len最小为4，最大为9
void OLED_ShowFloat(uint8_t x,uint8_t y,float num,uint8_t len)
{ int16_t a;
  uint8_t b;
  a=(int)num;
  if(num<0)
    b=(uint8_t)((a-num)*100+0.5);
  else
    b=(uint8_t)((num-a)*100+0.5);
  OLED_ShowNum(x,y,a,len-3,16);
  OLED_ShowChar(x+(len-3)*8,y,'.',16);
  if (b<10)
  { OLED_ShowChar(x+(len-2)*8,y,'0',16);
    OLED_ShowNum(x+(len-1)*8,y,b,1,16);
  }
  else  
    OLED_ShowNum(x+(len-2)*8,y,b,2,16);
}

