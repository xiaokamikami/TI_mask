/***********************************************
*            OLED用到的头文件、宏定义和函数声明
***********************************************/

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


#ifndef __OLED_H
#define __OLED_H
//#include "sys.h"
//#include "stdlib.h"
#define  u8 unsigned char 
#define  u32 unsigned int
#define OLED_CMD  0 //写命令
#define OLED_DATA 1 //写数据

//位操作定义
#define     Set_Bit(val, bitn)      (val |= (/*1 <<*/(bitn)))
#define     Clr_Bit(val, bitn)      (val &= ~(/*1<<*/(bitn)))
#define     Get_Bit(val, bitn)      (val & (1<<(bitn)) )
//----------------------------------------------------------------------------------
//OLED SSH1106 IIC  时钟D0    P4.0
#define     OLED_SSH1106_SCLK_PIN_NUM       (BIT0)

#define     OLED_SSH1106_SCLK_IO_INIT       (Set_Bit(P4DIR,OLED_SSH1106_SCLK_PIN_NUM))
#define     OLED_SCLK_Set()                 (Set_Bit(P4OUT,OLED_SSH1106_SCLK_PIN_NUM))
#define     OLED_SCLK_Clr()                 (Clr_Bit(P4OUT,OLED_SSH1106_SCLK_PIN_NUM))

//----------------------------------------------------------------------------------
//OLED SSH1106 IIC 数据D1     P3.2
#define     OLED_SSH1106_SDIN_PIN_NUM       (BIT2)

#define     OLED_SSH1106_SDIN_IO_INIT       (Set_Bit(P3DIR,OLED_SSH1106_SDIN_PIN_NUM))
#define     OLED_SDIN_Set()                 (Set_Bit(P3OUT,OLED_SSH1106_SDIN_PIN_NUM))
#define     OLED_SDIN_Clr()                 (Clr_Bit(P3OUT,OLED_SSH1106_SDIN_PIN_NUM))



#define SIZE 16
#define XLevelL     0x02
#define XLevelH     0x10
#define Max_Column  128
#define Max_Row     64
#define Brightness  0xFF
#define X_WIDTH     128
#define Y_WIDTH     64

void delay_ms(unsigned int ms);

//OLED控制用函数
void OLED_Clear_Line(uint8_t LINE);
void OLED_Pin_Init();
void OLED_WR_Byte(u8 dat,u8 cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2);
void OLED_ShowString(u8 x,u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
#endif
