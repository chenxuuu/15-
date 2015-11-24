/*
 * File:		nokia.h
 * Purpose:		Serial Input/Output routines
 *
 */

#ifndef _OELD_H
#define _OELD_H

#include  "MK60_gpio.h" 
/********************************************************************/
/*-----------------------------------------------------------------------
LCD_init          : OLED初始化

编写日期          ：2012-11-01
最后修改日期      ：2012-11-01
-----------------------------------------------------------------------*/
 extern uint8 lanzhou96x64[768];
 void OLED_Init(void);
 void OLED_CLS(void);
 void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_Print(uint8 x, uint8 y, uint8 ch[]);
 void OLED_PutPixel(uint8 x,uint8 y);
 void OLED_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);
 void OLED_Set_Pos(uint8 x, uint8 y);
 void OLED_WrDat(uint8 data);
 void Draw_LibLogo(void);
 void Draw_Landzo(void);
 void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]);
 void OLED_Fill(uint8 dat);
 void Dly_ms(uint16 ms);
 void Display_number7 (uint8 x,uint8 y,int16 i);
//void OLED_P6x8char(unsigned char x,unsigned char y,unsigned char c);
//void OLED_P6x8fig3(unsigned char x,unsigned char y,unsigned short int num);
//void OLED_P6x8fig4(unsigned char x,unsigned char y,unsigned short int num);

void Display_integer3 (uint8 x,uint8 y,int16 i);
void Display_number (uint8 x,uint8 y,int16 i);
void DisplayFloat3(uint8 x,uint8 y,int32 i);
 
//void Displayvoid (uint8 x,uint8 y,int16 i);
//void DisplayFloat(uint8 x,uint8 y,int16 i);
/********************************************************************/

#endif
