#ifndef __OLED_H
#define __OLED_H

#include "sys.h"
/********定义端口********/
#define OLED_DC_Clr() PAout(15)=0    //命令标志
#define OLED_DC_Set() PAout(15)=1    //数据标志

#define OLED_SCLK_Clr()  PBout(5)=0  //SCL
#define OLED_SCLK_Set()  PBout(5)=1   //SCL

#define OLED_SDIN_Clr()  PBout(4)=0   //SDA
#define OLED_SDIN_Set()  PBout(4)=1   //SDA

#define OLED_RES_Clr() PBout(3)=0
#define OLED_RES_Set() PBout(3)=1

/********定义常量********/
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#define WIDTH   128
#define HEIGHT  64	

/********声明变量********/

/********声明函数********/
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void OLED_DrawString(u8 x,u8 y,const u8 *p,u8 size);
void OLED_Showbmp(u8 x,u8 y,u8 width, u8 height, const u8 *pic, u8 mode);
void GUI_ShowCHinese(u8 x, u8 y, u8 hsize, u8 *str, u8 mode);
void StartingDisplay(void);

void Oled_Show(void);
void Get_GRAM (void);

#endif
