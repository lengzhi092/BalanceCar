/**
    ****************************************************************************
    *@file       oled.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      OLED显示屏控制
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "oled.h"
#include "oledfont.h"
#include "image.h"
u8 OLED_GRAM[128][8];

/**
    ****************************************************************************
    *@brief      向OLED写入一个字节。
    *@param      dat:要写入的数据/命令;dc:数据/命令标志,0表示命令;1表示数据
    *@retval     无
    ****************************************************************************
    */
void OLED_WR_Byte(u8 dat,u8 dc)
{
	u8 i;
	if(dc)
	  OLED_DC_Set();    //DC端口数据模式
	else
	  OLED_DC_Clr();    //DC端口命令模式
	for(i=0;i<8;i++)
	{
		OLED_SCLK_Clr();  //SCL
		if(dat&0x80)
		   OLED_SDIN_Set();  //SDA
		else
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;
	}
	OLED_DC_Set();
}
/**
    ****************************************************************************
    *@brief      刷新OLED屏幕
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void OLED_Refresh_Gram()
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示起始列地址（低四位）
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示起始列地址（高四位）
		for(n=0;n<128;n++)
        OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

/**
    ****************************************************************************
    *@brief      清屏
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void OLED_Clear()
{
	u8 i,n;
	for(i=0;i<8;i++)
        for(n=0;n<128;n++)
            OLED_GRAM[n][i]=0X00;  
    OLED_Refresh_Gram();  //更新显示
}
/**
    ****************************************************************************
    *@brief      初始化OLED
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void OLED_Init()
{
	OLED_RES_Clr();    //复位
	HAL_Delay(100);
	OLED_RES_Set();

	OLED_WR_Byte(0xAE,OLED_CMD); //关闭显示
	OLED_WR_Byte(0xD5,OLED_CMD); //设置时钟分频因子,震荡频率
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],分频因子;[7:4],震荡频率
	OLED_WR_Byte(0xA8,OLED_CMD); //设置驱动路数
	OLED_WR_Byte(0X3F,OLED_CMD); //默认0X3F(1/64)
	OLED_WR_Byte(0xD3,OLED_CMD); //设置显示偏移
	OLED_WR_Byte(0X00,OLED_CMD); //默认为0
	OLED_WR_Byte(0x40,OLED_CMD); //设置显示开始行 [5:0],行数
	OLED_WR_Byte(0x8D,OLED_CMD); //电荷泵设置
	OLED_WR_Byte(0x14,OLED_CMD); //bit2，开启/关闭
	OLED_WR_Byte(0x20,OLED_CMD); //设置内存地址模式
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	OLED_WR_Byte(0xA1,OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WR_Byte(0xDA,OLED_CMD); //设置COM硬件引脚配置
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]配置	
	OLED_WR_Byte(0x81,OLED_CMD); //对比度设置
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;默认0X7F (亮度设置,越大越亮)
	OLED_WR_Byte(0xD9,OLED_CMD); //设置预充电周期
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //设置VCOMH 电压倍率
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
	OLED_WR_Byte(0xA4,OLED_CMD); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WR_Byte(0xA6,OLED_CMD); //设置显示方式;bit0:1,反相显示;0,正常显示
	OLED_WR_Byte(0xAF,OLED_CMD); //开启显示
	OLED_Clear();
}  
/**
    ****************************************************************************
    *@brief      向图像寄存器GRAM中写入一个点，刷新后显示
    *@param      x,y:起点坐标;mode:0反白显示,1正常显示
    *@retval     无
    ****************************************************************************
    */
void OLED_DrawPoint(u8 x,u8 y,u8 mode)
{
	u8 pos,temp=0;
	if(x>127||y>63)
        return;//超出范围了
	pos = 7-y/8;  //页码
	temp = 1<<(7-y%8);   //页中的位置
	if(mode)
        OLED_GRAM[x][pos] |= temp;
	else
        OLED_GRAM[x][pos] &= ~temp;
}
/**
    ****************************************************************************
    *@brief      在指定位置写入一个字符（95个ascll中的内容），刷新后显示
    *@param      x,y :起点坐标; chr:字符; size:字号; mode:0反白显示,1正常显示
    *@retval     无
    ****************************************************************************
    */
void OLED_DrawChar(u8 x, u8 y, u8 chr, u8 size, u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
    u8 csize=(size/8+((size%8)?1:0))*(size/2);  //得到字体一个字符对应点阵集所占的字节数
	chr=chr-' ';	//得到偏移后的值
    for(t=0;t<csize;t++)
    {
			if(size==12)  temp=asc2_1206[chr][t];   //调用1206字体
			else if(size==16)temp=asc2_1608[chr][t];    //调用1608字体
            else if(size==24)temp=asc2_2412[chr][t];	//调用2412字体
            else return;	//没有的字库	                          
            for(t1=0;t1<8;t1++) //写入一个字节
			{
				if(temp&0x80)  OLED_DrawPoint(x,y,mode);
				else           OLED_DrawPoint(x,y,!mode);
				temp <<= 1;
				y++;
				if((y-y0) == size)  //完成点阵的一列
				{
					y=y0;
					x++;    //显示点阵下一列
					break;
				}
			}
    }
}
/**
    ****************************************************************************
    *@brief      m的n次方，供下面的OLED_ShowNumber函数调用
    *@param      m：底数，n：次方数
    *@retval     无
    ****************************************************************************
    */
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}
/**
    ****************************************************************************
    *@brief      显示n个数字
    *@param      x,y:起点坐标; num:数值;len:数字的位数; size:字体大小
    *@retval     无
    ****************************************************************************
    */  
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp,enshow=0;
    for(t=0;t<len;t++)
	{
        temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_DrawChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_DrawChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
}
/**
    ****************************************************************************
    *@brief      在指定位置写入字符串（95个ascll中的内容），刷新后显示
    *@param      x,y:起点坐标，*p:字符串起始地址,sise:字号
    *@retval     无
    ****************************************************************************
    */

void OLED_DrawString(u8 x,u8 y,const u8 *p,u8 size)
{
    while(*p != '\0')   //\0（空字符）是字符串的结束码
    {
        if(x>(128-(size/2)))    //剩余位置宽度不足容纳一个字符点阵
            {x=0;y+=size;}  //从下面另起一行
        if(y>(64-size))    //屏写满了
            {y=x=0;OLED_Clear();}
        OLED_DrawChar(x,y,*p,size,1);
        x+=size/2;
        p++;
    }
}
/**
    ****************************************************************************
    *@brief      写入一个16像素的汉字
    *@param      x,y :起点坐标; s：中文字符首地址; mode:0反白显示,1正常显示
    *@retval     无
    ****************************************************************************
    */
void GUI_ShowFont16(u8 x,u8 y,u8 *s,u8 mode)
{
	u8 i,j,k,tmp;
	u16 num;
	num = sizeof(cfont16)/sizeof(typFNT_GB16);
    for(i=0;i<num;i++)
	{
		if((cfont16[i].Index[0]==*s)&&(cfont16[i].Index[1]==*(s+1)))
		{
			for(j=0;j<32;j++)
			{
				if(mode)
					tmp = cfont16[i].Msk[j];
				else
					tmp = ~(cfont16[i].Msk[j]);
				for(k=0;k<8;k++)
				{
					if(tmp&(0x80>>k))
						OLED_DrawPoint(x+(j%2)*8+k, y+j/2,1);
					else
						OLED_DrawPoint(x+(j%2)*8+k, y+j/2,0);
				}
			}
			break;
		}
	}
}
/**
    ****************************************************************************
    *@brief      写入一个24像素的汉字
    *@param      x,y :起点坐标; s：中文字符首地址; mode:0反白显示,1正常显示
    *@retval     无
    ****************************************************************************
    */
void GUI_ShowFont24(u8 x,u8 y,u8 *s,u8 mode)
{
	u8 i,j,k,tmp;
	u16 num;
	num = sizeof(cfont24)/sizeof(typFNT_GB24);
    for(i=0;i<num;i++)
	{
		if((cfont24[i].Index[0]==*s)&&(cfont24[i].Index[1]==*(s+1)))
		{
			for(j=0;j<72;j++)
			{
				if(mode)
					tmp = cfont24[i].Msk[j];
				else
					tmp = ~(cfont24[i].Msk[j]);
				for(k=0;k<8;k++)
				{
					if(tmp&(0x80>>k))
						OLED_DrawPoint(x+(j%3)*8+k, y+j/3,1);
					else
						OLED_DrawPoint(x+(j%3)*8+k, y+j/3,0);
				}
			}	
			break;
		}	
	}
}
/**
    ****************************************************************************
    *@brief      写入一个32像素的汉字
    *@param      x,y :起点坐标; s：中文字符首地址; mode:0反白显示,1正常显示
    *@retval     无
    ****************************************************************************
    */
void GUI_ShowFont32(u8 x, u8 y, u8 *s, u8 mode)
{
	u8 i,j,k,tmp;
	u16 num;
	num = sizeof(cfont32)/sizeof(typFNT_GB32);
    for(i=0;i<num;i++)
	{
		if((cfont32[i].Index[0]==*s)&&(cfont32[i].Index[1]==*(s+1)))
		{
			for(j=0;j<128;j++)
			{
				if(mode)
					tmp = cfont32[i].Msk[j];
				else
					tmp = ~(cfont32[i].Msk[j]);
				for(k=0;k<8;k++)
				{
					if(tmp&(0x80>>k))
						OLED_DrawPoint(x+(j%4)*8+k, y+j/4,1);
					else
						OLED_DrawPoint(x+(j%4)*8+k, y+j/4,0);
				}
			}
			break;
		}
	}
}
/**
    ****************************************************************************
    *@brief      写入多个汉字
    *@param      x,y :起点坐标; size:字号; str：多个汉字首地址; mode:0反白显示,1正常显示
    *@retval     无
    ****************************************************************************
    */
void GUI_ShowCHinese(u8 x, u8 y, u8 hsize, u8 *str, u8 mode)
{ 
	while(*str != '\0')
	{
		if(hsize == 16)
			GUI_ShowFont16(x,y,str,mode);
		else if(hsize == 24)
			GUI_ShowFont24(x,y,str,mode);
		else if(hsize == 32)
			GUI_ShowFont32(x,y,str,mode);
		else    return;
		x += hsize;
		if(x > WIDTH-hsize)
		{
			x=0;
			y+=hsize;   //另起一行
		}
		str += 2;
	}
}

/**
    ****************************************************************************
    *@brief      写入图片，刷新后显示
    *@param      x,y:起始坐标，width：图片分辨率的宽度，height：图片分辨率的高度
    *            Image:图像数组名字，mode:1点亮数据点,0反白显示
    *@retval     无
    ****************************************************************************
    */
void OLED_Showbmp(u8 x,u8 y,u8 width, u8 height, const u8 *pic, u8 mode)
{
    u8 temp, j, y0 = y, *g_pic = NULL;
    u16 i, psize = 0;
    psize = (height/8 + ((height%8)?1:0)) * width;  //获取该图片的总字节数
    if ((x+width>128) || (y+height>64)) return;    //超出范围 直接返回
    g_pic = (u8 *)pic;
    for (i=0; i<psize; i++)
    {
        temp = g_pic[i];
        for (j=0; j<8; j++) /* 对一个字节中的8个位数据进行判断 */
        {
            if (temp & 0x80)    /* 高位存放的是低坐标 */
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y-y0) == height) /* 一列数据已经处理完毕 */
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}
/**
    ****************************************************************************
    *@brief      开机显示
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void StartingDisplay(void)
{
    OLED_Showbmp(27,0,74,64,Imagecar,1);
    OLED_Refresh_Gram();
    HAL_Delay(500);
    OLED_Clear();
    
    GUI_ShowCHinese(16,0,16,"河南科技大学",1);
    GUI_ShowCHinese(28,25,16,"电气",1);
    OLED_DrawString(60,25,"181",16);
    GUI_ShowCHinese(84,25,16,"班",1);
    GUI_ShowCHinese(0,47,16,"吴逍枫",1);
    OLED_DrawString(54,49,"181498000386",12);
    OLED_Refresh_Gram();
    HAL_Delay(1000);
    OLED_Clear();
}
/**
    ****************************************************************************
    *@brief      屏幕显示并刷新
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void Oled_Show()
{
    //=============第一行显示角度=======================//
    OLED_DrawString(00,00,"Angle",12);
    if( Angle_Balance<0)	OLED_DrawString(36,00,"-",12);
    if(Angle_Balance>=0)	OLED_DrawString(36,00,"+",12);
    OLED_ShowNumber(48,00, myabs((int)Angle_Balance),3,12);
    //=============第二行显示角速度===============//	
    OLED_DrawString(00,10,"Gyrox",12);
    if(Gyro_Balance<0)	  OLED_DrawString(36,10,"-",12);
    if(Gyro_Balance>=0)	  OLED_DrawString(36,10,"+",12);
    OLED_ShowNumber(42,10, myabs((int)Gyro_Balance),4,12);
    //=============第三行显示左编码器PWM与读数=======================//		
    OLED_DrawString(00,25,"L",12);
    if(Motor_Left<0)    OLED_DrawString(12,25,"-",12);
    if(Motor_Left>=0)   OLED_DrawString(12,25,"+",12);
    OLED_ShowNumber(18,25,myabs((int)Motor_Left),4,12);

    if(Velocity_Left<0)	  OLED_DrawString(60,25,"-",12);
    if(Velocity_Left>=0)	OLED_DrawString(60,25,"+",12);
    OLED_ShowNumber(68,25,myabs((int)Velocity_Left),4,12);
    OLED_DrawString(96,25,"mm/s",12);
    //=============第四行显示右编码器PWM与读数=======================//
    OLED_DrawString(00,36,"R",12);
    if(Motor_Right<0)   OLED_DrawString(12,36,"-",12);
    if(Motor_Right>=0)  OLED_DrawString(12,36,"+",12);
    OLED_ShowNumber(18,36,myabs((int)Motor_Right),4,12);

    if(Velocity_Right<0)    OLED_DrawString(60,36,"-",12);
    if(Velocity_Right>=0)   OLED_DrawString(60,36,"+",12);
    OLED_ShowNumber(68,36,myabs((int)Velocity_Right),4,12);
    OLED_DrawString(96,36,"mm/s",12);
    //=============第五行显示电压与电机开关=======================//
    OLED_DrawString(0,50,"V",12);
    OLED_ShowNumber(12,50,Voltage/100,2,12);
    OLED_DrawString(25,50,".",12);
    OLED_ShowNumber(30,50,Voltage%100,2,12);
    if(Voltage%100 < 10)
        OLED_DrawString(30,50,"0",12);  //若电压小数只有一位，则前一位补0
    OLED_DrawString(42,50,"V",12);
    OLED_DrawString(60,50,"motor",12);
    OLED_DrawString(92,50,":",12);
    if(Flag_Stop)   OLED_DrawString(102,50,"OFF",12);
    if(!Flag_Stop)  OLED_DrawString(102,50,"ON ",12);
    
    OLED_Showbmp(72,2,47,19,Imagewheel,1); //车轮图片
    OLED_Refresh_Gram();
}
/**
    ****************************************************************************
    *@brief      输出OLED_GRAM中的数据
    *@param      无
    *@retval     无
    ****************************************************************************
    *///debug长按按键断到此函数，打开串口调试助手，115200bps，打开串口运行一次该函数

void Get_GRAM (void)
{
    OLED_Clear();
        //先在此写入要提取的内容
    OLED_Refresh_Gram();
    
	u8 i,n;
	for(i=0;i<128;i++)
	{
        printf("{");
		for(n=0;n<8;n++)
        {
            printf("%#X", OLED_GRAM[i][n]);
            printf(",");
        }
        printf("},\n");
	}
}


