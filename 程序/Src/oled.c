/**
    ****************************************************************************
    *@file       oled.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      OLED��ʾ������
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
    *@brief      ��OLEDд��һ���ֽڡ�
    *@param      dat:Ҫд�������/����;dc:����/�����־,0��ʾ����;1��ʾ����
    *@retval     ��
    ****************************************************************************
    */
void OLED_WR_Byte(u8 dat,u8 dc)
{
	u8 i;
	if(dc)
	  OLED_DC_Set();    //DC�˿�����ģʽ
	else
	  OLED_DC_Clr();    //DC�˿�����ģʽ
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
    *@brief      ˢ��OLED��Ļ
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void OLED_Refresh_Gram()
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾ��ʼ�е�ַ������λ��
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾ��ʼ�е�ַ������λ��
		for(n=0;n<128;n++)
        OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

/**
    ****************************************************************************
    *@brief      ����
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void OLED_Clear()
{
	u8 i,n;
	for(i=0;i<8;i++)
        for(n=0;n<128;n++)
            OLED_GRAM[n][i]=0X00;  
    OLED_Refresh_Gram();  //������ʾ
}
/**
    ****************************************************************************
    *@brief      ��ʼ��OLED
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void OLED_Init()
{
	OLED_RES_Clr();    //��λ
	HAL_Delay(100);
	OLED_RES_Set();

	OLED_WR_Byte(0xAE,OLED_CMD); //�ر���ʾ
	OLED_WR_Byte(0xD5,OLED_CMD); //����ʱ�ӷ�Ƶ����,��Ƶ��
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],��Ƶ����;[7:4],��Ƶ��
	OLED_WR_Byte(0xA8,OLED_CMD); //��������·��
	OLED_WR_Byte(0X3F,OLED_CMD); //Ĭ��0X3F(1/64)
	OLED_WR_Byte(0xD3,OLED_CMD); //������ʾƫ��
	OLED_WR_Byte(0X00,OLED_CMD); //Ĭ��Ϊ0
	OLED_WR_Byte(0x40,OLED_CMD); //������ʾ��ʼ�� [5:0],����
	OLED_WR_Byte(0x8D,OLED_CMD); //��ɱ�����
	OLED_WR_Byte(0x14,OLED_CMD); //bit2������/�ر�
	OLED_WR_Byte(0x20,OLED_CMD); //�����ڴ��ַģʽ
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00���е�ַģʽ;01���е�ַģʽ;10,ҳ��ַģʽ;Ĭ��10;
	OLED_WR_Byte(0xA1,OLED_CMD); //���ض�������,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //����COMɨ�跽��;bit3:0,��ͨģʽ;1,�ض���ģʽ COM[N-1]->COM0;N:����·��
	OLED_WR_Byte(0xDA,OLED_CMD); //����COMӲ����������
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]����	
	OLED_WR_Byte(0x81,OLED_CMD); //�Աȶ�����
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;Ĭ��0X7F (��������,Խ��Խ��)
	OLED_WR_Byte(0xD9,OLED_CMD); //����Ԥ�������
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //����VCOMH ��ѹ����
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
	OLED_WR_Byte(0xA4,OLED_CMD); //ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����)
	OLED_WR_Byte(0xA6,OLED_CMD); //������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ
	OLED_WR_Byte(0xAF,OLED_CMD); //������ʾ
	OLED_Clear();
}  
/**
    ****************************************************************************
    *@brief      ��ͼ��Ĵ���GRAM��д��һ���㣬ˢ�º���ʾ
    *@param      x,y:�������;mode:0������ʾ,1������ʾ
    *@retval     ��
    ****************************************************************************
    */
void OLED_DrawPoint(u8 x,u8 y,u8 mode)
{
	u8 pos,temp=0;
	if(x>127||y>63)
        return;//������Χ��
	pos = 7-y/8;  //ҳ��
	temp = 1<<(7-y%8);   //ҳ�е�λ��
	if(mode)
        OLED_GRAM[x][pos] |= temp;
	else
        OLED_GRAM[x][pos] &= ~temp;
}
/**
    ****************************************************************************
    *@brief      ��ָ��λ��д��һ���ַ���95��ascll�е����ݣ���ˢ�º���ʾ
    *@param      x,y :�������; chr:�ַ�; size:�ֺ�; mode:0������ʾ,1������ʾ
    *@retval     ��
    ****************************************************************************
    */
void OLED_DrawChar(u8 x, u8 y, u8 chr, u8 size, u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
    u8 csize=(size/8+((size%8)?1:0))*(size/2);  //�õ�����һ���ַ���Ӧ������ռ���ֽ���
	chr=chr-' ';	//�õ�ƫ�ƺ��ֵ
    for(t=0;t<csize;t++)
    {
			if(size==12)  temp=asc2_1206[chr][t];   //����1206����
			else if(size==16)temp=asc2_1608[chr][t];    //����1608����
            else if(size==24)temp=asc2_2412[chr][t];	//����2412����
            else return;	//û�е��ֿ�	                          
            for(t1=0;t1<8;t1++) //д��һ���ֽ�
			{
				if(temp&0x80)  OLED_DrawPoint(x,y,mode);
				else           OLED_DrawPoint(x,y,!mode);
				temp <<= 1;
				y++;
				if((y-y0) == size)  //��ɵ����һ��
				{
					y=y0;
					x++;    //��ʾ������һ��
					break;
				}
			}
    }
}
/**
    ****************************************************************************
    *@brief      m��n�η����������OLED_ShowNumber��������
    *@param      m��������n���η���
    *@retval     ��
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
    *@brief      ��ʾn������
    *@param      x,y:�������; num:��ֵ;len:���ֵ�λ��; size:�����С
    *@retval     ��
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
    *@brief      ��ָ��λ��д���ַ�����95��ascll�е����ݣ���ˢ�º���ʾ
    *@param      x,y:������꣬*p:�ַ�����ʼ��ַ,sise:�ֺ�
    *@retval     ��
    ****************************************************************************
    */

void OLED_DrawString(u8 x,u8 y,const u8 *p,u8 size)
{
    while(*p != '\0')   //\0�����ַ������ַ����Ľ�����
    {
        if(x>(128-(size/2)))    //ʣ��λ�ÿ�Ȳ�������һ���ַ�����
            {x=0;y+=size;}  //����������һ��
        if(y>(64-size))    //��д����
            {y=x=0;OLED_Clear();}
        OLED_DrawChar(x,y,*p,size,1);
        x+=size/2;
        p++;
    }
}
/**
    ****************************************************************************
    *@brief      д��һ��16���صĺ���
    *@param      x,y :�������; s�������ַ��׵�ַ; mode:0������ʾ,1������ʾ
    *@retval     ��
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
    *@brief      д��һ��24���صĺ���
    *@param      x,y :�������; s�������ַ��׵�ַ; mode:0������ʾ,1������ʾ
    *@retval     ��
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
    *@brief      д��һ��32���صĺ���
    *@param      x,y :�������; s�������ַ��׵�ַ; mode:0������ʾ,1������ʾ
    *@retval     ��
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
    *@brief      д��������
    *@param      x,y :�������; size:�ֺ�; str����������׵�ַ; mode:0������ʾ,1������ʾ
    *@retval     ��
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
			y+=hsize;   //����һ��
		}
		str += 2;
	}
}

/**
    ****************************************************************************
    *@brief      д��ͼƬ��ˢ�º���ʾ
    *@param      x,y:��ʼ���꣬width��ͼƬ�ֱ��ʵĿ�ȣ�height��ͼƬ�ֱ��ʵĸ߶�
    *            Image:ͼ���������֣�mode:1�������ݵ�,0������ʾ
    *@retval     ��
    ****************************************************************************
    */
void OLED_Showbmp(u8 x,u8 y,u8 width, u8 height, const u8 *pic, u8 mode)
{
    u8 temp, j, y0 = y, *g_pic = NULL;
    u16 i, psize = 0;
    psize = (height/8 + ((height%8)?1:0)) * width;  //��ȡ��ͼƬ�����ֽ���
    if ((x+width>128) || (y+height>64)) return;    //������Χ ֱ�ӷ���
    g_pic = (u8 *)pic;
    for (i=0; i<psize; i++)
    {
        temp = g_pic[i];
        for (j=0; j<8; j++) /* ��һ���ֽ��е�8��λ���ݽ����ж� */
        {
            if (temp & 0x80)    /* ��λ��ŵ��ǵ����� */
                OLED_DrawPoint(x, y, mode);
            else
                OLED_DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y-y0) == height) /* һ�������Ѿ�������� */
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
    *@brief      ������ʾ
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void StartingDisplay(void)
{
    OLED_Showbmp(27,0,74,64,Imagecar,1);
    OLED_Refresh_Gram();
    HAL_Delay(500);
    OLED_Clear();
    
    GUI_ShowCHinese(16,0,16,"���ϿƼ���ѧ",1);
    GUI_ShowCHinese(28,25,16,"����",1);
    OLED_DrawString(60,25,"181",16);
    GUI_ShowCHinese(84,25,16,"��",1);
    GUI_ShowCHinese(0,47,16,"���з�",1);
    OLED_DrawString(54,49,"181498000386",12);
    OLED_Refresh_Gram();
    HAL_Delay(1000);
    OLED_Clear();
}
/**
    ****************************************************************************
    *@brief      ��Ļ��ʾ��ˢ��
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void Oled_Show()
{
    //=============��һ����ʾ�Ƕ�=======================//
    OLED_DrawString(00,00,"Angle",12);
    if( Angle_Balance<0)	OLED_DrawString(36,00,"-",12);
    if(Angle_Balance>=0)	OLED_DrawString(36,00,"+",12);
    OLED_ShowNumber(48,00, myabs((int)Angle_Balance),3,12);
    //=============�ڶ�����ʾ���ٶ�===============//	
    OLED_DrawString(00,10,"Gyrox",12);
    if(Gyro_Balance<0)	  OLED_DrawString(36,10,"-",12);
    if(Gyro_Balance>=0)	  OLED_DrawString(36,10,"+",12);
    OLED_ShowNumber(42,10, myabs((int)Gyro_Balance),4,12);
    //=============��������ʾ�������PWM�����=======================//		
    OLED_DrawString(00,25,"L",12);
    if(Motor_Left<0)    OLED_DrawString(12,25,"-",12);
    if(Motor_Left>=0)   OLED_DrawString(12,25,"+",12);
    OLED_ShowNumber(18,25,myabs((int)Motor_Left),4,12);

    if(Velocity_Left<0)	  OLED_DrawString(60,25,"-",12);
    if(Velocity_Left>=0)	OLED_DrawString(60,25,"+",12);
    OLED_ShowNumber(68,25,myabs((int)Velocity_Left),4,12);
    OLED_DrawString(96,25,"mm/s",12);
    //=============��������ʾ�ұ�����PWM�����=======================//
    OLED_DrawString(00,36,"R",12);
    if(Motor_Right<0)   OLED_DrawString(12,36,"-",12);
    if(Motor_Right>=0)  OLED_DrawString(12,36,"+",12);
    OLED_ShowNumber(18,36,myabs((int)Motor_Right),4,12);

    if(Velocity_Right<0)    OLED_DrawString(60,36,"-",12);
    if(Velocity_Right>=0)   OLED_DrawString(60,36,"+",12);
    OLED_ShowNumber(68,36,myabs((int)Velocity_Right),4,12);
    OLED_DrawString(96,36,"mm/s",12);
    //=============��������ʾ��ѹ��������=======================//
    OLED_DrawString(0,50,"V",12);
    OLED_ShowNumber(12,50,Voltage/100,2,12);
    OLED_DrawString(25,50,".",12);
    OLED_ShowNumber(30,50,Voltage%100,2,12);
    if(Voltage%100 < 10)
        OLED_DrawString(30,50,"0",12);  //����ѹС��ֻ��һλ����ǰһλ��0
    OLED_DrawString(42,50,"V",12);
    OLED_DrawString(60,50,"motor",12);
    OLED_DrawString(92,50,":",12);
    if(Flag_Stop)   OLED_DrawString(102,50,"OFF",12);
    if(!Flag_Stop)  OLED_DrawString(102,50,"ON ",12);
    
    OLED_Showbmp(72,2,47,19,Imagewheel,1); //����ͼƬ
    OLED_Refresh_Gram();
}
/**
    ****************************************************************************
    *@brief      ���OLED_GRAM�е�����
    *@param      ��
    *@retval     ��
    ****************************************************************************
    *///debug���������ϵ��˺������򿪴��ڵ������֣�115200bps���򿪴�������һ�θú���

void Get_GRAM (void)
{
    OLED_Clear();
        //���ڴ�д��Ҫ��ȡ������
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


