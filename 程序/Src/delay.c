/**
    ****************************************************************************
    *@file       delay.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.
    *@brief      �ṩms��us����ʱ����
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "delay.h"
/**
    ****************************************************************************
    *@brief      �ӳٳ�ʼ��
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
static u8  fac_us=0;    //us��ʱ������			   
static u16 fac_ms=0;    //ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

void delay_init()
{
	SysTick->CTRL &= ~(1<<2) ;   //����SysTickʹ���ⲿʱ��Դ����AHB����ʱ�ӵ�1/8  �� 72MHz/8 = 9MHz
	fac_us= 9;  //SysTick����һ������Ҫ 1/9MHz �� �� ����9��������Ҫ 9* 1/9MHz = 1us  ��������ʱ����delay_us�������ֵ�ǡ���Ҫ���ٸ�1us��,delay_msͬ��
	fac_ms=(u16)fac_us*1000;     //1ms = 1000us
}
/**
    ****************************************************************************
    *@brief      ��ʱnus
    *@param      nus��Ҫ��ʱ��us��
    *@retval     ��
    ****************************************************************************
    */
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us;   //ʱ�����	  		 
	SysTick->VAL=0x00;  //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;	//��ʼ����	  
	do  temp=SysTick->CTRL;
	while((temp&0x01)&&!(temp&(1<<16)));    //�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;    //�رռ�����
	SysTick->VAL =0X00; //��ռ�����
}
/**
    ****************************************************************************
    *@brief      ��ʱnms
    *@param      mus��Ҫ��ʱ��ms��,72M������,nms<=1864
    *SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:nms<=0xffffff*8*1000/SYSCLK,SYSCLK��λΪHz
    *@retval     ��
    ****************************************************************************
    */
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;  //ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00; //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;    //��ʼ����  
	do  temp=SysTick->CTRL;
	while((temp&0x01)&&!(temp&(1<<16)));    //�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;    //�رռ�����
	SysTick->VAL =0X00; //��ռ�����	  	    
}
