#include "sys.h"


//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//�ر������ж�
__asm void INTX_DISABLE(void)
{
	CPSID I;		  
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE I;		  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
/**
    ****************************************************************************
    *@brief      
    *@param      
    *@retval     
    ****************************************************************************
    */
/**************************************************************************
Function: Set JTAG mode
Input   : mode:JTAG, swd mode settings��00��all enable��01��enable SWD��10��Full shutdown
Output  : none
�������ܣ�����JTAGģʽ
��ڲ�����mode:jtag,swdģʽ����;00,ȫʹ��;01,ʹ��SWD;10,ȫ�ر�;	
����  ֵ����
**************************************************************************/
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00	
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
	AFIO->MAPR|=temp;       //����jtagģʽ
} 
