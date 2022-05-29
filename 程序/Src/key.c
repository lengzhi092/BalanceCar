/**
    ****************************************************************************
    *@file       key.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.20
    *@brief      ����ʶ��
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "key.h"
/**
    ****************************************************************************
    *@brief      ����ɨ��
    *@param      ˫���ȴ�ʱ��
    *@retval     ����״̬ 0���޶��� 1������ 2��˫�� 
    ****************************************************************************
    */
u8 click_N_Double (u8 time)
{
	static	u8 flag_key,count_key,double_key;	
	static	u16 count_single,Forever_count;
	if(KEY==0)  Forever_count++;   //������־λδ��1
	else        Forever_count=0;
	if(0==KEY&&0==flag_key) flag_key=1; //��һ�ΰ���
	if(0==count_key)
	{
        if(flag_key==1) 
        {
            double_key++;
            count_key=1;    //��ǰ���һ��
        }
        if(double_key==2)   //��������
        {
            double_key=0;
            count_single=0;
            return 2;//˫��ִ�е�ָ��
        }
	}
	if(1==KEY)  flag_key=0,count_key=0;
	if(1==double_key)
	{
		count_single++;
		if(count_single>time&&Forever_count<time)
		{
			double_key=0;
			count_single=0;	//��ʱ�����Ϊ˫��
		return 1;//����ִ�е�ָ��
		}
		if(Forever_count>time)
		{
			double_key=0;
			count_single=0;	
		}
	}	
	return 0;
}
/**
    ****************************************************************************
    *@brief      ����ɨ��
    *@param      ��
    *@retval     ����״̬ 0���޶��� 1������
    ****************************************************************************
    */
u8 click(void)
{
	static u8 flag_key=1;   //�������ɿ���־
	if(flag_key&&KEY==0)    //��⵽��������
	{
		flag_key=0;
		return 1;   //��������
	}
	else if(1==KEY) flag_key=1;
	return 0;   //�ް�������
}
/**
    ****************************************************************************
    *@brief      �������
    *@param      ��
    *@retval     ����״̬ 0���޶��� 1������2s
    ****************************************************************************
    */
u8 Long_Press(void)
{
    static u16 Long_Press_count,Long_Press;
    if(Long_Press==0&&KEY==0)   Long_Press_count++; //������־λδ��1
    else    Long_Press_count=0; 
    if(Long_Press_count>200)    //10msɨ��һ��
    {
        Long_Press=1;	
        Long_Press_count=0;
        return 1;
    }				
    if(Long_Press==1)   //������־λ��1
    {
        Long_Press=0;
    }
    return 0;
}
/**
    ****************************************************************************
    *@brief      �����޸�С������״̬ 
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void Key(void)
{
	u8 tmp,tmp2;
	tmp=click_N_Double(50);
	if(tmp==1)  Flag_Stop=!Flag_Stop;   //��������С������ͣ
	tmp2=Long_Press();
    if(tmp2==1) Flag_Show=!Flag_Show;   //�������ƽ�����λ��ģʽ��С������ʾֹͣ
}
