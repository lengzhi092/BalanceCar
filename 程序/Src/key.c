/**
    ****************************************************************************
    *@file       key.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.20
    *@brief      按键识别
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "key.h"
/**
    ****************************************************************************
    *@brief      按键扫描
    *@param      双击等待时间
    *@retval     按键状态 0：无动作 1：单击 2：双击 
    ****************************************************************************
    */
u8 click_N_Double (u8 time)
{
	static	u8 flag_key,count_key,double_key;	
	static	u16 count_single,Forever_count;
	if(KEY==0)  Forever_count++;   //长按标志位未置1
	else        Forever_count=0;
	if(0==KEY&&0==flag_key) flag_key=1; //第一次按下
	if(0==count_key)
	{
        if(flag_key==1) 
        {
            double_key++;
            count_key=1;    //标记按下一次
        }
        if(double_key==2)   //按下两次
        {
            double_key=0;
            count_single=0;
            return 2;//双击执行的指令
        }
	}
	if(1==KEY)  flag_key=0,count_key=0;
	if(1==double_key)
	{
		count_single++;
		if(count_single>time&&Forever_count<time)
		{
			double_key=0;
			count_single=0;	//超时不标记为双击
		return 1;//单击执行的指令
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
    *@brief      按键扫描
    *@param      无
    *@retval     按键状态 0：无动作 1：单击
    ****************************************************************************
    */
u8 click(void)
{
	static u8 flag_key=1;   //按键按松开标志
	if(flag_key&&KEY==0)    //检测到按键按下
	{
		flag_key=0;
		return 1;   //按键按下
	}
	else if(1==KEY) flag_key=1;
	return 0;   //无按键按下
}
/**
    ****************************************************************************
    *@brief      长按检测
    *@param      无
    *@retval     按键状态 0：无动作 1：长按2s
    ****************************************************************************
    */
u8 Long_Press(void)
{
    static u16 Long_Press_count,Long_Press;
    if(Long_Press==0&&KEY==0)   Long_Press_count++; //长按标志位未置1
    else    Long_Press_count=0; 
    if(Long_Press_count>200)    //10ms扫描一次
    {
        Long_Press=1;	
        Long_Press_count=0;
        return 1;
    }				
    if(Long_Press==1)   //长按标志位置1
    {
        Long_Press=0;
    }
    return 0;
}
/**
    ****************************************************************************
    *@brief      按键修改小车运行状态 
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void Key(void)
{
	u8 tmp,tmp2;
	tmp=click_N_Double(50);
	if(tmp==1)  Flag_Stop=!Flag_Stop;   //单击控制小车的启停
	tmp2=Long_Press();
    if(tmp2==1) Flag_Show=!Flag_Show;   //长按控制进入上位机模式，小车的显示停止
}
