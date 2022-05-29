/**
    ****************************************************************************
    *@file       encoder.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.20
    *@brief      编码器驱动
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "encoder.h"

/**
    ****************************************************************************
    *@brief      单位时间读取编码器计数
    *@param      TIMX：定时器
    *@retval     速度值
    ****************************************************************************
    */
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
    switch(TIMX)
    {
        case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;   //short：强制类型转换
        case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
        case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
        default: Encoder_TIM=0;
    }
    return Encoder_TIM;
}
/**
    ****************************************************************************
    *@brief      编码器读数转换为速度（mm/s）
    *@param      encoder_left：左轮编码器读数，encoder_right：右轮编码器读数
    *@retval     无
    ****************************************************************************
    */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;    //电机转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;    //求出编码器速度=转速*周长
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;    //求出编码器速度=转速*周长
}
