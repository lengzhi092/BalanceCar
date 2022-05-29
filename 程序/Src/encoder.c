/**
    ****************************************************************************
    *@file       encoder.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.20
    *@brief      ����������
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "encoder.h"

/**
    ****************************************************************************
    *@brief      ��λʱ���ȡ����������
    *@param      TIMX����ʱ��
    *@retval     �ٶ�ֵ
    ****************************************************************************
    */
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
    switch(TIMX)
    {
        case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;   //short��ǿ������ת��
        case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
        case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
        default: Encoder_TIM=0;
    }
    return Encoder_TIM;
}
/**
    ****************************************************************************
    *@brief      ����������ת��Ϊ�ٶȣ�mm/s��
    *@param      encoder_left�����ֱ�����������encoder_right�����ֱ���������
    *@retval     ��
    ****************************************************************************
    */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;    //���ת��=������������5msÿ�Σ�*��ȡƵ��/��Ƶ��/���ٱ�/����������
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;    //����������ٶ�=ת��*�ܳ�
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;    //����������ٶ�=ת��*�ܳ�
}
