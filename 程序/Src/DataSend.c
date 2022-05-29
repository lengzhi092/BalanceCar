/**
    ****************************************************************************
    *@file       DateSend.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.21
    *@brief      ������λ�����ݴ���ͷ���
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "DataSend.h"
unsigned char DataScope_OutPut_Buffer[42] = {0};   //���ڷ��ͻ�����

/**
    ****************************************************************************
    *@brief      �������ȸ�������ת��4�ֽ�(32λ)���ݲ�����ָ����ַ
    *@param      target:Ŀ�굥�������ݣ�buf:��д�����飻beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
    *@retval     ��
    ****************************************************************************
    */
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;  //�˴�Ϊ����ָ�룬4����Ա��u8 (*p)[4];
    point = (unsigned char*)target; //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
/**
    ****************************************************************************
    *@brief      ��������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
    *@param      Data��ͨ�����ݣ�Channel��ѡ��ͨ����1-10��
    *@retval     ��
    ****************************************************************************
    */
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) return;  //ͨ����������10�����0��ֱ����������ִ�к���
    else
    {
        switch (Channel)
		{
            case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
            case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
            case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
            case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
            case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
            case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
            case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
            case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
            case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
            case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
        }
    }	 
}
/**
    ****************************************************************************
    *@brief      ���� DataScopeV1.0 ����ȷʶ���֡��ʽ
    *@param      Channel_Number����Ҫ���͵�ͨ������
    *@retval     ���ͻ��������ݸ�����0��ʾ֡��ʽ����ʧ��
    ****************************************************************************
    */
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) )   return 0;    //ͨ����������10�����0��ֱ����������ִ�к���
    else
    {
        DataScope_OutPut_Buffer[0] = '$';   //֡ͷ
        switch(Channel_Number)  //֡β
        {
            case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;
            case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
            case 3:   DataScope_OutPut_Buffer[13] = 13; return 14;
            case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
            case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;
            case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
            case 7:   DataScope_OutPut_Buffer[29] = 29; return 30;
            case 8:   DataScope_OutPut_Buffer[33] = 33; return 34;
            case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
            case 10:  DataScope_OutPut_Buffer[41] = 41; return 42;
        }
    }
	return 0;
}
/**
    ****************************************************************************
    *@brief      ����ʾ��������λ���������� �ر���ʾ��
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void DataScope(void)
{
    u8 i, Send_Count;   //�������� | ������Ҫ���͵����ݸ���
//	float Vol=(float)Voltage/100;  //��ѹ����
	DataScope_Get_Channel_Data(Angle_Balance, 1);   //��ʾ�Ƕ� ��λ���ȣ��㣩
    DataScope_Get_Channel_Data(Accel_Angle_x, 2);
//	DataScope_Get_Channel_Data(Vol, 3); //��ʾ��ص�ѹ ��λ��V
//    DataScope_Get_Channel_Data(0, 4); //����Ҫ��ʾ�����ݱ������滻0������
//    DataScope_Get_Channel_Data(0, 5);
//    DataScope_Get_Channel_Data(0, 6);
//    DataScope_Get_Channel_Data(0, 7);
//    DataScope_Get_Channel_Data(0, 8);
//    DataScope_Get_Channel_Data(0, 9);
//    DataScope_Get_Channel_Data(0, 10);
    Send_Count = DataScope_Data_Generate(2);    //���ӷ���ͨ����ʱ����Ҫ��
	for(i=0; i<Send_Count; i++)
	{
		while((USART1->SR&0X40)==0);
		USART1->DR = DataScope_OutPut_Buffer[i];
	}
}
/**
    ****************************************************************************
    *@brief      ��APP��������
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void APP_Show(void)
{
    static u8 flag;
	int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1110)*2/3;    //�Ե�ѹ���ݽ��д���
    if(Voltage_Show<0)  Voltage_Show=0;
    if(Voltage_Show>100)    Voltage_Show=100;
	Encoder_Right_Show=Velocity_Right*1.1;  //�Ա��������ݾ������ݴ������ͼ�λ�
    if(Encoder_Right_Show<0)    Encoder_Right_Show=-Encoder_Right_Show;
	Encoder_Left_Show=Velocity_Left*1.1;
    if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
	flag=!flag;
	if(PID_Send==1) //����PID����,��APP���ν�����ʾ
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
               (int)Balance_Kp,(int)Balance_Kd,(int)Velocity_Kp,(int)Velocity_Ki,(int)Turn_Kp,(int)Turn_Kd,0,0,0); //��ӡ��APP����
        PID_Send=0;
    }
    else    if(flag==0) // ���͵�ص�ѹ���ٶȣ��ǶȵȲ�������APP��ҳ��ʾ
		printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)Angle_Balance);  //��ӡ��APP����
    else    //����С����̬�ǣ��ڲ��ν�����ʾ
        printf("{B%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw);   //x��y��z��Ƕ� ��APP������ʾ����
                                                              //�ɰ���ʽ����������ʾ���Σ�������ʾ���
}
