/**
    ****************************************************************************
    *@file       DateSend.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.21
    *@brief      进行上位机数据处理和发送
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "DataSend.h"
unsigned char DataScope_OutPut_Buffer[42] = {0};   //串口发送缓冲区

/**
    ****************************************************************************
    *@brief      将单精度浮点数据转成4字节(32位)数据并存入指定地址
    *@param      target:目标单精度数据；buf:待写入数组；beg:指定从数组第几个元素开始写入
    *@retval     无
    ****************************************************************************
    */
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;  //此处为数组指针，4个成员。u8 (*p)[4];
    point = (unsigned char*)target; //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
/**
    ****************************************************************************
    *@brief      将待发送通道的单精度浮点数据写入发送缓冲区
    *@param      Data：通道数据；Channel：选择通道（1-10）
    *@retval     无
    ****************************************************************************
    */
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
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
    *@brief      生成 DataScopeV1.0 能正确识别的帧格式
    *@param      Channel_Number，需要发送的通道个数
    *@retval     发送缓冲区数据个数，0表示帧格式生成失败
    ****************************************************************************
    */
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) )   return 0;    //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        DataScope_OutPut_Buffer[0] = '$';   //帧头
        switch(Channel_Number)  //帧尾
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
    *@brief      虚拟示波器往上位机发送数据 关闭显示屏
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void DataScope(void)
{
    u8 i, Send_Count;   //计数变量 | 串口需要发送的数据个数
//	float Vol=(float)Voltage/100;  //电压变量
	DataScope_Get_Channel_Data(Angle_Balance, 1);   //显示角度 单位：度（°）
    DataScope_Get_Channel_Data(Accel_Angle_x, 2);
//	DataScope_Get_Channel_Data(Vol, 3); //显示电池电压 单位：V
//    DataScope_Get_Channel_Data(0, 4); //用您要显示的数据变量名替换0就行了
//    DataScope_Get_Channel_Data(0, 5);
//    DataScope_Get_Channel_Data(0, 6);
//    DataScope_Get_Channel_Data(0, 7);
//    DataScope_Get_Channel_Data(0, 8);
//    DataScope_Get_Channel_Data(0, 9);
//    DataScope_Get_Channel_Data(0, 10);
    Send_Count = DataScope_Data_Generate(2);    //增加发送通道数时这里要改
	for(i=0; i<Send_Count; i++)
	{
		while((USART1->SR&0X40)==0);
		USART1->DR = DataScope_OutPut_Buffer[i];
	}
}
/**
    ****************************************************************************
    *@brief      向APP发送数据
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void APP_Show(void)
{
    static u8 flag;
	int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1110)*2/3;    //对电压数据进行处理
    if(Voltage_Show<0)  Voltage_Show=0;
    if(Voltage_Show>100)    Voltage_Show=100;
	Encoder_Right_Show=Velocity_Right*1.1;  //对编码器数据就行数据处理便于图形化
    if(Encoder_Right_Show<0)    Encoder_Right_Show=-Encoder_Right_Show;
	Encoder_Left_Show=Velocity_Left*1.1;
    if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
	flag=!flag;
	if(PID_Send==1) //发送PID参数,在APP调参界面显示
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
               (int)Balance_Kp,(int)Balance_Kd,(int)Velocity_Kp,(int)Velocity_Ki,(int)Turn_Kp,(int)Turn_Kd,0,0,0); //打印到APP上面
        PID_Send=0;
    }
    else    if(flag==0) // 发送电池电压，速度，角度等参数，在APP首页显示
		printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)Angle_Balance);  //打印到APP上面
    else    //发送小车姿态角，在波形界面显示
        printf("{B%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw);   //x，y，z轴角度 在APP上面显示波形
                                                              //可按格式自行增加显示波形，最多可显示五个
}
