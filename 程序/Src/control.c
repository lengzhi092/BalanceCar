/**
    ****************************************************************************
    *@file       control.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      ���Ƶĺ��Ĳ���
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "control.h"
/**
    ****************************************************************************
    *@brief      �ⲿ�жϻص�����
    *@param      5ms�ⲿ�ж���MPU6050��INT��PA12�����Ŵ���,�ϸ�֤���������ݴ����ʱ��ͬ��
    *@retval     0
    ****************************************************************************
    */
int HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static int Voltage_Temp,Voltage_Count,Voltage_All;  //��ѹ������ر���
	static u8 Flag_Target;  //���ƺ�����ر������ṩ10ms��׼ 
	int Encoder_Left,Encoder_Right;    //���ұ��������������
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;  //ƽ�⻷PWM�������ٶȻ�PWM������ת��PWM����
	if(GPIO_Pin==MPU6050_EXTI_Pin)
	{  
        Flag_Target=!Flag_Target;
        Get_Angle();    //������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
        Encoder_Left=-Read_Encoder(2);  //��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
        Encoder_Right=-Read_Encoder(4); //��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
                                    //����A���TIM2_CH1,����A���TIM4_CH2,�����������������ļ�����ͬ
        Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);    //����������ת�ٶȣ�mm/s��
        if(Flag_Target==1)  //10ms����һ��
        {
            Voltage_Temp=Get_battery_volt();    //��ȡ��ص�ѹ		
            Voltage_Count++;    //ƽ��ֵ������
            Voltage_All+=Voltage_Temp;    //��β����ۻ�
            if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;   //��ƽ��ֵ		
            return 0;
        }
    	Key();    //����ɨ�账�� ����˫�����Ըı�С������״̬
    	Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //ƽ��PID���� Gyro_Balanceƽ����ٶȼ��ԣ�ǰ��Ϊ��������Ϊ��
    	Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //�ٶȻ�PID����	��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
    	Turn_Pwm=Turn(Gyro_Turn);    //ת��PID����

    	Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;   //�������ֵ������PWM
    	Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;  //�������ֵ������PWM
    													//PWMֵ����ʹС��ǰ��������ʹС������
    	Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
    	Motor_Right=PWM_Limit(Motor_Right,6900,-6900);  //PWM�޷�
    	if(Turn_Off(Angle_Balance,Voltage)==0)  //����������쳣
    		Set_Pwm(Motor_Left,Motor_Right);    //��ֵ��PWM�Ĵ���  
	}
  	return 0;
}
/**
    ****************************************************************************
    *@brief      ֱ��PD����
    *@param      Angle:�Ƕȣ�Gyro�����ٶ�
    *@retval     balance��ֱ������PWM
    ****************************************************************************
    */
int Balance(float Angle,float Gyro)
{
    float Angle_bias,Gyro_bias;
    int balance;
    Angle_bias=Middle_angle-Angle;  //���ƽ��ĽǶ���ֵ �ͻ�е���
    Gyro_bias=0-Gyro;
    balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100;    //����ƽ����Ƶĵ��PWM��PD���ƣ�kp��Pϵ����kd��Dϵ�� 
    return balance;
}
/**
    ****************************************************************************
    *@brief      �ٶȿ���PWM
    *@param      encoder_left�����ֱ�����������encoder_right�����ֱ���������
    *@retval     �ٶȿ���PWM
    ****************************************************************************
    *///�޸�ǰ�������ٶȣ����޸�Target_Velocity
int Velocity(int encoder_left,int encoder_right)
{
    static float velocity,Encoder_Latest,Encoder_bias,Movement;
    static float Encoder_Integral,Target_Velocity = 35;
	  //================ң��ǰ�����˲���====================//
    if(Flag_front==1)   Movement=Target_Velocity/Flag_velocity; //�յ�ǰ���ź�
    else if(Flag_back==1)   Movement=-Target_Velocity/Flag_velocity;    //�յ������ź�
    else    Movement=0;
   //================�ٶ�PI������=====================//	
    Encoder_Latest =0-(encoder_left+encoder_right);  //��ȡ�����ٶ�ƫ��=Ŀ���ٶȣ��˴�Ϊ�㣩-�����ٶȣ����ұ�����֮�ͣ� 
    Encoder_bias *= 0.86;   //һ�׵�ͨ�˲���ʱ����ʽΪ: y(n)= Kx(n) + (1-K)y(n-1) ȡk=0.14   
    Encoder_bias += Encoder_Latest*0.14; //һ�׵�ͨ�˲����������ٶȱ仯 
    Encoder_Integral +=Encoder_bias;    //���ֳ�λ�� ����ʱ�䣺5ms
    Encoder_Integral=Encoder_Integral+Movement; //����ң�������ݣ�����ǰ������
    if(Encoder_Integral>10000)  Encoder_Integral=10000; //�����޷�
    if(Encoder_Integral<-10000) Encoder_Integral=-10000;    //�����޷�	
    velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;    //�ٶ�PI����
    if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)    Encoder_Integral=0;    //����رպ��������
	return velocity;
}
/**
    ****************************************************************************
    *@brief      ת�����
    *@param      Z��������
    *@retval     ת�����PWM
    ****************************************************************************
    */
int Turn(float gyro)
{
    static float Turn_Target,turn,Turn_Amplitude=54;
    float Kp=Turn_Kp,Kd; //�޸�ת���ٶȣ����޸�Turn_Amplitude����
    //===================ң��������ת����=================//
    if(1==Flag_Left)   Turn_Target=-Turn_Amplitude/Flag_velocity;
    else if(1==Flag_Right) Turn_Target=Turn_Amplitude/Flag_velocity;
    else Turn_Target=0;
    
    if(1==Flag_front||1==Flag_back) Kd=Turn_Kd;
    else Kd=0;    //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
    //===================ת��PD������=================//
    turn=Turn_Target*Kp/100+gyro*Kd/100; //���Z�������ǽ���PD����
    return turn;   //ת��PWM��תΪ������תΪ��
}
/**
    ****************************************************************************
    *@brief      ����ֵ����
    *@param      a����Ҫ�������ֵ����
    *@retval     u32
    ****************************************************************************
    */
u32 myabs(int a)
{
	int temp;
	if(a<0)    temp=-a;  
	else    temp=a;
	return temp;
}
/**
    ****************************************************************************
    *@brief      ��ֵ��PWM�Ĵ���
    *@param      ����PWM������PWM
    *@retval     ��
    ****************************************************************************
    */
void Set_Pwm(int motor_left,int motor_right)
{
    if(motor_left>0)    BIN1=1, BIN2=0; //ǰ�� 
	else    BIN1=0, BIN2=1; //����
        PWMB=myabs(motor_left);	
    if(motor_right>0)    AIN2=1,AIN1=0; //ǰ��
    else    AIN2=0, AIN1=1; //����
        PWMA=myabs(motor_right);
}
/**
    ****************************************************************************
    *@brief      ����PWM��ֵ
    *@param      IN���������  max���޷����ֵ  min���޷���Сֵ
    *@retval     �޷����ֵ
    ****************************************************************************
    */
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max)    OUT = max;
	if(OUT<min)    OUT = min;
	return OUT;
}
/**
    ****************************************************************************
    *@brief      �쳣�رյ��
    *@param      angle��С����ǣ�voltage����ѹ
    *@retval     1���쳣  0������
    ****************************************************************************
    */
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1110)//��ص�ѹ����11.1V�رյ��
	{   //��Ǵ���40�ȹرյ��
		temp=1;
		AIN1=0; AIN2=0; BIN1=0; BIN2=0;
	}
	else
		temp=0;
	return temp;			
}
