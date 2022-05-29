/**
    ****************************************************************************
    *@file       control.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      控制的核心部分
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "control.h"
/**
    ****************************************************************************
    *@brief      外部中断回调函数
    *@param      5ms外部中断由MPU6050的INT（PA12）引脚触发,严格保证采样和数据处理的时间同步
    *@retval     0
    ****************************************************************************
    */
int HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static int Voltage_Temp,Voltage_Count,Voltage_All;  //电压测量相关变量
	static u8 Flag_Target;  //控制函数相关变量，提供10ms基准 
	int Encoder_Left,Encoder_Right;    //左右编码器的脉冲计数
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;  //平衡环PWM变量，速度环PWM变量，转向环PWM变量
	if(GPIO_Pin==MPU6050_EXTI_Pin)
	{  
        Flag_Target=!Flag_Target;
        Get_Angle();    //更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
        Encoder_Left=-Read_Encoder(2);  //读取左轮编码器的值，前进为正，后退为负
        Encoder_Right=-Read_Encoder(4); //读取右轮编码器的值，前进为正，后退为负
                                    //左轮A相接TIM2_CH1,右轮A相接TIM4_CH2,故这里两个编码器的极性相同
        Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);    //编码器读数转速度（mm/s）
        if(Flag_Target==1)  //10ms控制一次
        {
            Voltage_Temp=Get_battery_volt();    //读取电池电压		
            Voltage_Count++;    //平均值计数器
            Voltage_All+=Voltage_Temp;    //多次采样累积
            if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;   //求平均值		
            return 0;
        }
    	Key();    //按键扫描处理 单击双击可以改变小车运行状态
    	Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
    	Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
    	Turn_Pwm=Turn(Gyro_Turn);    //转向环PID控制

    	Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;   //计算左轮电机最终PWM
    	Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;  //计算右轮电机最终PWM
    													//PWM值正数使小车前进，负数使小车后退
    	Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
    	Motor_Right=PWM_Limit(Motor_Right,6900,-6900);  //PWM限幅
    	if(Turn_Off(Angle_Balance,Voltage)==0)  //如果不存在异常
    		Set_Pwm(Motor_Left,Motor_Right);    //赋值给PWM寄存器  
	}
  	return 0;
}
/**
    ****************************************************************************
    *@brief      直立PD控制
    *@param      Angle:角度；Gyro：角速度
    *@retval     balance：直立控制PWM
    ****************************************************************************
    */
int Balance(float Angle,float Gyro)
{
    float Angle_bias,Gyro_bias;
    int balance;
    Angle_bias=Middle_angle-Angle;  //求出平衡的角度中值 和机械相关
    Gyro_bias=0-Gyro;
    balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100;    //计算平衡控制的电机PWM：PD控制：kp是P系数，kd是D系数 
    return balance;
}
/**
    ****************************************************************************
    *@brief      速度控制PWM
    *@param      encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
    *@retval     速度控制PWM
    ****************************************************************************
    *///修改前进后退速度，请修改Target_Velocity
int Velocity(int encoder_left,int encoder_right)
{
    static float velocity,Encoder_Latest,Encoder_bias,Movement;
    static float Encoder_Integral,Target_Velocity = 35;
	  //================遥控前进后退部分====================//
    if(Flag_front==1)   Movement=Target_Velocity/Flag_velocity; //收到前进信号
    else if(Flag_back==1)   Movement=-Target_Velocity/Flag_velocity;    //收到后退信号
    else    Movement=0;
   //================速度PI控制器=====================//	
    Encoder_Latest =0-(encoder_left+encoder_right);  //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和） 
    Encoder_bias *= 0.86;   //一阶低通滤波器时域表达式为: y(n)= Kx(n) + (1-K)y(n-1) 取k=0.14   
    Encoder_bias += Encoder_Latest*0.14; //一阶低通滤波器，减缓速度变化 
    Encoder_Integral +=Encoder_bias;    //积分出位移 积分时间：5ms
    Encoder_Integral=Encoder_Integral+Movement; //接收遥控器数据，控制前进后退
    if(Encoder_Integral>10000)  Encoder_Integral=10000; //积分限幅
    if(Encoder_Integral<-10000) Encoder_Integral=-10000;    //积分限幅	
    velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;    //速度PI控制
    if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)    Encoder_Integral=0;    //电机关闭后清除积分
	return velocity;
}
/**
    ****************************************************************************
    *@brief      转向控制
    *@param      Z轴陀螺仪
    *@retval     转向控制PWM
    ****************************************************************************
    */
int Turn(float gyro)
{
    static float Turn_Target,turn,Turn_Amplitude=54;
    float Kp=Turn_Kp,Kd; //修改转向速度，请修改Turn_Amplitude即可
    //===================遥控左右旋转部分=================//
    if(1==Flag_Left)   Turn_Target=-Turn_Amplitude/Flag_velocity;
    else if(1==Flag_Right) Turn_Target=Turn_Amplitude/Flag_velocity;
    else Turn_Target=0;
    
    if(1==Flag_front||1==Flag_back) Kd=Turn_Kd;
    else Kd=0;    //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
    //===================转向PD控制器=================//
    turn=Turn_Target*Kp/100+gyro*Kd/100; //结合Z轴陀螺仪进行PD控制
    return turn;   //转向环PWM右转为正，左转为负
}
/**
    ****************************************************************************
    *@brief      绝对值函数
    *@param      a：需要计算绝对值的数
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
    *@brief      赋值给PWM寄存器
    *@param      左轮PWM、右轮PWM
    *@retval     无
    ****************************************************************************
    */
void Set_Pwm(int motor_left,int motor_right)
{
    if(motor_left>0)    BIN1=1, BIN2=0; //前进 
	else    BIN1=0, BIN2=1; //后退
        PWMB=myabs(motor_left);	
    if(motor_right>0)    AIN2=1,AIN1=0; //前进
    else    AIN2=0, AIN1=1; //后退
        PWMA=myabs(motor_right);
}
/**
    ****************************************************************************
    *@brief      限制PWM幅值
    *@param      IN：输入参数  max：限幅最大值  min：限幅最小值
    *@retval     限幅后的值
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
    *@brief      异常关闭电机
    *@param      angle：小车倾角；voltage：电压
    *@retval     1：异常  0：正常
    ****************************************************************************
    */
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1110)//电池电压低于11.1V关闭电机
	{   //倾角大于40度关闭电机
		temp=1;
		AIN1=0; AIN2=0; BIN1=0; BIN2=0;
	}
	else
		temp=0;
	return temp;			
}
