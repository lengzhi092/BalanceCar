#ifndef __CONTROL_H
#define __CONTROL_H

#include "sys.h"
/********����˿�********/
#define INT PAin(12)    //PA12���ӵ�MPU6050���ж�����

/********���峣��********/
#define PWMB   TIM1->CCR4  //PA11
#define BIN2   PBout(12)
#define BIN1   PBout(13)
#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define PWMA   TIM1->CCR1  //PA8

#define Middle_angle 0
#define Control_Frequency  200.0    //��������ȡƵ��
#define PI 3.14159265
#define Diameter_67  67.0   //����ֱ��67mm 
#define EncoderMultiples   4.0  //��������Ƶ��
#define Encoder_precision  13.0 //���������� 13��
#define Reduction_Ratio  30.0   //���ٱ�30

/********��������********/
u32 myabs(int a);
int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);

void Set_Pwm(int motor_left,int motor_right);

int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);

#endif
