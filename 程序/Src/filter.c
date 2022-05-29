/**
    ****************************************************************************
    *@file       filter.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      卡尔曼滤波
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "filter.h"

/**
    ****************************************************************************
    *@brief      获取x轴角度简易卡尔曼滤波
    *@param      Accel：加速度计获取的角度、Gyro：陀螺仪获取的角速度
    *@retval     x轴角度
    ****************************************************************************
    */
float dt=0.005; //每5ms进行一次滤波
float Kalman_Filter_x(float Accel,float Gyro)		
{
//	static float angle_dot; 角度微分=角速度，本系统用不到
	static float angle;
	float Q_angle=0.001;    // 过程噪声的协方差
	float Q_gyro=0.003; //过程噪声的协方差
	float R_angle=0.5;  //测量噪声的协方差,值越大表示越不信任测量值
	char  C_0 = 1;  //在此程序中显得多余
	static float Q_bias, Angle_err; //陀螺仪静态漂移
	static float PCt_0, PCt_1, E;   //E:卡尔曼增益分母
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };  //PP为协方差矩阵Pk-的先验估计值
    
/************* 角度先验估计 ****************///Xk = A Xk-1 + B Uk-1
    angle += (Gyro - Q_bias) * dt;

/************* 先验估计误差协方差 ****************///Pk- =A Pk-1 AT + Q
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
/************* 计算卡尔曼增益 ****************///Kk = Pk- HT / (H Pk- HT + R)
	PCt_0 = C_0 * PP[0][0]; //显得多余
	PCt_1 = C_0 * PP[1][0];	//显得多余
	E = R_angle + C_0 * PCt_0;  //求卡尔曼增益分母，直接E = R_angle + PP[0][0]就行
	K_0 = PCt_0 / E;    //计算卡尔曼增益，为两行一列的矩阵
	K_1 = PCt_1 / E;
/************* 角度后验估计 ****************///Xk = Xk- + Kk (Zk - H Xk-)
    Angle_err = Accel - angle;  //(Zk - H Xk-)
	angle += K_0 * Angle_err; //后验估计
	Q_bias += K_1 * Angle_err; //后验估计
//	angle_dot = Gyro - Q_bias;    //输出值(后验估计)的微分=角速度

/************* 更新协方差矩阵 ****************///Pk = (I - Kk H) Pk-
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;  //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
    
	return angle;
}
/**
    ****************************************************************************
    *@brief      获取y轴角度简易卡尔曼滤波，这个函数跟上个完全一样，显得多余
    *@param      加速度计获取的角度、陀螺仪获取的角速度
    *@retval     y轴角度
    ****************************************************************************
    */
float Kalman_Filter_y(float Accel,float Gyro)		
{
	static float angle;
	float Q_angle=0.001;    // 过程噪声的协方差
	float Q_gyro=0.003; // 过程噪声的协方差 过程噪声的协方差为一个 一行两列矩阵
	float R_angle=0.5;  // 测量噪声的协方差 既测量偏差
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    
	angle+=(Gyro - Q_bias) * dt;    //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0];  // Pk-先验估计误差协方差的微分
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-先验估计
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;  //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	angle	+= K_0 * Angle_err; //后验估计
	Q_bias	+= K_1 * Angle_err; //后验估计
	return angle;
}
