/**
    ****************************************************************************
    *@file       filter.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      �������˲�
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "filter.h"

/**
    ****************************************************************************
    *@brief      ��ȡx��Ƕȼ��׿������˲�
    *@param      Accel�����ٶȼƻ�ȡ�ĽǶȡ�Gyro�������ǻ�ȡ�Ľ��ٶ�
    *@retval     x��Ƕ�
    ****************************************************************************
    */
float dt=0.005; //ÿ5ms����һ���˲�
float Kalman_Filter_x(float Accel,float Gyro)		
{
//	static float angle_dot; �Ƕ�΢��=���ٶȣ���ϵͳ�ò���
	static float angle;
	float Q_angle=0.001;    // ����������Э����
	float Q_gyro=0.003; //����������Э����
	float R_angle=0.5;  //����������Э����,ֵԽ���ʾԽ�����β���ֵ
	char  C_0 = 1;  //�ڴ˳������Եö���
	static float Q_bias, Angle_err; //�����Ǿ�̬Ư��
	static float PCt_0, PCt_1, E;   //E:�����������ĸ
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };  //PPΪЭ�������Pk-���������ֵ
    
/************* �Ƕ�������� ****************///Xk = A Xk-1 + B Uk-1
    angle += (Gyro - Q_bias) * dt;

/************* ����������Э���� ****************///Pk- =A Pk-1 AT + Q
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
/************* ���㿨�������� ****************///Kk = Pk- HT / (H Pk- HT + R)
	PCt_0 = C_0 * PP[0][0]; //�Եö���
	PCt_1 = C_0 * PP[1][0];	//�Եö���
	E = R_angle + C_0 * PCt_0;  //�󿨶��������ĸ��ֱ��E = R_angle + PP[0][0]����
	K_0 = PCt_0 / E;    //���㿨�������棬Ϊ����һ�еľ���
	K_1 = PCt_1 / E;
/************* �ǶȺ������ ****************///Xk = Xk- + Kk (Zk - H Xk-)
    Angle_err = Accel - angle;  //(Zk - H Xk-)
	angle += K_0 * Angle_err; //�������
	Q_bias += K_1 * Angle_err; //�������
//	angle_dot = Gyro - Q_bias;    //���ֵ(�������)��΢��=���ٶ�

/************* ����Э������� ****************///Pk = (I - Kk H) Pk-
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;  //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
    
	return angle;
}
/**
    ****************************************************************************
    *@brief      ��ȡy��Ƕȼ��׿������˲�������������ϸ���ȫһ�����Եö���
    *@param      ���ٶȼƻ�ȡ�ĽǶȡ������ǻ�ȡ�Ľ��ٶ�
    *@retval     y��Ƕ�
    ****************************************************************************
    */
float Kalman_Filter_y(float Accel,float Gyro)		
{
	static float angle;
	float Q_angle=0.001;    // ����������Э����
	float Q_gyro=0.003; // ����������Э���� ����������Э����Ϊһ�� һ�����о���
	float R_angle=0.5;  // ����������Э���� �Ȳ���ƫ��
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
    
	angle+=(Gyro - Q_bias) * dt;    //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0];  // Pk-����������Э�����΢��
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-�������
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;  //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	angle	+= K_0 * Angle_err; //�������
	Q_bias	+= K_1 * Angle_err; //�������
	return angle;
}
