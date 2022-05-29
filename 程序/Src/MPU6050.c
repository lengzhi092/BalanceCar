/**
    ****************************************************************************
    *@file       MPU6050.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      ��̬������ģ������
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "MPU6050.h"
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)   //200Hz,���ݸ�������5ms����5ms�ᴥ��һ���ⲿ�ж�
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30 1073741824.0f
short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw,Accel_Angle_x;   //�����������ƫ������z��ת����
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
uint8_t buffer[14];
int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}
static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		//printf("setting bias succesfully ......\r\n");
    }
}
/**
    ****************************************************************************
    *@brief      ����  MPU6050 ��ʱ��Դ
    *@param      source��ʱ��Դ���
    * CLK_SEL | Clock Source
    * --------+--------------------------------------
    * 0       | Internal oscillator
    * 1       | PLL with X Gyro reference
    * 2       | PLL with Y Gyro reference
    * 3       | PLL with Z Gyro reference
    * 4       | PLL with external 32.768kHz reference
    * 5       | PLL with external 19.2MHz reference
    * 6       | Reserved
    * 7       | Stops the clock and keeps the timing generator in reset
    *@retval     ��
    ****************************************************************************
    */
void MPU6050_setClockSource(uint8_t source)
{
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}
/**
    ****************************************************************************
    *Set full-scale gyroscope range.
    * @param range New full-scale gyroscope range value
    * @see getFullScaleRange()
    * @see MPU6050_GYRO_FS_250
    * @see MPU6050_RA_GYRO_CONFIG
    * @see MPU6050_GCONFIG_FS_SEL_BIT
    * @see MPU6050_GCONFIG_FS_SEL_LENGTH  
    ****************************************************************************
    */
void MPU6050_setFullScaleGyroRange(uint8_t range)
{
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}
/**
    ****************************************************************************
    *@brief      ���� MPU6050 ���ٶȼƵ��������
    *@param      range��MPU6050_ACCEL_FS_2Ϊ+-2G   4��8��16�Դ�����
    *@retval     ��
    ****************************************************************************
    */
void MPU6050_setFullScaleAccelRange(uint8_t range)
{
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**
    ****************************************************************************
    *@brief      ���� MPU6050 �Ƿ����˯��ģʽ
    *@param      enable��1��˯����0������
    *@retval     ��
    ****************************************************************************
    */
void MPU6050_setSleepEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
/**
    ****************************************************************************
    *@brief      ��ȡ  MPU6050 WHO_AM_I ��ʶ
    *@param      ��
    *@retval     0x68
    ****************************************************************************
    */
uint8_t MPU6050_getDeviceID(void)
 {
    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}
/**
    ****************************************************************************
    *@brief      ���MPU6050 �Ƿ��Ѿ�����
    *@param      ��
    *@retval     1�������ӣ�0��δ����
    ****************************************************************************
    */
uint8_t MPU6050_testConnection(void)
{
    if(MPU6050_getDeviceID() == 0x68)    //0b01101000;
    return 1;
   	else return 0;
}
/**
    ****************************************************************************
    *@brief      ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
    *@param      enable��1���ǣ�0����
    *@retval     ��
    ****************************************************************************
    */
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
/**
    ****************************************************************************
    *@brief      ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
    *@param      enable��1���ǣ�0����
    *@retval     ��
    ****************************************************************************
    */
void MPU6050_setI2CBypassEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/**
    ****************************************************************************
    *@brief      ��ʼ��	MPU6050 �Խ������״̬
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void MPU6050_initialize(void)
{
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO);    //����ʱ��
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);    //��������������
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2); //���ٶȶ�������� +-2G
    MPU6050_setSleepEnabled(0); //���빤��״̬
    MPU6050_setI2CMasterModeEnabled(0);   //����MPU6050 ����AUXI2C
    MPU6050_setI2CBypassEnabled(0);   //����������I2C��	MPU6050��AUXI2C	ֱͨ�ر�
}
/**************************************************************************
Function: Initialization of DMP in mpu6050
Input   : none
Output  : none
�������ܣ�MPU6050����DMP�ĳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void DMP_Init(void)
{ 
    u8 temp[1]={0};
    Flag_Show=1;
    i2cRead(0x68,0x75,1,temp);
    printf("mpu_set_sensor complete ......\r\n");
    if(temp[0]!=0x68)   NVIC_SystemReset();
    if(!mpu_init())
    {
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_set_sensor complete ......\r\n");
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
            printf("mpu_configure_fifo complete ......\r\n");
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
            printf("mpu_set_sample_rate complete ......\r\n");
        if(!dmp_load_motion_driver_firmware())
            printf("dmp_load_motion_driver_firmware complete ......\r\n");
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
            printf("dmp_set_orientation complete ......\r\n");
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
           DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
           DMP_FEATURE_GYRO_CAL))
            printf("dmp_enable_feature complete ......\r\n");
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
            printf("dmp_set_fifo_rate complete ......\r\n");
            run_self_test();
        if(!mpu_set_dmp_state(1))
            printf("mpu_set_dmp_state complete ......\r\n");
    }
    Flag_Show=0;
}
/**
    ****************************************************************************
    *@brief      ���ÿ������˲� ��ȡ�Ƕ�
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void Get_Angle(void)
{
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
//	Temperature=Read_Temperature(); //��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
    Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
    Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
    Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
    Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
    Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
    Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
    if(Gyro_X>32768)  Gyro_X-=65536;    //��������ת��  Ҳ��ͨ��shortǿ������ת��
    if(Gyro_Y>32768)  Gyro_Y-=65536;    //��������ת��  Ҳ��ͨ��shortǿ������ת��
    if(Gyro_Z>32768)  Gyro_Z-=65536;    //��������ת��
    if(Accel_X>32768) Accel_X-=65536;   //��������ת��
    if(Accel_Y>32768) Accel_Y-=65536;   //��������ת��
    if(Accel_Z>32768) Accel_Z-=65536;   //��������ת��
    Gyro_Balance=-Gyro_X;   //����ƽ����ٶ�
    Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;    //������ǣ�ת����λΪ��	
    Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;    //������ǣ�ת����λΪ��
    Gyro_X=Gyro_X/16.4; //����������ת��,���̡�2000��/s,�Ĵ�����Χ-32768~32767
    Gyro_Y=Gyro_Y/16.4; //����������ת��,��Ӧ������(0.5*2^16)/2000=16.4
    
    Pitch = -Kalman_Filter_x(Accel_Angle_x,Gyro_X); //�������˲�
    Roll = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
    Angle_Balance=Pitch;    //����ƽ�����
    Gyro_Turn=Gyro_Z;   //����ת����ٶ�
    Acceleration_Z=Accel_Z; //����Z����ٶȼ�	
}
/**
    ****************************************************************************
    *@brief      ��ȡMPU6050�����¶ȴ���������
    *@param      ��
    *@retval     �����¶�
    ****************************************************************************
    */
//int Read_Temperature(void)
//{
//    float Temp;
//    Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
//    if(Temp>32768) Temp-=65536; //��������ת��
//    Temp=(36.53+Temp/340)*10;   //�¶ȷŴ�ʮ�����
//    return (int)Temp;
//}
