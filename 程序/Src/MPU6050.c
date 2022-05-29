/**
    ****************************************************************************
    *@file       MPU6050.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3
    *@brief      姿态传感器模块驱动
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
#define DEFAULT_MPU_HZ  (200)   //200Hz,数据更新周期5ms，即5ms会触发一次外部中断
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30 1073741824.0f
short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw,Accel_Angle_x;   //俯仰，横滚，偏航（绕z轴转动）
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
    *@brief      设置  MPU6050 的时钟源
    *@param      source：时钟源编号
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
    *@retval     无
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
    *@brief      设置 MPU6050 加速度计的最大量程
    *@param      range：MPU6050_ACCEL_FS_2为+-2G   4、8、16以此类推
    *@retval     无
    ****************************************************************************
    */
void MPU6050_setFullScaleAccelRange(uint8_t range)
{
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**
    ****************************************************************************
    *@brief      设置 MPU6050 是否进入睡眠模式
    *@param      enable：1，睡觉；0，工作
    *@retval     无
    ****************************************************************************
    */
void MPU6050_setSleepEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
/**
    ****************************************************************************
    *@brief      读取  MPU6050 WHO_AM_I 标识
    *@param      无
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
    *@brief      检测MPU6050 是否已经连接
    *@param      无
    *@retval     1：已连接；0：未连接
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
    *@brief      设置 MPU6050 是否为AUX I2C线的主机
    *@param      enable：1，是；0：否
    *@retval     无
    ****************************************************************************
    */
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
/**
    ****************************************************************************
    *@brief      设置 MPU6050 是否为AUX I2C线的主机
    *@param      enable：1，是；0：否
    *@retval     无
    ****************************************************************************
    */
void MPU6050_setI2CBypassEnabled(uint8_t enabled)
{
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/**
    ****************************************************************************
    *@brief      初始化	MPU6050 以进入可用状态
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void MPU6050_initialize(void)
{
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO);    //设置时钟
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);    //陀螺仪量程设置
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2); //加速度度最大量程 +-2G
    MPU6050_setSleepEnabled(0); //进入工作状态
    MPU6050_setI2CMasterModeEnabled(0);   //不让MPU6050 控制AUXI2C
    MPU6050_setI2CBypassEnabled(0);   //主控制器的I2C与	MPU6050的AUXI2C	直通关闭
}
/**************************************************************************
Function: Initialization of DMP in mpu6050
Input   : none
Output  : none
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
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
    *@brief      调用卡尔曼滤波 获取角度
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void Get_Angle(void)
{
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
//	Temperature=Read_Temperature(); //读取MPU6050内置温度传感器数据，近似表示主板温度。
    Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
    Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
    Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
    Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
    Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
    Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
    if(Gyro_X>32768)  Gyro_X-=65536;    //数据类型转换  也可通过short强制类型转换
    if(Gyro_Y>32768)  Gyro_Y-=65536;    //数据类型转换  也可通过short强制类型转换
    if(Gyro_Z>32768)  Gyro_Z-=65536;    //数据类型转换
    if(Accel_X>32768) Accel_X-=65536;   //数据类型转换
    if(Accel_Y>32768) Accel_Y-=65536;   //数据类型转换
    if(Accel_Z>32768) Accel_Z-=65536;   //数据类型转换
    Gyro_Balance=-Gyro_X;   //更新平衡角速度
    Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;    //计算倾角，转换单位为度	
    Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;    //计算倾角，转换单位为度
    Gyro_X=Gyro_X/16.4; //陀螺仪量程转换,量程±2000°/s,寄存器范围-32768~32767
    Gyro_Y=Gyro_Y/16.4; //陀螺仪量程转换,对应灵敏度(0.5*2^16)/2000=16.4
    
    Pitch = -Kalman_Filter_x(Accel_Angle_x,Gyro_X); //卡尔曼滤波
    Roll = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
    Angle_Balance=Pitch;    //更新平衡倾角
    Gyro_Turn=Gyro_Z;   //更新转向角速度
    Acceleration_Z=Accel_Z; //更新Z轴加速度计	
}
/**
    ****************************************************************************
    *@brief      读取MPU6050内置温度传感器数据
    *@param      无
    *@retval     摄氏温度
    ****************************************************************************
    */
//int Read_Temperature(void)
//{
//    float Temp;
//    Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
//    if(Temp>32768) Temp-=65536; //数据类型转换
//    Temp=(36.53+Temp/340)*10;   //温度放大十倍存放
//    return (int)Temp;
//}
