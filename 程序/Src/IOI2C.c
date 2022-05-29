/**
    ****************************************************************************
    *@file       IOI2C.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.17
    *@brief      ģ��I2CͨѶ
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "ioi2c.h"
#include "delay.h"

/**
    ****************************************************************************
    *@brief      ģ��IIC��ʼ�ź�
    *@param      ��
    *@retval     1
    ****************************************************************************
    */
int IIC_Start(void)
{
	SDA_OUT();  //sda�����
	IIC_SDA=1;
	if(!READ_SDA)   return 0;
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0;  //START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;  //ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}
/**
    ****************************************************************************
    *@brief      ģ��IIC�����ź�
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void IIC_Stop(void)
{
	SDA_OUT();  //sda�����
	IIC_SCL=0;
	IIC_SDA=0;  //STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1;
	IIC_SDA=1;  //����I2C���߽����ź�
	delay_us(1);
}
/**
    ****************************************************************************
    *@brief      IIC�ȴ�Ӧ���ź�
    *@param      ��
    *@retval     0��û���յ�Ӧ��1���յ�Ӧ��
    ****************************************************************************
    */
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();   //SDA����Ϊ����
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL=0;  //ʱ�����0
	return 1;
}
/**
    ****************************************************************************
    *@brief      IICӦ��
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**
    ****************************************************************************
    *@brief      IIC��Ӧ��
    *@param      ��
    *@retval     ��
    ****************************************************************************
    */
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;  //Ӧ�����Ĳ�ͬ������
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**
    ****************************************************************************
    *@brief      IIC����һ���ֽ�
    *@param      txd�����͵��ֽ�����
    *@retval     ��
    ****************************************************************************
    */
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
			IIC_SDA=(txd&0x80)>>7;
			txd<<=1;
			delay_us(1);
			IIC_SCL=1;
			delay_us(1);
			IIC_SCL=0;
			delay_us(1);
    }
}
/**
    ****************************************************************************
    *@brief      IICд���ݵ��Ĵ���
    *@param      addr���豸��ַ��reg���Ĵ�����ַ��len;�ֽ�����data������
    *@retval     0���ɹ�д�룻1��û�гɹ�д��
    ****************************************************************************
    */
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i;
    if (!IIC_Start())   return 1;
    IIC_Send_Byte(addr << 1 );
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack())
        {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**
    ****************************************************************************
    *@brief      IIC���Ĵ���������
    *@param      addr���豸��ַ��reg���Ĵ�����ַ��len;�ֽ�����*buf���������ݻ���
    *@retval     0���ɹ�������1��û�гɹ�����
    ****************************************************************************
    */
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!IIC_Start())   return 1;
    IIC_Send_Byte(addr << 1);
    if (!IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1)+1);
    IIC_Wait_Ack();
    while (len)
    {
        if (len == 1)    *buf = IIC_Read_Byte(0);
        else    *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    IIC_Stop();
    return 0;
}
/**
    ****************************************************************************
    *@brief      IIC��ȡһ��λ
    *@param      ack���Ƿ���Ӧ���źţ�1�����ͣ�0��������
    *@retval     receive����ȡ������
    ****************************************************************************
    */
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();   //SDA����Ϊ����
    for(i=0;i<8;i++ )
	 {
        IIC_SCL=0;
        delay_us(2);
        IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)    receive++;
        delay_us(2);
    }
    if (ack)    IIC_Ack();  //����ACK
    else    IIC_NAck(); //����nACK
    return receive;
}
/**
    ****************************************************************************
    *@brief      ��ȡָ���豸ָ���Ĵ�����һ��ֵ
    *@param      I2C_Addr���豸IIC��ַ��addr:�Ĵ�����ַ
    *@retval     res����ȡ������
    ****************************************************************************
    */
u8 I2C_ReadOneByte(u8 I2C_Addr,u8 addr)
{
	unsigned char res=0;
	IIC_Start();
	IIC_Send_Byte(I2C_Addr);    //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;   //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);
    IIC_Stop(); //����һ��ֹͣ����
	return res;
}
/**
    ****************************************************************************
    *@brief      IIC����������
    *@param      dev��Ŀ���豸IIC��ַ��reg:�Ĵ�����ַ��length���ֽ�����
				 *data:���������ݽ�Ҫ��ŵ�ָ��
    *@retval     count�����������ֽ�����-1
    ****************************************************************************
    */
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev); //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //���͵�ַ
    IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(dev+1);   //�������ģʽ
	IIC_Wait_Ack();
    for(count=0;count<length;count++)
    {
        if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 else   data[count]=IIC_Read_Byte(0);  //���һ���ֽ�NACK
	}
    IIC_Stop(); //����һ��ֹͣ����
    return count;
}
/**
    ****************************************************************************
    *@brief      ������ֽ�д��ָ���豸ָ���Ĵ���
    *@param      dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��length��Ҫд���ֽ�����
                 *data����Ҫд�����ݵ��׵�ַ
    *@retval     1��д��ɹ�
    ****************************************************************************
    */
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev); //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //���͵�ַ
    IIC_Wait_Ack();
	for(count=0; count<length; count++)
    {
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	 }
	IIC_Stop(); //����һ��ֹͣ����
    return 1;   //status == 0;
}
/**
    ****************************************************************************
    *@brief      ��ȡָ���豸ָ���Ĵ�����һ��ֵ
    *@param      dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��*data����Ҫд�����ݵ��׵�ַ
    *@retval     1:��ȡ�ɹ�
    ****************************************************************************
    */
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}
/**
    ****************************************************************************
    *@brief      д��ָ���豸ָ���Ĵ���һ���ֽ�
    *@param      dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��data����Ҫд������
    *@retval     1��д��ɹ�
    ****************************************************************************
    */
u8 IICwriteByte(u8 dev, u8 reg, u8 data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}
/**
    ****************************************************************************
    *@brief      �����޸ġ�дָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
    *@param      dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��bitStart��Ŀ���ֽڵ���ʼλ��
				 data����Ÿı�Ŀ���ֽ�λ��ֵ
    *@retval     1���ɹ���0��ʧ��
    ****************************************************************************
    */
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
    u8 b;
    if (IICreadByte(dev, reg, &b) != 0)
    {
        u8 mask = (0xFF<<(bitStart+1)) | 0xFF>>((8-bitStart)+length-1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    }
    else    return 0;
}
/**
    ****************************************************************************
    *@brief      �����޸ġ�дָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
    *@param      dev��Ŀ���豸��ַ��reg���Ĵ�����ַ��bitNum��Ҫ�޸�Ŀ���ֽڵ�bitNumλ��
				 data��Ϊ0ʱ��Ŀ��λ�����壬���򽫱���λ
    *@retval     1���ɹ���0��ʧ��
    ****************************************************************************
    */
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data!=0) ? (b|(1<<bitNum)) : (b&~(1<<bitNum));
    return IICwriteByte(dev, reg, b);
}
