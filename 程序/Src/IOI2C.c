/**
    ****************************************************************************
    *@file       IOI2C.c
    *@auther     wxf
    *@version    V1.0.0
    *@date       2022.3.17
    *@brief      模拟I2C通讯
    ****************************************************************************
    *@attension  
    *
    ****************************************************************************
    */
#include "ioi2c.h"
#include "delay.h"

/**
    ****************************************************************************
    *@brief      模拟IIC起始信号
    *@param      无
    *@retval     1
    ****************************************************************************
    */
int IIC_Start(void)
{
	SDA_OUT();  //sda线输出
	IIC_SDA=1;
	if(!READ_SDA)   return 0;
	IIC_SCL=1;
	delay_us(1);
 	IIC_SDA=0;  //START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	delay_us(1);
	IIC_SCL=0;  //钳住I2C总线，准备发送或接收数据 
	return 1;
}
/**
    ****************************************************************************
    *@brief      模拟IIC结束信号
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void IIC_Stop(void)
{
	SDA_OUT();  //sda线输出
	IIC_SCL=0;
	IIC_SDA=0;  //STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL=1;
	IIC_SDA=1;  //发送I2C总线结束信号
	delay_us(1);
}
/**
    ****************************************************************************
    *@brief      IIC等待应答信号
    *@param      无
    *@retval     0：没有收到应答；1：收到应答
    ****************************************************************************
    */
int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();   //SDA设置为输入
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
	IIC_SCL=0;  //时钟输出0
	return 1;
}
/**
    ****************************************************************************
    *@brief      IIC应答
    *@param      无
    *@retval     无
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
    *@brief      IIC不应答
    *@param      无
    *@retval     无
    ****************************************************************************
    */
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;  //应答与否的不同就在这
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}
/**
    ****************************************************************************
    *@brief      IIC发送一个字节
    *@param      txd：发送的字节数据
    *@retval     无
    ****************************************************************************
    */
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
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
    *@brief      IIC写数据到寄存器
    *@param      addr：设备地址；reg：寄存器地址；len;字节数；data：数据
    *@retval     0：成功写入；1：没有成功写入
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
    *@brief      IIC读寄存器的数据
    *@param      addr：设备地址；reg：寄存器地址；len;字节数；*buf：读出数据缓存
    *@retval     0：成功读出；1：没有成功读出
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
    *@brief      IIC读取一个位
    *@param      ack：是否发送应答信号；1：发送；0：不发送
    *@retval     receive：读取的数据
    ****************************************************************************
    */
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();   //SDA设置为输入
    for(i=0;i<8;i++ )
	 {
        IIC_SCL=0;
        delay_us(2);
        IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)    receive++;
        delay_us(2);
    }
    if (ack)    IIC_Ack();  //发送ACK
    else    IIC_NAck(); //发送nACK
    return receive;
}
/**
    ****************************************************************************
    *@brief      读取指定设备指定寄存器的一个值
    *@param      I2C_Addr：设备IIC地址；addr:寄存器地址
    *@retval     res：读取的数据
    ****************************************************************************
    */
u8 I2C_ReadOneByte(u8 I2C_Addr,u8 addr)
{
	unsigned char res=0;
	IIC_Start();
	IIC_Send_Byte(I2C_Addr);    //发送写命令
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //发送地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;   //进入接收模式			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);
    IIC_Stop(); //产生一个停止条件
	return res;
}
/**
    ****************************************************************************
    *@brief      IIC连续读数据
    *@param      dev：目标设备IIC地址；reg:寄存器地址；length：字节数；
				 *data:读出的数据将要存放的指针
    *@retval     count：读出来的字节数量-1
    ****************************************************************************
    */
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev); //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //发送地址
    IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(dev+1);   //进入接收模式
	IIC_Wait_Ack();
    for(count=0;count<length;count++)
    {
        if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //带ACK的读数据
		 else   data[count]=IIC_Read_Byte(0);  //最后一个字节NACK
	}
    IIC_Stop(); //产生一个停止条件
    return count;
}
/**
    ****************************************************************************
    *@brief      将多个字节写入指定设备指定寄存器
    *@param      dev：目标设备地址；reg：寄存器地址；length：要写的字节数；
                 *data：将要写的数据的首地址
    *@retval     1：写入成功
    ****************************************************************************
    */
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev); //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //发送地址
    IIC_Wait_Ack();
	for(count=0; count<length; count++)
    {
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	 }
	IIC_Stop(); //产生一个停止条件
    return 1;   //status == 0;
}
/**
    ****************************************************************************
    *@brief      读取指定设备指定寄存器的一个值
    *@param      dev：目标设备地址；reg：寄存器地址；*data：将要写的数据的首地址
    *@retval     1:读取成功
    ****************************************************************************
    */
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}
/**
    ****************************************************************************
    *@brief      写入指定设备指定寄存器一个字节
    *@param      dev：目标设备地址；reg：寄存器地址；data：将要写的数据
    *@retval     1：写入成功
    ****************************************************************************
    */
u8 IICwriteByte(u8 dev, u8 reg, u8 data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}
/**
    ****************************************************************************
    *@brief      读、修改、写指定设备 指定寄存器一个字节 中的多个位
    *@param      dev：目标设备地址；reg：寄存器地址；bitStart：目标字节的起始位；
				 data：存放改变目标字节位的值
    *@retval     1：成功；0：失败
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
    *@brief      读、修改、写指定设备 指定寄存器一个字节 中的1个位
    *@param      dev：目标设备地址；reg：寄存器地址；bitNum：要修改目标字节的bitNum位；
				 data：为0时，目标位将被清，否则将被置位
    *@retval     1：成功；0：失败
    ****************************************************************************
    */
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data!=0) ? (b|(1<<bitNum)) : (b&~(1<<bitNum));
    return IICwriteByte(dev, reg, b);
}
