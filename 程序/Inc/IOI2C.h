#ifndef __IOI2C_H
#define __IOI2C_H

#include "sys.h"

//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

//IO��������	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //����SDA 

//IIC���в�������
void IIC_Init(void);           //��ʼ��IIC��IO��				 
int IIC_Start(void);					 //����IIC��ʼ�ź�
void IIC_Stop(void);	  			 //����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);		 //IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
int IIC_Wait_Ack(void); 			 //IIC�ȴ�ACK�ź�
void IIC_Ack(void);						 //IIC����ACK�ź�
void IIC_NAck(void);					 //IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 

u8 I2C_ReadOneByte(u8 I2C_Addr,u8 addr);
u8 IICwriteByte(u8 dev, u8 reg, u8 data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
