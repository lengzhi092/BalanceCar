/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
///！！！！！！！！！！！！！！！！！！！！！！！！！！///
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	if(Flag_Show==0)
	{	
	  while((USART3->SR&0X40)==0);//Flag_Show=0  使用串口3   
	  USART3->DR = (u8) ch;      
	}
	else
	{	
		while((USART1->SR&0X40)==0);//Flag_Show!=0  使用串口1   
		USART1->DR = (u8) ch;      
	}	
	return ch;
}

#endif

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf)); //打开串口3接收中断
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = usart1_tx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(usart1_tx_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = usart1_rx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(usart1_rx_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = usart3_tx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(usart3_tx_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = usart3_rx_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(usart3_rx_GPIO_Port, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, usart1_tx_Pin|usart1_rx_Pin);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, usart3_tx_Pin|usart3_rx_Pin);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
u8 Usart3_Receive_buf[1];          //串口3接收中断数据存放的缓冲区
u8 Usart3_Receive;                 //从串口3读取的数据
/**
    ****************************************************************************
    *@brief      串口3接收回调函数
    *@param      Uartx:x为1、2、3
    *@retval     无
    ****************************************************************************
    */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if(UartHandle->Instance == USART3)
	{
        static int uart_receive=0;  //蓝牙接收相关变量
        static u8 Flag_PID,i,j,Receive[50];
        static float Data;
        uart_receive=Usart3_Receive_buf[0];
        Usart3_Receive=uart_receive;
        if(uart_receive==0x59)  Flag_velocity=2;  //"Y"低速挡（默认值）
        if(uart_receive==0x58)  Flag_velocity=1;  //"X"高速档
        if(uart_receive==0x5A)  Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;  //刹车
        else if(uart_receive==0x41) Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;  //前
        else if(uart_receive==0x45) Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;  //后
        else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)
            Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //右
        else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)
            Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //左
        else    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;  //刹车
        if(Usart3_Receive==0x7B)    Flag_PID=1;    //APP参数指令起始位"{"
        if(Usart3_Receive==0x7D)    Flag_PID=2;    //APP参数指令停止位"}"
        if(Flag_PID==1)  //采集数据
        {
            Receive[i]=Usart3_Receive;
            i++;
        }
        if(Flag_PID==2)  //分析数据
        {
            if(Receive[3]==0x50)    PID_Send=1; //"P"
            else if(Receive[1] != 0x23) //不是"#"，接收PID参数
            {
                for(j=i; j>=4; j--)   Data+=(Receive[j-1]-48)*pow(10,i-j);
                switch(Receive[1])
                {
                    case 0x30:  Balance_Kp=Data;break;
                    case 0x31:  Balance_Kd=Data;break;
                    case 0x32:  Velocity_Kp=Data;break;
                    case 0x33:  Velocity_Ki=Data;break;
                    case 0x34:  Turn_Kp=Data;break; 
                    case 0x35:  Turn_Kd=Data;break; 
                    case 0x36:  break;  //预留
                    case 0x37:  break;  //预留
                    case 0x38:  break;  //预留
                }
            }
            Flag_PID=0; i=0;    j=0;    Data=0;
            memset(Receive, 0, sizeof(u8)*50);  //数组清零
        }
        HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf)); //串口3回调函数执行完毕后，需要再次开启接收中断等待接收下次中断
    }
}
/* USER CODE END 1 */
