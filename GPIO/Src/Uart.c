#include "Uart.h"
#include <string.h>
/*==========================UART DMA 不定长接收实现思路=========================
1.初始化阶段启动串口中断接收模式 HAL_UART_Receive_IT(*huart,*RxBuf,Size)
  此处接收字节长度Size 设为 1
2.当收到一个字节的数据时,清除接收中断，开启IDLE空闲中断 开启DMA接收（DMA地址+1）
3.产生IDLE空闲中断时，清除标志位，停止DMA接收，接收数据转存（处理），开启DMA传输

说明：IDLE中断只在接收完一帧数据时产生
==============================================================================*/
/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern unsigned char RX1_Buf[];
extern unsigned char RX2_Buf[];

uint8_t Buftemp;
 
uint16_t usart1_recv_len,data_length;
uint8_t USART1RECV[MAX_RCV_LEN];//USART1接收缓存区

/* USART1 init function */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	HAL_UART_Receive_IT(&huart1, (uint8_t *)(&Buftemp), 1);//开启串口中断接收模式
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		if(usart1_recv_len == 0)
		{			
			USART1RECV[usart1_recv_len++]=Buftemp;//第一次中断的数据被写入USART1RECV[0]处
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能IDLE中断
			HAL_UART_Receive_DMA(&huart1,USART1RECV+1,MAX_RCV_LEN);//打开DMA接收	
	  }
	}
	if(huart == &huart2)
	{
		HAL_UART_Receive_IT(&huart2, RX2_Buf, 1);
		printf("%s",RX2_Buf);
	}
}
void USART_IDLECallBack(void)
{
	unsigned int temp;

  __HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
  temp = USART1->RDR;  //清除状态寄存器RDR
	temp = temp;
  HAL_UART_DMAStop(&huart1); //停止传输	
	//获取DMA接收数据长度
	data_length = MAX_RCV_LEN - hdma_usart1_rx.Instance->CNDTR;
	//数据转存
	memcpy(RX1_Buf,USART1RECV,data_length);
}
void GetRcvData(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t rcv_len)
{ 
	if(huart->Instance == USART1)
	{	
		printf("%d\r\n",buf[0]);
		if(buf[0])
		{
			memcpy(buf,USART1RECV, rcv_len);
		}
		USART_Clear(&huart1);
	}
	else if(huart->Instance == USART2)
	{
		
	}
}
//串口数据接收完成后，清除串口接收缓冲区及接收数据长度
void USART_Clear(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		usart1_recv_len = 0;
//		printf("strlen(USART1RECV) %d\r\n",strlen(USART1RECV));
//		printf("sizeof(USART1RECV) %d\r\n",sizeof(USART1RECV));
		memset(USART1RECV, 0x0, sizeof(USART1RECV));
	}
	else if(huart->Instance == USART2)
	{
		
	}
}
/* USART1 init function */
/*
//void MX_USART1_UART_Init(void)
//{

//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//使能IDLE中断
//	HAL_UART_Receive_DMA(&huart1,RX1_DMABuf,RX1_DMALen);//打开DMA接收，数据存入RX1_DMABuf中
//}
*/
/*
// USART2 init function
//void MX_USART2_UART_Init(void)
//{
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//}
*/
/*
void UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
	uint32_t temp;
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET)
	{
			//__HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_IDLEF);
		  __HAL_UART_CLEAR_IDLEFLAG(huart);	//清除IDLE中断标志位
			temp = huart->Instance->ISR;			//清除状态寄存器SR，读取SR寄存器可以实现清除SR寄存器功能
			temp = huart->Instance->RDR;			//读取数据寄存器中的数据
			temp = temp;
		  HAL_UART_DMAStop(huart);					
			
			if(huart->Instance == USART1)
			{
				DMA_Usart1_RxSize = RX1_DMALen - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//获取DMA单元中的数据
				printf("DMA_Len :%x__%x\r\n",DMA_Usart1_RxSize,__HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
				if(DMA_Usart1_RxSize >1 && RxBuf_Lock1 == 0)
				{
					//RX1_Buf串口最终接收缓存区  RX1_DMABuf DMA接收暂存区
					memcpy(RX1_Buf + RxBufSize1,RX1_DMABuf,DMA_Usart1_RxSize);
					RxBufSize1 += DMA_Usart1_RxSize;
				}
				HAL_UART_Receive_DMA(&huart1,RX1_DMABuf,RxBufSize1);
			}	
	}
}
uint8_t Uart_GetRxSize(UART_HandleTypeDef *huart,uint8_t *buf)
{
	uint8_t Size = 0;
	if(huart->Instance == USART1)
	{
		RxBuf_Lock1 = 1;  
		if(RxBufSize1 > 0)
		{
			Size = RxBufSize1;
			RX1_Buf[RxBufSize1] = 0;
			memcpy(buf,RX1_Buf,RxBufSize1);
			RxBufSize1 = 0;
		}
		RxBuf_Lock1 = 0;
		//printf("接收了 %d 字节\r\n",Size);
		
		return Size;
	}

}
*/