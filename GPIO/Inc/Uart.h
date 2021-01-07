#ifndef __UART_H
#define __UART_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include <stdio.h>
/* Private define ------------------------------------------------------------*/
#define RX1_DMALen 255
#define RX2_DMALen 255
#define MAX_RCV_LEN 1024
/* Private function prototypes -----------------------------------------------*/
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);//串口接收中断回调函数
void UART_IDLE_Callback(UART_HandleTypeDef *huart);//接收空闲中断回调函数
void USART_Clear(UART_HandleTypeDef *huart);//串口接收数据缓冲区数据清零
void GetRcvData(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t rcv_len);//接收数据转存
#endif
