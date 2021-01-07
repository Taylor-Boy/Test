/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define WK_UP_Pin GPIO_PIN_13
#define WK_UP_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOE
#define KEY2_EXTI_Pin GPIO_PIN_8
#define KEY2_EXTI_GPIO_Port GPIOD
#define KEY2_EXTI_EXTI_IRQn EXTI9_5_IRQn
#define KEY1_EXTI_Pin GPIO_PIN_9
#define KEY1_EXTI_GPIO_Port GPIOD
#define KEY1_EXTI_EXTI_IRQn EXTI9_5_IRQn
#define KEY0_EXTI_Pin GPIO_PIN_10
#define KEY0_EXTI_GPIO_Port GPIOD
#define KEY0_EXTI_EXTI_IRQn EXTI15_10_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define Key0State	HAL_GPIO_ReadPin(KEY0_EXTI_GPIO_Port,KEY0_EXTI_Pin) 
#define Key1State	HAL_GPIO_ReadPin(KEY1_EXTI_GPIO_Port,KEY1_EXTI_Pin)
#define Key2State	HAL_GPIO_ReadPin(KEY2_EXTI_GPIO_Port,KEY2_EXTI_Pin)
#define LED_R_ON	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,1)
#define LED_R_OFF	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,0)
#define LED_G_ON	HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,1)
#define LED_G_OFF	HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,0)
#define LED_B_ON	HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,1)
#define LED_B_OFF	HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,0)

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
