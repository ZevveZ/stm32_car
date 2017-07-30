/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
/**
@brief  This function handles SysTick Handler.
@param  None
@retval None
*/
void SysTick_Handler(void);
/**
@brief		记录定时器1的溢出计数
@param		None
@retval 	None
*/
void TIM1_UP_TIM10_IRQHandler(void);
/**
@brief		记录定时器8的溢出计数
@param		None
@retval 	None
*/
void TIM8_UP_TIM13_IRQHandler(void);
/**
@brief		定时器2每隔100ms会触发此中断进行PID控制，因此PID的采样周期为100ms
@param		None
@retval 	None
*/
void TIM1_CC_IRQHandler(void);
/**
@brief		用于调试模糊决策算法的距离参数，包括安全阈值，障碍物距离远，障碍物距离近，模糊逻辑的作用时间，模糊逻辑算法的时间变化量，速度快的型心，速度慢的型心（参考论文模糊逻辑算法）
@param		None
@retval 	None
@note		通过调整上述的参数可以优化小车的避障效果；距离单位为m，时间单位为ms，速度单位为dm/s
*/
void USART2_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
