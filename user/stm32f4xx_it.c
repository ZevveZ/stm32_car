/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "pid.h"
#include "motor.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    timing_delay_decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

static int32_t overflow_cnt0, overflow_cnt1; ///<分别记录定时器1和定时器8的溢出计数
/**
@brief      记录定时器1的溢出计数
@param      None
@retval     None
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update))
    {
        overflow_cnt0++;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
/**
@brief      记录定时器8的溢出计数
@param      None
@retval     None
*/
void TIM8_UP_TIM13_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update))
    {
        overflow_cnt1++;
        TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
    }
}

static int32_t last_cnt0, last_cnt1; ///<分别记录定时器1和定时器8的上一次Counter值
/**
@brief      计算脉冲的变化量
@param      current 当前脉冲计数
            last 上一次脉冲计数
            dir 当前电机的转动方向
@retval     脉冲的变化量
@note       根据电机的转动方向有两种计算方式
*/
int32_t get_cnt_delta(int32_t current, int32_t last, MOTOR_DIRECTION dir)
{
    switch (dir)
    {
    case MOTOR_DIRECTION_FORWARD:
        return current - last;
    case MOTOR_DIRECTION_BACKWARD:
        return last - current;
    default:
        return current - last;
    }
}
/**
@brief      定时器2每隔100ms会触发此中断进行PID控制，因此PID的采样周期为100ms
@param      None
@retval     None
*/
void TIM1_CC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC3))
    {
        int32_t current_cnt0 = TIM_GetCapture3(TIM1); //右边电机的脉冲计数
        int32_t current_cnt1 = TIM_GetCapture3(TIM8); //左边电机的脉冲计数
        int32_t tmp0, tmp1;

        tmp0 = get_cnt_delta(current_cnt0, last_cnt0, motor_get_direction(MOTOR_0)); //右边电机的脉冲变化量
        tmp1 = get_cnt_delta(current_cnt1, last_cnt1, motor_get_direction(MOTOR_1)); //左边电机的脉冲变化量

        last_cnt0 = current_cnt0;
        last_cnt1 = current_cnt1;

        pid_control((TIM1_PERIOD + 1) * overflow_cnt0 + tmp0, (TIM1_PERIOD + 1) * overflow_cnt1 + tmp1); //进行PID控制
        overflow_cnt0 = overflow_cnt1 = 0;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
    }
}

/**
@brief      用于与上位机调试PID，上位机根据绘制的PID图形发送比例系数，积分时间，微分时间给下位机，实时调整参数
@param      None
@retval     None
@note       接收数据时注意大小端问题
*/
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        uint32_t pwm;
        float pd, ti, td;
        MOTOR_DIRECTION dir;
        union {
            float f;
            char s[sizeof(float)];
        } s2f;

        for (int i = 0; i < sizeof(float); ++i)
        {
            while (!USART_GetITStatus(USART2, USART_IT_RXNE))
                ;
            s2f.s[i] = USART_ReceiveData(USART2) & 0xff;
        }
        pwm = s2f.f;
        for (int i = 0; i < sizeof(float); ++i)
        {
            while (!USART_GetITStatus(USART2, USART_IT_RXNE))
                ;
            s2f.s[i] = USART_ReceiveData(USART2) & 0xff;
        }
        pd = s2f.f;
        for (int i = 0; i < sizeof(float); ++i)
        {
            while (!USART_GetITStatus(USART2, USART_IT_RXNE))
                ;
            s2f.s[i] = USART_ReceiveData(USART2) & 0xff;
        }
        ti = s2f.f;
        for (int i = 0; i < sizeof(float); ++i)
        {
            while (!USART_GetITStatus(USART2, USART_IT_RXNE))
                ;
            s2f.s[i] = USART_ReceiveData(USART2) & 0xff;
        }
        td = s2f.f;
        for (int i = 0; i < sizeof(float); ++i)
        {
            while (!USART_GetITStatus(USART2, USART_IT_RXNE))
                ;
            s2f.s[i] = USART_ReceiveData(USART2) & 0xff;
        }
        dir = (MOTOR_DIRECTION)s2f.f;

        //pid_debug(pwm,pd,ti,td,dir);
        //printf("%d,%f,%f,%f,%d\n",pwm,kp,ti,td,action);
    }
}

/**
@brief      用于调试模糊决策算法的距离参数，包括安全阈值，障碍物远的距离，障碍物近的距离，模糊决策算法的时间变化量，模糊决策算法的作用时间，速度快的型心，速度慢的型心（参考论文模糊逻辑算法）
@param      None
@retval     None
@note       通过调整上述的参数可以优化小车的避障效果；距离单位为m，时间单位为ms(模糊逻辑算法的时间变化量的单位为s)，速度单位为dm/s
*/
/*
void USART2_IRQHandler(void){
    if(USART_GetITStatus(USART2,USART_IT_RXNE)){
        char key;
        union{
            float f;
            char s[sizeof(float)];
        }s2f;
        
        key=USART_ReceiveData(USART2);
        for(int i=sizeof(float)-1;i>=0;--i){
            while(!USART_GetITStatus(USART2,USART_IT_RXNE));
            s2f.s[i]=USART_ReceiveData(USART2)&0xff;
        }
        
        switch(key){
        case 1:
            threshold_safe=s2f.f;
            break;
        case 2:
            threshold_barrier_far=s2f.f;
            break;
        case 3:
            threshold_barrier_near=s2f.f;
            break;
        case 4:
            time_fuzzy_delay=s2f.f;
            break;
        case 5:
            time_delta=s2f.f;
            break;
        case 6:
            speed_fast=s2f.f;
            break;
        case 7:
            speed_low=s2f.f;
            break;
        }
    }
}
*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
