/**
@file       motor.h
@brief      定义与电机控制有关的函数接口
@author     Zev
*/

/**
@addtogroup motor
@brief      电机驱动模块
@{
*/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

#define TIM1_PERIOD (0xffff - 1)   ///<定时器1的计数周期，与定时器8的计数周期一致，两个定时器都配置为编码器模式
#define TIM5_PERIOD (1000 - 1)   ///<定时器5的计时周期，定时器5控制电机的转动速度
#define MOTOR_0 ((uint16_t)0x0001) ///<代表右边的电机
#define MOTOR_1 ((uint16_t)0x0002) ///<代表左边的电机
#define POINT_MAX 2500             ///<电机全速运行时在采样时间内100ms能够达到的最大脉冲数
#define MAX_SPEED 23               ///<电机能够达到的最高速度，单位为dm/s

typedef enum { MOTOR_DIRECTION_FORWARD,
               MOTOR_DIRECTION_STOP,
               MOTOR_DIRECTION_BACKWARD,
               MOTOR_DIRECTION_KEEP } MOTOR_DIRECTION; ///<电机的转动控制

/**
@brief      配置电机控制所需的时钟，GPIO口，定时器，中断
@param      None
@retval     None
*/
void motor_config(void);
/**
@brief      设定左右电机的速度
@param      speed0 设定右电机的速度
            speed1 设定左电机的速度
@retval     None
@note       速度单位为dm/s，速度的正负可以表示方向
*/
void motor_set_speed(float speed0, float speed1);
/**
@brief      设定电机的转动方向
@param      dir 可选MOTOR_DIRECTION_FORWARD,MOTOR_DIRECTION_STOP,MOTOR_DIRECTION_BACKWARD,MOTOR_DIRECTION_KEEP
            motor 可选MOTOR_0，MOTOR_1或者两者的组合MOTOR_0|MOTOR_1
@retval     None
*/
void motor_set_direction(MOTOR_DIRECTION dir, uint16_t motor);
/**
@brief      获得单个电机的转动方向
@param      motor 可选MOTOR_0，MOTOR_1
@retval     返回电机motor的转动方向
*/
MOTOR_DIRECTION motor_get_direction(uint16_t motor);
#endif
/**
@}
*/
