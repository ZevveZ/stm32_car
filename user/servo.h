/**
@file		servo.h
@brief		定义舵机控制相关的函数接口
@author		Zev
*/
/**
@addtogroup servo
@brief 		舵机驱动模块
@{
*/

#ifndef __SERVO_H
#define __SERVO_H

typedef enum { SERVO_DIRECTION_LEFT = 25,
               SERVO_DIRECTION_FRONT = 15,
               SERVO_DIRECTION_RIGHT = 5 } SERVO_DIRECTION; ///<因为没有具体舵机手册所以这三个数字可能会有些许偏差

/**
@brief		配置舵机控制所需的时钟，GPIO口和定时器,并将舵机转动到正前方
@param		None
@retval 	None
*/
void servo_config(void);
/**
@brief		控制舵机的转动方向
@param		dir 可选SERVO_DIRECTION_LEFT，SERVO_DIRECTION_FRONT，SERVO_DIRECTION_RIGHT
@retval 	None
@note		延时一段时间使舵机转动到位
*/
void servo_turn_to(SERVO_DIRECTION dir);
#endif
/**
@}
*/
