/**
@file		main.c
@brief		实现避障小车主要的流程控制
@author		Zev
*/

#include <stdio.h>
#include "motor.h"
#include "pid.h"
#include "usart.h"
#include "sonar.h"
#include "delay.h"
#include "servo.h"
#include "fuzzy_decision.h"

#define USE_ITM 0					///<如果USE_ITM为0通过蓝牙串口发送调试信息；否则通过JLink输出调试信息
#define TIME_FUZZY_DELAY 600	///<模糊逻辑决策的作用时间，单位为毫秒

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

#if USE_ITM
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000
int fputc(int ch, FILE *f) {
	if (DEMCR & TRCENA) {
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return(ch);
}
#else
int fputc(int ch,FILE *f){
	usart_send_char(ch);
	return ch;
}
#endif

/**
@brief		配置小车的各个模块，并实现小车避障的主要流程：获取前方障碍物的距离，如果距离在安全距离以内，那么小车会全速前进；否则小车会停下来检测左边和右边障碍物
			的距离，通过模糊逻辑算法获取小车两个电机需要的转速，并以这样的转速行驶一段时间，再检测前方障碍物的距离，如此循环
@param		None
@retval		返回0表示程序正常结束，否则程序异常退出
*/
int main(void){
	float barrier_distance_left=0,barrier_distance_front=0,barrier_distance_right=0;	//分别为左边，前方，右边障碍物的距离，单位为米
	FUZZY_ST fst;	//存放模糊逻辑决策的结果
	
	usart_config();	//配置串口模块，只有调试时才需要使用串口模块
	servo_config();	//配置舵机模块
	pid_config();	//配置PID控制模块
	motor_config();	//配置电机模块
	sonar_config();	//配置超声波模块
	
	while(1){
		barrier_distance_front=sonar_scan_barrier();	//获取前方障碍物的距离信息
		if(barrier_distance_front>=DISTANCE_SAFE){
			motor_set_speed(MAX_SPEED,MAX_SPEED);	//全速前进
			delay_ms(60);	//设置超声波测量周期为60ms以上，防止发射信号影响回响信号
		}else{
			motor_set_speed(0,0);	//通过PID控制小车的速度为0，使小车停下来，这里也可以直接控制电机刹车使小车停止
			
			//获取左边障碍物距离信息
			servo_turn_to(SERVO_DIRECTION_LEFT);
			barrier_distance_left=sonar_scan_barrier();
			
			//获取右边障碍物距离信息
			servo_turn_to(SERVO_DIRECTION_RIGHT);
			barrier_distance_right=sonar_scan_barrier();
			
			servo_turn_to(SERVO_DIRECTION_FRONT);	//前方障碍物的距离信息在小车停止时已经获得，所以这里只是将舵机转向前方
			fst=fuzzy_decision(barrier_distance_left,barrier_distance_front,barrier_distance_right);	//使用模糊逻辑进行决策
			motor_set_speed(fst.speed0,fst.speed1);	//将模糊逻辑决策的结果作用到电机上
			
			delay_ms(TIME_FUZZY_DELAY);	//延时，让模糊逻辑决策的结果作用一段时间
		}
	}
	
}

#ifdef USE_FULL_ASSERT	///<如果传入库函数的参数错误，则会调用此函数，调试时使用
void assert_failed(uint8_t* file, uint32_t line){
	printf("Error:In %s,at %d\n", file,line);
	while(1);
}
#endif
