/**
@file       pid.c
@brief      实现与PID控制有关的函数
@author     Zev
*/

#include "pid.h"
#include "usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"

#define T_SAMPLING 0.1f               ///<采样时间，单位为秒
#define PD 2.5f                       ///<比例带
#define TI 0.4f                       ///<积分时间，单位为秒
#define KP (1.0f / PD)                ///<比例系数
#define KI (KP * T_SAMPLING / TI)    ///<积分系数
#define PID_PWM_MIN 0                 ///<PWM的最小脉宽
#define PID_PWM_MAX (TIM5_PERIOD + 1) ///<PWM的最大脉宽
#define MIN_2(i, j) (i < j ? i : j)   ///<返回两个数的最小值

#if defined(DEBUG_PID)
#define USART_BUFFER_SIZE 100         //每次发送给上位机的最大字节数
int32_t usart_buf[USART_BUFFER_SIZE]; //数据缓冲区
uint32_t usart_buf_cnt;               //缓冲区计数

/**
@brief      将电机脉冲数据通过串口发送到上位机，用于上位机绘制PID图形
@param      None
@retval     None
*/
static void pid_send_to_computer(void)
{
    for (uint32_t i = 0; i < USART_BUFFER_SIZE; ++i)
    {
        usart_send_int32(usart_buf[i]);
        if (i < 50)
            usart_buf[i] = usart_buf[50 + i]; //保留前50个脉冲数据，用于观察连续性
    }
    usart_buf_cnt = 50;
}

/**
@brief      根据函数的形参调整PID的参数和电机的转动方向，用于整定PID控制的参数
@param      point 在采样时间段T_SAMPLING电机光电码盘脉冲数要达到的目标值
@param      pd 比例带，其倒数即为比例系数
@param      ti 积分时间，单位为秒
@param      td 微分时间，单位为秒，本程序使用的是PI控制，因此不需要使用该参数
@param      dir 电机转动方向
@retval     None
@note       在调试PID的参数时才需要使用此函数
*/
void pid_debug_params(uint32_t point, float pd, float ti, float td, MOTOR_DIRECTION dir)
{
    pd = 1 / pd;                            //将比例带pd转换为比例系数
    pid0.proportion = pid1.proportion = pd; //pid0对应的是小车右侧的电机，可以根据需要修改为左侧电机pid1
    pid0.integral = pid1.integral = ti == 0 ? 0 : pd * 0.1f / ti;
    //pid0.derivative=pid1.derivative=pd*td/0.1f;   //本程序使用的是PI控制，因此不需要使用该参数
    pid_set_point(point, point);       //设定脉冲目标值
    motor_set_direction(dir, MOTOR_1); //设置电机转动方向
}

#endif

/**
@brief      PID控制有关的结构体
@details    因为小车只需要控制两个电机，因此只定义了两个结构体变量：pid0和pid1，分别代表小车右边和左边的电机。本程序只用到了PI控制，因此把不需要的变量注释掉了
@note       这是私有结构体
*/
static struct PID_ST
{
    int32_t set_point;  ///<设定小车在采样时间内要达到的脉冲数
    int32_t last_error; ///<上一次误差
    //int32_t prev_error;   ///<上上次误差
    float proportion; ///<比例系数
    float integral;   ///<积分系数
                      //float derivative;   ///<微分系数
} pid0, pid1;

static void pid_rcc_config(void);
static void pid_tim_config(void);
static void pid_struct_init(struct PID_ST *pid);
static int32_t pid_increment_calc(int32_t step, struct PID_ST *pid);

/**
@brief      配置PID控制所需的时钟和定时器，并初始化与PID控制有关的结构体
@param      None
@retval     None
@note       在第一次使用PID控制前必须调用此函数进行时钟和定时器的配置，以后不需要调用此函数
*/
void pid_config(void)
{
    pid_rcc_config(); //配置PID控制所需的时钟
    pid_tim_config(); //配置PID控制所需的定时器

    //初始化PID控制的结构体
    pid_struct_init(&pid0);
    pid_struct_init(&pid1);
}

/**
@brief      通过当前时刻的两个电机的光电码盘脉冲数计算并设置下一时刻两个电机的输出脉宽
@param      step0 右边电机在采样时间段T_SAMPLING的光电码盘脉冲数
@param      step1 左边电机在采样时间段T_SAMPLING的光电码盘脉冲数
@retval     None
*/
void pid_control(int32_t step0, int32_t step1)
{
    //因为TIM5->CCRx为无符号数，但函数pid_increment_calc可能返回负值，因此必须使用有符号数pwm0和pwm1作为临时变量
    int32_t pwm0 = TIM5->CCR1;
    int32_t pwm1 = TIM5->CCR2;

    //通过增量式PID获得下一时刻的输出脉宽
    pwm0 += pid_increment_calc(step0, &pid0);
    pwm1 += pid_increment_calc(step1, &pid1);

    //限制变量pwm的取值在区间[PID_PWM_MIN,PID_PWM_MAX]，因为变量pwm的值最终会赋给TIM5->CCRx
    if (pwm0 > PID_PWM_MAX)
    {
        pwm0 = PID_PWM_MAX;
    }
    else if (pwm0 < PID_PWM_MIN)
    {
        pwm0 = PID_PWM_MIN;
    }
    if (pwm1 > PID_PWM_MAX)
    {
        pwm1 = PID_PWM_MAX;
    }
    else if (pwm1 < PID_PWM_MIN)
    {
        pwm1 = PID_PWM_MIN;
    }

    //通过将变量pwm赋给TIM5->CCRx，控制PWM的输出脉宽，进而影响电机的转速
    TIM5->CCR1 = pwm0;
    TIM5->CCR2 = pwm1;
}

/**
@brief      设置两个电机在一秒要达到的脉冲数
@param      point0 右边电机在一秒要达到的脉冲数
@param      point1 左边电机在一秒要达到的脉冲数
@retval     None
*/
void pid_set_point(int32_t point0, int32_t point1)
{
    pid0.set_point = MIN_2(point0 * T_SAMPLING, POINT_MAX); //point0和point1对应的是一秒要达到的脉冲数，需要将其转换为采样时间T_SAMPLING对应的脉冲数
    pid1.set_point = MIN_2(point1 * T_SAMPLING, POINT_MAX);
}

/**
@brief      配置PID控制所需的时钟
@param      None
@retval     None
@note       这是一个私有函数
*/
static void pid_rcc_config(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/**
@brief      配置PID控制所需的定时器
@param      None
@retval     None
@note       这是一个私有函数
*/
static void pid_tim_config(void)
{
    TIM_TimeBaseInitTypeDef tim_base_init;

    tim_base_init.TIM_ClockDivision = 0;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base_init.TIM_Period = 1000 - 1;
    tim_base_init.TIM_Prescaler = 8400 - 1; //TIM2默认时钟频率为84MHz
    tim_base_init.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim_base_init);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); //设置当TIM2发生更新事件时，输出触发信号
    TIM_Cmd(TIM2, ENABLE);
}

/**
@brief      初始化PID控制的结构体
@param      pid 需要初始化的PID_ST变量
@retval     None
@note       这是一个私有函数；本程序只是用到了PI控制，因此只是初始化与PI控制有关的变量
*/
static void pid_struct_init(struct PID_ST *pid)
{
    pid->set_point = pid->last_error = 0;
    pid->proportion = KP; //根据宏定义KP设置比例系数
    pid->integral = KI;   //根据宏定义KI设置积分系数
}

/**
@brief      增量式PID算法
@details    此函数实现了增量式PID算法，输入量是电机的光电码盘脉冲数，输出量是控制电机的PWM波形脉宽的增量。通过计算当前时刻输入量与设定目标值的误差，
            获得下一时刻的输出量，如此循环往复，不断减小输入量与设定目标值的误差
@param      step 当前时刻参数pid对应电机的光电码盘的脉冲数
@param      pid 参数step对应的电机的PID控制结构体
@retval     下一时刻控制电机的PWM波形脉宽的增量
@note       这是一个私有函数
*/
static int32_t pid_increment_calc(int32_t step, struct PID_ST *pid)
{
    int32_t current_error; //当前时刻光电码盘脉冲数与设定目标值的误差
    int32_t inc;           //下一时刻控制电机的PWM波形脉宽的增量

#if defined(DEBUG_PID)
    if (pid == &pid1)
    {
        usart_buf[usart_buf_cnt++] = step;
        if (usart_buf_cnt == USART_BUFFER_SIZE)
        {
            pid_send_to_computer();
        }
    }
#endif

    current_error = pid->set_point - step;

    //增量式PID的计算公式，由于程序只需要PI控制，因此注释掉微分控制部分的计算
    inc = pid->proportion * (current_error - pid->last_error) + pid->integral * current_error; //+pid->derivative*(current_error-2*pid->last_error+pid->prev_error);
    //pid->prev_error=pid->last_error;
    pid->last_error = current_error;

    return inc;
}
