/**
@file       itm.h
@brief      包含此文件后，可以实现通过Jlink发送调试信息给上位机，通过Keil的调试窗口直接查看
@author     Zev
*/

#ifndef ITM_H
#define ITM_H
#include <stdio.h>

/*To use printf*/
#define ITM_Port8(n) (*((volatile unsigned char *)(0xE0000000 + 4 * n)))
#define ITM_Port16(n) (*((volatile unsigned short *)(0xE0000000 + 4 * n)))
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4 * n)))

#define ITM_DEMCR (*((volatile unsigned long *)(0xE000EDFC)))
#define ITM_TRCENA 0x01000000

struct __FILE
{
  int handle; /* Add whatever you need here */
};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
  if (ITM_DEMCR & ITM_TRCENA)
  {
    while (ITM_Port32(0) == 0)
      ;
    ITM_Port8(0) = ch;
  }
  return (ch);
}

#endif
