#ifndef TICK_H
#define TICK_H
#include "pt32x031.h"
#include <stdint.h>
#include <stdbool.h>

#define CLOCK_TIME_UNIT_US          1
#define CLOCK_TIME_UNIT_MS          1000 * CLOCK_TIME_UNIT_US
#define CLOCK_TIME_UNIT_S           1000 * CLOCK_TIME_UNIT_MS
// SysTick定时器周期，设置为500毫秒
#define CLOCK_TIME_LOAD_VALUE       (SystemCoreClock / (2))
#define CLOCK_TIME_INIT_PERIOD_US   (CLOCK_TIME_LOAD_VALUE / (SystemCoreClock / CLOCK_TIME_UNIT_S))
#define CLOCK_TIME_PER_VALUE_US     (CLOCK_TIME_UNIT_S/SystemCoreClock)
void clock_time_init(void);
unsigned long clock_time(void);
int clock_time_exceed(unsigned int ref, unsigned int span_us);
void delay_us(unsigned int us);
void clock_time_Handler(void);

#endif
