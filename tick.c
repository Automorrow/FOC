#include "tick.h"

volatile unsigned int timestamp_us = 0;
void clock_time_init(void) {    
	SysTick_Config(CLOCK_TIME_LOAD_VALUE);//0.5s Ò»´ÎÖÐ¶Ï
}
unsigned long clock_time(void) {
    return timestamp_us + ((CLOCK_TIME_LOAD_VALUE - SysTick->VAL) * CLOCK_TIME_PER_VALUE_US);
}
int clock_time_exceed(unsigned int ref, unsigned int span_us) {
    return (clock_time() - ref) > span_us;
}

void delay_us(unsigned int us) {
    unsigned int start = clock_time();
    while (!clock_time_exceed(start, us));
}

void clock_time_Handler(void) {
	timestamp_us += CLOCK_TIME_INIT_PERIOD_US;
}
