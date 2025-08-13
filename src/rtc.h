#ifndef RTC_H
#define RTC_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include <inttypes.h>

void rtc2_init(void);
uint64_t get_timestamp_ms(void);
void format_timestamp(uint64_t timestamp_ms, char *buffer, size_t size);
void rtc2_set_alarm(uint32_t ms_from_now);
bool rtc2_alarm_triggered(void);
void RTC2_IRQHandler(void);

#endif 
