#ifndef RTC2_H
#define RTC2_H
#include <stdint.h>
#include <stddef.h>

void rtc2_init(void);
uint64_t get_timestamp_ms(void);
void format_timestamp(uint64_t timestamp_ms, char *buffer, size_t size);

#endif