#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>       
#include <time.h>
#include <stdio.h>
#include "rtc.h"

//#define UNIX_TIME_START 1718000000UL
#define LFCLK_FREQ      32768UL
#define RTC2_PRESCALER  0
#define RTC_TICKS_PER_SEC (LFCLK_FREQ / (RTC2_PRESCALER + 1))

static volatile bool rtc2_alarm_fired = false;

/* Direct IRQ handler registration */
void RTC2_IRQHandler(void)
{
    if (NRF_RTC2->EVENTS_COMPARE[0]) {
        NRF_RTC2->EVENTS_COMPARE[0] = 0;
        rtc2_alarm_fired = true;
        __SEV(); // Wake CPU from WFE
    }
}

static uint32_t unix_base = 0;   // UNIX time in seconds, set from BLE

/* Initialize RTC2 peripheral */
void rtc2_init(uint32_t unix_time) {
    unix_base = unix_time;
    printk("[RTC2] Initializing RTC2...\n");

    // Start LFCLK from XTAL
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (!NRF_CLOCK->EVENTS_LFCLKSTARTED) {}

    // Stop and configure RTC2
    NRF_RTC2->TASKS_STOP = 1;
    NRF_RTC2->PRESCALER = RTC2_PRESCALER;
    NRF_RTC2->TASKS_CLEAR = 1;
    NRF_RTC2->TASKS_START = 1;

    // // Enable COMPARE0 event & interrupt
    // NRF_RTC2->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    // NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    // /* Register the interrupt with Zephyr */
    // IRQ_DIRECT_CONNECT(RTC2_IRQn, 3, RTC2_IRQHandler, 0);

    // /* Enable the interrupt in NVIC */
    // irq_enable(RTC2_IRQn);

    

    printk("[RTC2] RTC2 running (prescaler=%d, unix start=%u).\n",
           RTC2_PRESCALER, unix_time);
}

// uint64_t get_timestamp_ms(void) {
//     uint32_t ticks = NRF_RTC2->COUNTER;
//     uint64_t ms = ((uint64_t)ticks * 1000UL) / RTC_TICKS_PER_SEC;
//     return ((uint64_t)UNIX_TIME_START * 1000UL) + ms;
// }

/*This function returns the current timestamp in milliseconds based on the number of ticks from the RTC2 hardware counter.

It calculates the elapsed time since a defined starting point (UNIX_TIME_START) and adds the milliseconds computed from the counter ticks.*/
uint64_t get_timestamp_ms(void) {
    uint32_t ticks = NRF_RTC2->COUNTER;
    uint64_t elapsed_ms = ((uint64_t)ticks * 1000UL) / RTC_TICKS_PER_SEC;
    //printk("\n---------------%llu------------------",((uint64_t)unix_base * 1000ULL) + elapsed_ms);
    return ((uint64_t)unix_base * 1000ULL) + elapsed_ms;
}


void format_timestamp(uint64_t timestamp_ms, char *buffer, size_t size) {
    time_t sec = timestamp_ms / 1000;
    uint16_t ms = timestamp_ms % 1000;
    struct tm t;
    gmtime_r(&sec, &t);
    snprintk(buffer, size, "%02d/%02d/%04d %02d:%02d:%02d.%03d",
        t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
        t.tm_hour, t.tm_min, t.tm_sec, ms);
}

void rtc2_set_alarm(uint32_t ms_from_now) {
    uint32_t current_ticks = NRF_RTC2->COUNTER;
    uint32_t ticks_offset = (ms_from_now * RTC_TICKS_PER_SEC) / 1000UL;
    uint32_t compare_value = (current_ticks + ticks_offset) & 0x00FFFFFF;
    NRF_RTC2->CC[0] = compare_value;
    rtc2_alarm_fired = false;
}

bool rtc2_alarm_triggered(void) {
    return rtc2_alarm_fired;
}
