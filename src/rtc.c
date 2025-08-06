#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <time.h>
#include <stdio.h>
#include "rtc.h"

#define UNIX_TIME_START 1718000000UL   // Unix timestamp (set it manually)
#define LFCLK_FREQ 32768UL             // Low-frequency clock 32768 Hz
#define RTC2_PRESCALER 0
#define RTC_TICKS_PER_SEC (LFCLK_FREQ / (RTC2_PRESCALER + 1))  // Number of RTC ticks per second

/*This function initializes the Real-Time Counter 2 (RTC2) on the Nordic SoC to function as a system timer for timestamping sensor data. It performs the following steps:

-> Starts the low-frequency clock (LFCLK) using an external crystal.

-> Stops, resets, and sets the prescaler for RTC2.

-> Starts RTC2 to run continuously.

-> It is used later to provide precise time stamps in milliseconds for sampled sensor data.*/

void rtc2_init(void) {
    // Print a message to indicate RTC2 initialization has started
    printk("[RTC2] Initializing RTC2...\n");

    // Set the LFCLK (low-frequency clock) source to external crystal oscillator
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;

    // Clear the 'LFCLK started' event flag
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

    // Trigger the task to start the LFCLK
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    // Wait until the LFCLK has started (polling the event flag)
    while (!NRF_CLOCK->EVENTS_LFCLKSTARTED) {}

    NRF_RTC2->TASKS_STOP = 1;               // Stop RTC2 before reconfiguration
    NRF_RTC2->PRESCALER = RTC2_PRESCALER;   // Set the RTC2 prescaler to control tick frequency (0 = no division)
    NRF_RTC2->TASKS_CLEAR = 1;              // Clear the RTC2 counter (reset to 0)
    NRF_RTC2->TASKS_START = 1;              // Start the RTC2 counter
    
    // Print a message indicating RTC2 is now running with configured settings
    printk("[RTC2] RTC2 running (prescaler=%d, unix start=%lu).\n", RTC2_PRESCALER, (unsigned long)UNIX_TIME_START);
}


/*This function returns the current timestamp in milliseconds based on the number of ticks from the RTC2 hardware counter.

It calculates the elapsed time since a defined starting point (UNIX_TIME_START) and adds the milliseconds computed from the counter ticks.*/
uint64_t get_timestamp_ms(void) {
    uint32_t ticks = NRF_RTC2->COUNTER;               // Read the current tick count from the RTC2 hardware counter (increments every ~30.5 µs)
    uint64_t ms = ((uint64_t)ticks * 1000UL) / RTC_TICKS_PER_SEC;       // Convert tick count to milliseconds using the tick rate (32768 ticks/sec)
    return ((uint64_t)UNIX_TIME_START * 1000UL) + ms;           // Return the full timestamp = initial UNIX time (in ms) + elapsed milliseconds
}


/*This function converts a timestamp into a human-readable format (dd/mm/yyyy  hr:min:sec:ms)*/
void format_timestamp(uint64_t timestamp_ms, char *buffer, size_t size) {
    time_t sec = timestamp_ms / 1000;       // Convert milliseconds to seconds (standard Unix timestamp format)
    uint16_t ms = timestamp_ms % 1000;      // Extract the milliseconds part (remainder after dividing by 1000)
    struct tm t;                            // Structure to hold broken-down time (year, month, day, hour, etc.)
    gmtime_r(&sec, &t);                     // Convert 'sec' into UTC time represented in 'struct tm' (thread-safe version)

    // Format the date & time into the buffer as: dd/mm/yyyy hh:mm:ss.mmm
    snprintf(buffer, size, "%02d/%02d/%04d %02d:%02d:%02d.%03d",
        t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
        t.tm_hour, t.tm_min, t.tm_sec, ms);
}
