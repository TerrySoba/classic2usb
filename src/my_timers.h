#ifndef MY_TIMERS_H
#define MY_TIMERS_H

#include <stdint.h>

/*
 * Description:
 *  waits time_ms milliseconds and then calls the callback function
 *
 * Side Effects:
 *  When called the timer1 is used, so you may not use it until the
 *  callback was called.
 */

uint8_t my_timer_oneshot(uint16_t time_ms, void (*callback)(void* ptr), void* ptr) __attribute__((signal));

/*
 * Description:
 *  Aborts any timer currently active
 */
void my_timer_abort();


#endif
