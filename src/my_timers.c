#include "my_timers.h"

#include <avr/interrupt.h>
#include "bit_tools.h"
#include <avr/io.h>

static void (*timer_callback)(void* ptr);
static void* timer_ptr;

ISR(TIMER1_OVF_vect, ISR_NOBLOCK) {
    // at first disable timer1
    TCCR1B &= ~(1<<CS10|1<<CS11|1<<CS12);

    // now call callback function
    timer_callback(timer_ptr);
}

uint8_t my_timer_oneshot(uint16_t time_ms, void (*callback)(void* ptr), void* ptr) {
    // at first set callback function
    timer_callback = callback;
    timer_ptr = ptr;

    // now start timer1
    TCNT1 = (uint16_t)(0xffff - (((uint32_t)(F_CPU))/(1024UL * 1000UL)) * time_ms);

    TCCR1B |= (1<<CS10|1<<CS12);
    TIMSK |= (1<<TOIE1);

    return 1;
}

void my_timer_abort() {
    TCCR1B &= ~(1<<CS10|1<<CS11|1<<CS12);
}


