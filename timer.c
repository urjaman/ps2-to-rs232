#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "timer.h"


#define TIMER_DIV 64
#define DR(a,b) (((a)+((b)/2))/(b))
#define TIMER_HZ DR(F_CPU,TIMER_DIV)
#define CTC_VAL (DR(TIMER_HZ,1000)-1)

static volatile uint16_t msectimer;

void timer_init(void) {
	OCR1C = CTC_VAL;
	TCCR1 = _BV(CTC1) | _BV(CS12) | _BV(CS11) | _BV(CS10);
	TIMSK |= _BV(TOIE1);
}

uint16_t get_msectimer(void) {
	uint16_t v; // TODO: use asm to shave 1 cycle of ISR-off time ... or use reti to save the sei() totally...
	cli();
	v = msectimer;
	sei();
	return v;
}

ISR(TIMER1_OVF_vect) {
	msectimer++;
}
