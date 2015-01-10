/* This is helpers for sleep-while-doing-something-external-and-watch-power-supply ... */
/* To be used by button pressing subsystem and led pulsing (in the first app) */
#include <stdint.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "main.h"
#include "actslp.h"

#define P5V_LIMIT 260

EMPTY_INTERRUPT(ADC_vect); // Handled by adc_sample, but use INT to wakeup.

uint16_t adc_sample(void) {
	// sleepy sampling
	cli();
	ADCSRA |= _BV(ADSC);
	sys_sleep(SLEEP_MODE_ADC);
	while (ADCSRA&_BV(ADSC)) { // We might get woken up due to other reasons, thus dont assume sample done
		sys_sleep(SLEEP_MODE_ADC);
	}
	return ADC;
}

void adc_enable_p5v(void) {
	power_adc_enable();
	ADCSRA = _BV(ADEN) | _BV(ADIF) | _BV(ADIE) | _BV(ADPS1) | _BV(ADPS0);
	ADCSRB = 0;
	ADMUX = 0b00100001; // 1.1V ref according to Vcc (=measure VCC)
	adc_sample(); // flush one sample, it might be bad due to just booted 1.1V ref
	adc_sample(); // or two?, yep, seems to be so...
}

void adc_disable(void) {
	ADMUX = 0; // To turn off the 1.1V ref
	ADCSRA = 0; // To turn off other ADC stuff
	power_adc_disable(); // To turn off ADC clocks
}


uint16_t adc_test_sample(void) {
	adc_enable_p5v();
	uint16_t sam = adc_sample();
	adc_disable(); // save power
	return sam;
}


/* This is to be called before doing action, in case power already too little. */
uint8_t sleep_act_testpower(void) {
	uint16_t sam = adc_test_sample();
	if (sam>P5V_LIMIT) {
		return 1; // OMG OMG OMG
	}
	return 0; // Okay
}

static volatile uint8_t wdt_ticks = 0;

void wdt_init(void) {
	/* This sequence both turns off WDT reset mode and sets the period to 32ms */
	/* Thus preparing WDT to be used as a simple timer later. */
	wdt_reset();
	MCUSR &= ~_BV(WDRF);
	WDTCSR = _BV(WDCE) | _BV(WDE);
	WDTCSR = _BV(WDIF) | _BV(WDIE) | _BV(WDP0);
	WDTCSR &= ~_BV(WDIE);
}

static void actslp_wdt_init(void) {
	cli();
	wdt_ticks = 0;
	wdt_reset(); // clear the counter
	WDTCSR |=  _BV(WDIE);
}

// Time is roughly in 32ms units, and this is propably called right after init.
uint8_t sleep_act_do(uint8_t time) {
	actslp_wdt_init();
	// counter is running, now sleep
	while (wdt_ticks < time) { // 255 is never less than any possible value of time, thus safe loop.
		sys_sleep(SLEEP_MODE_PWR_DOWN);
	}
	WDTCSR &= ~_BV(WDIE); // disable the WDT interrupt mode
	if (wdt_ticks==255) return 1; // Out of power (or timeout saturation)...
	return 0;
}

// The operating voltage is too low, disable everything you can and wait for better times.
void sleep_until_pwr(void) {
	// Turn off timer, USI, pin change interrupt, everything.
	cli();
	uint8_t tsave = TCCR0B;
	TCCR0B = 0;
	uint8_t usisave = USICR;
	USICR = 0;
	uint8_t gimsave = GIMSK;
	GIMSK = 0;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	actslp_wdt_init();
	power_timer0_disable();
	power_usi_disable();
	for(;;) {
		cli();
		wdt_ticks = 0;
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
		if (wdt_ticks==1) break; // it would be 255 if out of power - 0 if wrong int.
	}
	cli(); // we might have-non-logical moments here while restoring.
	WDTCSR &= ~_BV(WDIE);
	power_usi_enable();
	power_timer0_enable();
	GIMSK = gimsave;
	USICR = usisave;
	TCCR0B = tsave;
	sei();
}

ISR(WDT_vect)
{
	uint8_t t = wdt_ticks;
	if (t<255) t++;
	if (sleep_act_testpower()) { // This will enable interrupts, thats OK. (run UART and ADC stuff if needed)
		t = 255;
	}
	wdt_ticks = t;
}
