#include <stdint.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "actslp.h"
#include "USI_UART.h"


#define BTN1 0x01
#define BTN2 0x04
#define BTN3 0x02
#define BTN4 0x08

#define BTN5 0x11
#define BTN6 0x14
#define BTN7 0x12
#define BTN8 0x18

#define BUTTON_TIME 8

/* LED Pulses:
 * 4: attach
 * 2: detach
 * 32: L-key
 */
uint8_t led_pulse(uint8_t time)
{
	uint8_t r;
	if (sleep_act_testpower()) {
		sleep_until_pwr();
		return 1;
	}
	/* LED ON */
	PORTA |= _BV(7);
	DDRA |= _BV(7);
	r = sleep_act_do(time);
	/* LED OFF */
	DDRA &= ~_BV(7);
	PORTA &= ~_BV(7);
	if (r) {
		sleep_until_pwr();
		return r;
	}
	return 0;
}

uint8_t push_button(uint8_t btn)
{
	uint8_t r;
	if (sleep_act_testpower()) {
		sleep_until_pwr();
		return 1;
	}
	/* push button */
	if (btn&0x10) PORTB |= _BV(0);
	else PORTB &= ~_BV(0);
	PORTA = (PORTA&0xF0) | (btn&0xF);
	r = sleep_act_do(BUTTON_TIME);
	/* release button */
	PORTA = PORTA&0xF0;
	if (r) {
		sleep_until_pwr();
		return r;
	}
	return 0;
}

/* serial debug fn */
void led_out_byte(uint8_t byte)
{
	for (uint8_t b=0;b<8;b++) {
		if (byte&0x80) {
			if (led_pulse(24)) return;
			if (sleep_act_do(16)) return;
		} else {
			if (led_pulse(8)) return;
			if (sleep_act_do(32)) return;
		}
		byte = byte<<1;
	}
	sleep_act_do(64); // inter-byte gap
}

void sys_sleep(uint8_t mode)
{
	uint8_t cr = MCUCR & 0xC7; // Clear sleep-related bits
	uint8_t cre = cr | _BV(SE);
	cli();
	// Clear sets mode = idle
	if (!TCCR0B) cre |= mode; // Idle needed if TCCR0B to let timer run
	MCUCR = cre; /* sleep enable and set mode */
	sei();
	sleep_cpu();
	MCUCR = cr; /* sleep_disable */
}

void main(void) {
	PORTA = 0x10;
	DDRA  = 0x0F;
	PORTB = 0x06;
	DDRB  = 0x01;
	DIDR0 = _BV(ADC5D);
	PRR = _BV(PRTIM1) | _BV(PRADC);
	CLKPR = _BV(CLKPCE);
	CLKPR = _BV(CLKPS1)|_BV(CLKPS0);
	wdt_init(); // This needs to be early.
	USI_UART_Init();
	sei();
	if (sleep_act_testpower()) { // This is our BOR attempt :P
		sleep_until_pwr();
	}
attach_detach:
	if (PINA&_BV(6)) {
		led_pulse(2);
		cli(); // prevent race here where we dont wake up due to no PCI.
		if (PINA&_BV(6)) { // true detach - still detached after led pulse
			while (PINA&_BV(6)) {
				sys_sleep(SLEEP_MODE_PWR_DOWN);
				cli(); // same here, cli, then check.
			}
			led_pulse(4); // attach (this will sei())
		}
	} else {
		led_pulse(4);
	}
	for(;;) {
		unsigned char c = USI_UART_Receive_Byte();
		switch (c) {
			case 0: // detach or break
				goto attach_detach;
			case '1': push_button(BTN1); break;
			case '2': push_button(BTN2); break;
			case '3': push_button(BTN3); break;
			case '4': push_button(BTN4); break;

			case 'Q':
			case 'q':
				push_button(BTN5);
				break;
			case 'W':
			case 'w':
				push_button(BTN6);
				break;
			case 'E':
			case 'e':
				push_button(BTN7);
				break;
			case 'R':
			case 'r':
				push_button(BTN8);
				break;

			case 'l':
			case 'L':
				led_pulse(32);
				break;
			case 'S':
			case 's':
				led_out_byte(adc_test_sample());
				break;
			default:
				// attach might send something garbled, usually 0xFF, thus ignore those with b7 set...
				// disable that filter for debugging if needed.
				if (!(c&0x80)) led_out_byte(c);
				break;
		}
	}
}
