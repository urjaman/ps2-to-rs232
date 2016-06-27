#include <stdint.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USI_UART.h"
#include "ps2host.h"
#include "timer.h"
#include "ps2mouse.h"
#include "msmouse.h"

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

uint8_t hex_char(uint8_t nib) {
	nib &= 0xF;
	nib |= 0x30;
	if (nib > 0x39) nib += 7;
	return nib;
}

void hex_out(uint8_t d) {
	USI_UART_Transmit_Byte(hex_char(d>>4));
	USI_UART_Transmit_Byte(hex_char(d));
	USI_UART_Transmit_Byte(' ');
}


void main(void) {
	PORTB = 0x0F;
	DDRB  = 0x10;
	CLKPR = _BV(CLKPCE);
//	CLKPR = _BV(CLKPS1)|_BV(CLKPS0);
	CLKPR = 0;

	USI_UART_Init();
	ps2host_init();
	timer_init();
	ps2mouse_init();
	sei();
	DDRB &= ~_BV(4);
	PORTB |= _BV(4);
	msmouse_init();

#if 0
	USI_UART_Transmit_Byte('\r');
	USI_UART_Transmit_Byte('\n');
	USI_UART_Transmit_Byte('H');
	USI_UART_Transmit_Byte('i');
	hex_out(0xE5);
	USI_UART_Transmit_Byte('\r');
	USI_UART_Transmit_Byte('\n');
#endif
	
	for (;;) {
		ps2mouse_run();
		msmouse_run();
#if 0
		if (USI_UART_Data_In_Receive_Buffer()) {
			_delay_ms(15);
			while (USI_UART_Data_In_Receive_Buffer()) USI_UART_Transmit_Byte(USI_UART_Receive_Byte());
		}
		if (!USI_UART_Transmit_Busy()) {
			uint16_t bv=0;
			uint8_t b;
			int8_t x,y;
			b = ps2mouse_get_delta(bv,bv,&x,&y);
			hex_out(PINB);
			hex_out(b);
			hex_out(x);
			hex_out(y);
			USI_UART_Transmit_Byte('\r');
			USI_UART_Transmit_Byte('\n');
		}
#endif

	}
}
