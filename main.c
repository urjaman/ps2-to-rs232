#include <stdint.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USI_UART.h"

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
	PORTB = 0x1F;
	DDRB  = 0x00;
	CLKPR = _BV(CLKPCE);
	CLKPR = _BV(CLKPS1)|_BV(CLKPS0);

	USI_UART_Init();
	sei();
	USI_UART_Transmit_Byte('\r');
	USI_UART_Transmit_Byte('\n');
	USI_UART_Transmit_Byte('H');
	USI_UART_Transmit_Byte('i');
	USI_UART_Transmit_Byte('\r');
	USI_UART_Transmit_Byte('\n');

	for (;;) {
		if (USI_UART_Data_In_Receive_Buffer()) {
			_delay_ms(15);
			USI_UART_Transmit_Byte(USI_UART_Receive_Byte());
		}
	}
}
