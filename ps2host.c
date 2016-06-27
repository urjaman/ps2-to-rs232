#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "ps2host.h"


#define RXBUF_SIZE 4

#define DAT 3
#define CLK 4


static uint8_t odd(uint8_t d) {
	d ^= (d >> 4);
	d ^= (d >> 2);
	return (d ^ (d >> 1) ^ 1) & 1;
}

enum stat {
	RX_START = 0,
	RX_D0,
	RX_D1,
	RX_D2,
	RX_D3,
	RX_D4,
	RX_D5,
	RX_D6,
	RX_D7,
	RX_PAR,
	RX_STOP,
	RX_ERR, // a spacer value ... 

	TX_D0,
	TX_D1,
	TX_D2,
	TX_D3,
	TX_D4,
	TX_D5,
	TX_D6,
	TX_D7,
	TX_PAR,
	TX_ACK
};

static uint8_t rx_buff[RXBUF_SIZE];
static volatile uint8_t rx_wp;
static uint8_t rx_rp;


static uint8_t rx_byte;
static uint16_t tx_dat;
static volatile uint8_t status;

static void clk_lo(void) {
	PORTB &= ~_BV(CLK);
	DDRB |= _BV(CLK);
}

static void clk_hi(void) {
	DDRB &= ~_BV(CLK);
	PORTB |= _BV(CLK);
}

static void dat_lo(void) {
	PORTB &= ~_BV(DAT);
	DDRB |= _BV(DAT);
}

static void dat_hi(void) {
	DDRB &= ~_BV(DAT);
	PORTB |= _BV(DAT);
}

void ps2host_pcint0_hook(void)
{
	uint8_t b = PINB;
	if ( (!(b & _BV(CLK))) && (PCMSK & _BV(CLK))) {
		uint8_t d = b & _BV(DAT);
		uint8_t s = status;
		status = s+1;
		/* first, check for TX... */
		if (s>=TX_D0) {
			if (s == TX_ACK) {
				// TODO: check the ack bit...
				dat_hi();
				status = 0;
				return;
			}
			if (tx_dat&1) {
				dat_hi();
			} else {
				dat_lo();
			}
			tx_dat = tx_dat >> 1;
			return;
		} else if (s == RX_PAR) {
		//	if ((!!d) == odd(rx_byte)) {
				uint8_t off = rx_wp;
				rx_buff[off] = rx_byte;
				rx_wp = (off+1)&(RXBUF_SIZE-1);
		//	} else {
				// TODO: Handle error...
		//	}
			return;
		} else if (s == RX_START) {
			if (d) {
				// not a start bit...
				status = 0;
			}
			return;
		} else if (s == RX_STOP) {
			// maybe check for framing error, but for now, ignore...
			status = 0;
			return;
		}
		uint8_t db = rx_byte;
		db = db >> 1;
		if (d) db |= 0x80;
		rx_byte = db;
	}

}

uint8_t ps2host_has_data(void) {
	if (rx_wp != rx_rp) return 1;
	return 0;
}

uint8_t ps2host_get_data(void) {
	if (rx_wp == rx_rp) return 0;
	uint8_t off = rx_rp;
	uint8_t d = rx_buff[off++];
	rx_rp = off & (RXBUF_SIZE-1);
	return d;
}

void ps2host_tx_data(uint8_t d) {
	uint16_t dat = (odd(d) << 8) | d;
	/* Wait for actions ... (previous tx, receive, whatever..) */
	do {
		PCMSK |= _BV(CLK);
		while (status); // TODO: timeout
		PCMSK &= ~_BV(CLK);
	} while (status); // check that we "got" the authority to do stuff...
	tx_dat = dat;
	status = TX_D0;
	clk_lo();
	_delay_us(120);
	dat_lo();
	_delay_us(10);
	clk_hi();
	PCMSK |= _BV(CLK);
}


void ps2host_init(void) {
	status = 0;
	PCMSK |= _BV(CLK);
}
