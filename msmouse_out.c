#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#include "USI_UART.h"
#include "timer.h"
#include "ps2mouse.h"
#include "msmouse.h"

static int16_t ser_x,ser_y;
static uint8_t ser_btns;

static uint8_t rstate = 0;

void msmouse_init(void) {
}

void msmouse_run(void) {
	if (!rstate) {
		if (!(PINB & _BV(2))) { // oh, activated. 
			rstate = 1;
			USI_UART_Transmit_Byte('M' | 0x80);
			ser_btns = 0xFF; /* send a frame with the buttons on "init" anyways */
			ps2mouse_get_state(&ser_x,&ser_y);
		}
	} else {
		if (PINB & _BV(2)) {
			/* reset ... */
			rstate = 0;
			return;
		}
		if (!USI_UART_Transmit_Busy()) {
			int8_t dx, dy;
			uint8_t nb = ps2mouse_get_delta(ser_x, ser_y, &dx, &dy) & 0x3; // ignore middle button ...
			if ((nb != ser_btns)||(dx)||(dy)) {
				ser_x += dx;
				ser_y += dy;
				ser_btns = nb;
				dy *= -1; // MS mouse vs PS/2 ...

				uint8_t mspacket[3] = { 0xC0, 0x80, 0x80 };
				if (nb & 1) mspacket[0] |= 0x20;
				if (nb & 2) mspacket[0] |= 0x10;
				mspacket[0] |= (dy >> 4) & 0xC;
				mspacket[0] |= (dx >> 6) & 0x3;
				mspacket[1] |=  dx & 0x3F;
				mspacket[2] |=  dy & 0x3F;
				for (uint8_t i=0;i<3;i++) {
					USI_UART_Transmit_Byte(mspacket[i]);
				}
			}
		}
	}
}
