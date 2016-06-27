#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#include "ps2host.h"
#include "timer.h"
#include "ps2mouse.h"

static int16_t int_x,int_y;
static uint8_t btns;

static uint8_t frame_buf[3];
static uint8_t frame_off;

static uint16_t last_data_ms;

static uint8_t waiting_fa;

void ps2mouse_init(void)
{
	last_data_ms = get_msectimer();

}

static void enable_stream(void) {
	last_data_ms = get_msectimer();
	waiting_fa = 1;
	ps2host_tx_data(0xF4); // enable data reporting
}

static void parse_frame(void) {
	btns = frame_buf[0]&7;

	uint16_t tmp = frame_buf[0]&0x20 ? 0xFF00 : 0;
	tmp |= frame_buf[2];
	int16_t n_y = (int16_t)tmp;

	tmp = frame_buf[0]&0x10 ? 0xFF00 : 0;
	tmp |= frame_buf[1];
	int16_t n_x = (int16_t)tmp;

	int_x += n_x;
	int_y += n_y;

	frame_off = 0;
}

void ps2mouse_run(void)
{
	while (ps2host_has_data()) {
		uint8_t d = ps2host_get_data();
		last_data_ms = get_msectimer();
		if ((waiting_fa)&&(d==0xFA)) {
			waiting_fa = 0;
			continue;
		}
		if (((!frame_off)&&(d & 0x08))||(frame_off)) {
			frame_buf[frame_off++] = d;
			if (frame_off>=3) parse_frame();
		}
	}
	uint16_t passed = get_msectimer() - last_data_ms;
	if ((frame_off)&&(passed >= 50)) {
		/* poof... */
		frame_off = 0;
		enable_stream();
		return;
	}
	/* Just do something once in a while... */
	if (passed >= 1000) {
		enable_stream();
		return;
	}

}

uint8_t ps2mouse_get_state(int16_t *x, int16_t *y) {
	if (x) *x = int_x;
	if (y) *y = int_y;
	return btns;

}

uint8_t ps2mouse_get_delta(int16_t bx, int16_t by, int8_t *dx, int8_t*dy) {
	int16_t bdx = int_x - bx;
	int16_t bdy = int_y - by;
	if (bdx < -127) bdx = -127;
	if (bdx > 127) bdx = 127;
	if (bdy < -127) bdy = -127;
	if (bdy > 127) bdy = 127;
	if (dx) *dx = bdx;
	if (dy) *dy = bdy;
	return btns;
}

