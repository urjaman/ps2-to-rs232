void ps2mouse_init(void);
void ps2mouse_run(void);
uint8_t ps2mouse_get_state(int16_t *x, int16_t *y);
uint8_t ps2mouse_get_delta(int16_t bx, int16_t by, int8_t *dx, int8_t*dy);
