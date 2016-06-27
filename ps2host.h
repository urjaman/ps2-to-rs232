#ifndef PS2HOST_H
#define PS2HOST_H

void ps2host_pcint0_hook(void);
uint8_t ps2host_has_data(void);
uint8_t ps2host_get_data(void);
void ps2host_init(void);
void ps2host_tx_data(uint8_t d);

#endif
