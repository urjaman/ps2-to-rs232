void wdt_init(void);
uint16_t adc_sample(void);
void adc_enable_p5v(void);
void adc_disable(void);
/* This is to be called before doing action, in case power already too little. */
uint8_t sleep_act_testpower(void);
/* Time is roughly in 32ms units, and this is propably called right after init. */
uint8_t sleep_act_do(uint8_t time);
/* Wait for more power... */
void sleep_until_pwr(void);
uint16_t adc_test_sample(void);
