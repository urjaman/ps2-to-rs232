#define BAUDRATE 38400

void USI_UART_Flush_Buffers( void );
void USI_UART_Transmit_Byte( unsigned char data );
unsigned char USI_UART_Receive_Byte( void );
unsigned char USI_UART_Data_In_Receive_Buffer( void );
void USI_UART_Init(void);

#include "ps2host.h"
#define PCINT0_HOOK ps2host_pcint0_hook()
