/*****************************************************************************
*
* Original Copyright (C) 2003 Atmel Corporation
* Modified 2015 by Urja Rannikko. Modifications are in Public Domain.
*
* File              : USI_UART.c
*
* Supported devices : ATtiny84
*
* Description       : Functions for USI_UART_receiver and USI_UART_transmitter.
*                     Uses Pin Change Interrupt to detect incoming signals.
*
*
****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "USI_UART.h"

/* Configuration */

#define TIMER_PRESCALER           1
#define UART_RX_BUFFER_SIZE        4     /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART_TX_BUFFER_SIZE        4
//#define UTX_ENABLED
#define URX_ENABLED
#define INVERTED_IO
#define USE_GPIOR
#define SLEEP
#define SLEEP_HARD

//********** USI UART Defines **********//

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define DR(a,b) (((a)+((b)/2))/(b))

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)

#define BIT_TIMER		DR(F_CPU/TIMER_PRESCALER,BAUDRATE)
#define HALF_BIT		DR(BIT_TIMER,2)

#define PCINT_DELAY		(0x11 / TIMER_PRESCALER)

#if PCINT_DELAY >= HALF_BIT
#error "Too much baud or not enough F_CPU"
#endif

#if BIT_TIMER > 256
#error "Not enough timer prescaler or too low baud or too high F_CPU"
#endif

#define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )

#ifdef URX_ENABLED
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
    #error RX buffer size is not a power of 2
#endif
#endif

#ifdef UTX_ENABLED
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
    #error TX buffer size is not a power of 2
#endif
#endif

/* General defines */
#define TRUE                      1
#define FALSE                     0

/* Timer configuration */
#if TIMER_PRESCALER == 1
#define TIMER_CS _BV(CS00)
#define TIMER_TST _BV(CS00)
#elif TIMER_PRESCALER == 8
#define TIMER_CS _BV(CS01)
#elif TIMER_PRESCALER == 64
#define TIMER_CS (_BV(CS01)|_BV(CS00))
#elif TIMER_PRESCALER == 256
#define TIMER_CS _BV(CS02)
#elif TIMER_PRESCALER == 1024
#define TIMER_CS (_BV(CS02) | _BV(CS00))
#else
#error "This timer prescaler wasnt possible when the code was written..."
#endif



//********** Static Variables **********//

#ifdef URX_ENABLED
static unsigned char          UART_RxBuf[UART_RX_BUFFER_SIZE];  // UART buffers. Size is definable in the header file.
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
#endif

#ifdef UTX_ENABLED
#ifdef USE_GPIOR
#define USI_UART_TxData GPIOR1
#else
static unsigned char USI_UART_TxData;
#endif
static unsigned char          UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;
#endif

/* Status flags */

#define OTX_fromBuff 0x01
#define OTX_ofPkg 0x02
#define ORX_ofPkg 0x04


#ifdef USE_GPIOR
#define USI_STATUS GPIOR0
#else
static volatile unsigned char USI_STATUS;
#endif

#define SET_STAT(x) do { USI_STATUS |= (x); } while(0)
#define CLR_STAT(x) do { USI_STATUS &= ~(x); } while(0)
#define TST_STAT(x) (USI_STATUS & (x))


//********** USI_UART functions **********//

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
static unsigned char Bit_Reverse( unsigned char x )
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
#ifdef INVERTED_IO
    return ~x;
#else
    return x;
#endif
}

// Flush the UART buffers.
void USI_UART_Flush_Buffers( void ) 
{
#ifdef URX_ENABLED
    UART_RxTail = 0;
    UART_RxHead = 0;
#endif

#ifdef UTX_ENABLED
    UART_TxTail = 0;
    UART_TxHead = 0;
#endif
}

#ifdef UTX_ENABLED
// Initialise USI for UART transmission.
static void USI_UART_Initialise_Transmitter( void )
{
    cli();
    TCNT0  = 0x00;
#if TIMER_PRESCALER != 1
    GTCCR |= _BV(PSR10);	// Note: you might need to check with your Timer1 application on if this is acceptable
#endif
    TCCR0B  = TIMER_CS;         // Reset the prescaler and start Timer0.

    USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
             (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
             (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 compare match as USI Clock source.
             (0<<USITC);
   // Make sure MSB is '1' before enabling USI_DO.
#ifdef INVERTED_IO
    USIDR = 0;
#else
    USIDR = 0xff;
#endif
    USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
             0x0F;                                            // Preload the USI counter to generate interrupt at first USI clock.
    DDRA  |= _BV(5);                                        // Configure USI_DO as output.

    SET_STAT(OTX_fromBuff);

    sei();
}


// Puts data in the transmission buffer, after reverseing the bits in the byte.
// Initiates the transmission rutines if not already started.
void USI_UART_Transmit_Byte( unsigned char data )
{
    unsigned char tmphead;

    tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;        // Calculate buffer index.
    while ( tmphead == UART_TxTail );                           // Wait for free space in buffer.
    UART_TxBuf[tmphead] = Bit_Reverse(data);                    // Reverse the order of the bits in the data byte and store data in buffer.
    UART_TxHead = tmphead;                                      // Store new index.

    if ( !TST_STAT(OTX_fromBuff) )    // Start transmission from buffer (if not already started).
    {
        while ( TST_STAT(ORX_ofPkg) ); // Wait for USI to finsh reading incoming data.
        USI_UART_Initialise_Transmitter();
    }
}
#endif

#ifdef URX_ENABLED

// Initialise USI for UART reception.
// Note that this function only enables pinchange interrupt on the USI Data Input pin.
// The USI is configured to read data within the pinchange interrupt.
static void USI_UART_Initialise_Receiver( void )
{
    USICR  =  0;                                            // Disable USI.
#ifdef INVERTED_IO
    USIDR = 0;
#else
    USIDR = 0xff;
#endif

    PCMSK0 = _BV(6);
    GIFR   =  (1<<PCIF0);                                    // Clear pin change interrupt flag.
    GIMSK |=  (1<<PCIE0);                                   // Enable pin change interrupt for DI/PA6/PCINT6.
}

static void USI_UART_Wait_Data(void)
{
#ifdef SLEEP
    if ( UART_RxHead != UART_RxTail ) return; // fast cli-lesss path
    while (1) { // manual exit
	cli();
#ifdef SLEEP_HARD
	// If timer is running then we depend on it.
	if (TCCR0B) set_sleep_mode(SLEEP_MODE_IDLE);
	else set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#endif
	if ( UART_RxHead == UART_RxTail ) {
	 	sleep_enable();
	 	sei();
	 	sleep_cpu();
	 	sleep_disable();
	} else {
		sei();
		return;
	}
   }
#else
    while ( UART_RxHead == UART_RxTail );                 // Wait for incomming data
#endif
}

// Returns a byte from the receive buffer. Waits if buffer is empty.
unsigned char USI_UART_Receive_Byte( void )
{
    unsigned char tmptail;
    USI_UART_Wait_Data();
    tmptail = ( UART_RxTail + 1 ) & UART_RX_BUFFER_MASK;  // Calculate buffer index
    UART_RxTail = tmptail;                                // Store new index
    return Bit_Reverse(UART_RxBuf[tmptail]);              // Reverse the order of the bits in the data byte before it returns data from the buffer.
}

// Check if there is data in the receive buffer.
unsigned char USI_UART_Data_In_Receive_Buffer( void )
{
    return ( UART_RxHead != UART_RxTail );                // Return 0 (FALSE) if the receive buffer is empty.
}


// ********** Interrupt Handlers ********** //

// The pin change interrupt is used to detect USI_UART reseption.
// It is here the USI is configured to sample the UART signal.
ISR(PCINT0_vect)
{
#ifdef INVERTED_IO
    if (PINA & _BV(6))
#else
    if (!( PINA & _BV(6) ))
#endif
    {                                                             //  was this pin that generated the pin change interrupt.
        TCNT0  = PCINT_DELAY + HALF_BIT;   // Plant TIMER0 seed to match baudrate (incl interrupt start up time.).
#if TIMER_PRESCALER != 1
        GTCCR |= _BV(PSR10);	    // Reset the prescaler ... (NB. If you're using Timer1 this might be ... not desirable.)
#endif
        TCCR0B  = TIMER_CS;         // and start Timer0.

        USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
                 (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
                 (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 compare match as USI Clock source.
                 (0<<USITC);
                                                                  // Note that enabling the USI will also disable the pin change interrupt.
        USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
                 USI_COUNTER_SEED_RECEIVE;                        // Preload the USI counter to generate interrupt.
        GIMSK &=  ~_BV(PCIE0);                                    // Disable pin change interrupt.

	SET_STAT(ORX_ofPkg);
    }
}
#endif

// The USI Counter Overflow interrupt is used for moving data between memmory and the USI data register.
// The interrupt is used for both transmission and reception.
ISR(USI_OVF_vect)
{

#ifdef UTX_ENABLED
    // Check if we are running in Transmit mode.
    if( TST_STAT(OTX_fromBuff) )
    {
        // If ongoing transmission, then send second half of transmit data.
        if( TST_STAT(OTX_ofPkg) )
        {
            CLR_STAT(OTX_ofPkg);    // Clear on-going package transmission flag.

            USISR = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);                 // Load USI Counter seed and clear all USI flags.
#ifdef INVERTED_IO
            USIDR = (USI_UART_TxData << 3);                      // Reload the USIDR with the rest of the data and a stop-bit.
#else
            USIDR = (USI_UART_TxData << 3) | 0x07;                      // Reload the USIDR with the rest of the data and a stop-bit.
#endif
        }
        // Else start sendinbg more data or leave transmit mode.
        else
        {
            // If there is data in the transmit buffer, then send first half of data.
            if ( UART_TxHead != UART_TxTail )
            {
                unsigned char tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;    // Calculate buffer index.

                UART_TxTail = tmptail;                                  // Store new index.
                USI_UART_TxData = UART_TxBuf[tmptail];                  // Read out the data that is to be sent. Note that the data must be bit reversed before sent.
                                                                        // The bit reversing is moved to the application section to save time within the interrupt.
                USISR  = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);            // Load USI Counter seed and clear all USI flags.
#ifdef INVERTED_IO
                USIDR  = (USI_UART_TxData >> 2) | 0x40;
#else
                USIDR  = (USI_UART_TxData >> 2) | 0x80;                 // Copy (initial high state,) start-bit and 6 LSB of original data (6 MSB
#endif                                                                  //  of bit of bit reversed data).
                SET_STAT(OTX_ofPkg); // Set on-going package transmission flag.

            }
            // Else enter receive mode.
            else
            {
            	CLR_STAT(OTX_fromBuff);

                TCCR0B  = 0;                 // Stop Timer0.
                DDRA  &= ~_BV(5);        // Set USI DO as input
#ifndef INVERTED_IO
		PORTA |= _BV(5); // set USI DO pullup
#else
		PORTA &= ~_BV(5); // unset USI DO pullup (btw, external hard-ish pull-down recommended if TX used with INVERTED_IO)
#endif
                USICR  =  0;                                            // Disable USI.
                GIFR   =  (1<<PCIF0);                                    // Clear pin change interrupt flag.
                GIMSK |=  (1<<PCIE0);                                   // Enable pin change interrupt for PB3:0.
            }
        }
    }

    // Else running in receive mode.
    else
#endif
    {
#ifdef URX_ENABLED
        unsigned char tmphead     = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK;        // Calculate buffer index.

        if ( tmphead != UART_RxTail )                                   // If buffer is full trash data and set buffer full flag.
        {
            UART_RxHead = tmphead;                                      // Store new index.
            UART_RxBuf[tmphead] = USIDR;                                // Store received data in buffer. Note that the data must be bit reversed before used. 
        }                                                               // The bit reversing is moved to the application section to save time within the interrupt.
#ifdef INVERTED_IO
	USIDR = 0;
#else
	USIDR = 0xff;
#endif
        TCCR0B  = 0;	// Stop Timer0
#ifndef INVERTED_IO
	PORTA |= _BV(5); // set USI DO pullup
#else
	PORTA &= ~_BV(5); // unset USI DO pullup (btw, external hard-ish pull-down recommended if TX used with INVERTED_IO)
#endif
        USICR  =  0;                                            // Disable USI.
        GIFR   =  (1<<PCIF0);                                    // Clear pin change interrupt flag.
        GIMSK |=  (1<<PCIE0);                                   // Enable pin change interrupt for PB3:0.
#endif
	CLR_STAT(ORX_ofPkg);

    }

}

void USI_UART_Init(void) {
    USI_STATUS = 0;
    TCCR0A = _BV(WGM01);
    TCCR0B = 0;
    OCR0A = (BIT_TIMER-1);
    DDRA &=  ~_BV(6);
    DDRA &= ~_BV(5);
#ifdef INVERTED_IO
    PORTA &= ~_BV(6);
    PORTA &= ~_BV(5);
#else
    PORTA |= _BV(6);
    PORTA |= _BV(5);
#endif

    USI_UART_Flush_Buffers();
#ifdef URX_ENABLED
    USI_UART_Initialise_Receiver();
#endif
}
