#ifndef UART_H
#define UART_H


/************************************************************************
Title:    Interrupt UART library with receive/transmit circular buffers
Author:   Andy Gock
Software: AVR-GCC 4.1, AVR Libc 1.4
Hardware: any AVR with built-in UART, tested on AT90S8515 & ATmega8 at 4 Mhz
License:  GNU General Public License 
Usage:    see Doxygen manual

Based on original library by Peter Fluery, Tim Sharpe, Nicholas Zambetti.

https://github.com/andygock/avr-uart

LICENSE:
	Copyright (C) 2012 Andy Gock

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
************************************************************************/

/************************************************************************
uart_available, uart_flush, uart1_available, and uart1_flush functions
were adapted from the Arduino HardwareSerial.h library by Tim Sharpe on 
11 Jan 2009.  The license info for HardwareSerial.h is as follows:

  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
************************************************************************/

/************************************************************************
Changelog for modifications made by Tim Sharpe, starting with the current
  library version on his Web site as of 05/01/2009. 

Date        Description
=========================================================================
05/12/2009  Added Arduino-style available() and flush() functions for both
			supported UARTs.  Really wanted to keep them out of the library, so
			that it would be as close as possible to Peter Fleury's original
			library, but has scoping issues accessing internal variables from
			another program.  Go C!

************************************************************************/

/** 
 *  @defgroup avr-uart UART Library
 *  @code #include <uart.h> @endcode
 * 
 *  @brief Interrupt UART library using the built-in UART with transmit and receive circular buffers. 
 *
 *  This library can be used to transmit and receive data through the built in UART. 
 *
 *  An interrupt is generated when the UART has finished transmitting or
 *  receiving a byte. The interrupt handling routines use circular buffers
 *  for buffering received and transmitted data.
 *
 *  The UART_RXn_BUFFER_SIZE and UART_TXn_BUFFER_SIZE constants define
 *  the size of the circular buffers in bytes. Note that these constants must be a power of 2.
 *
 *  You need to define these buffer sizes in uart.h
 *
 *  @note Based on Atmel Application Note AVR306
 *  @author Andy Gock <andy@gock.net>
 *  @note Based on original library by Peter Fleury and Tim Sharpe.
 */
 
/**@{*/
#include <stdint.h>
#include <avr/io.h>

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

/*
 * constants and macros
 */



/* Enable USART 1, 2, 3 as required */
#define USART0_ENABLED
//#define USART1_ENABLED
//#define USART2_ENABLED 
//#define USART3_ENABLED

/* Set size of receive and transmit buffers */

#ifndef UART_RX0_BUFFER_SIZE
	#define UART_RX0_BUFFER_SIZE 256 /**< Size of the circular receive buffer, must be power of 2 */
#endif
#ifndef UART_RX1_BUFFER_SIZE
	#define UART_RX1_BUFFER_SIZE 128 /**< Size of the circular receive buffer, must be power of 2 */
#endif
#ifndef UART_RX2_BUFFER_SIZE
	#define UART_RX2_BUFFER_SIZE 128 /**< Size of the circular receive buffer, must be power of 2 */
#endif
#ifndef UART_RX3_BUFFER_SIZE
	#define UART_RX3_BUFFER_SIZE 128 /**< Size of the circular receive buffer, must be power of 2 */
#endif

#ifndef UART_TX0_BUFFER_SIZE
	#define UART_TX0_BUFFER_SIZE 128 /**< Size of the circular transmit buffer, must be power of 2 */
#endif
#ifndef UART_TX1_BUFFER_SIZE
	#define UART_TX1_BUFFER_SIZE 128 /**< Size of the circular transmit buffer, must be power of 2 */
#endif
#ifndef UART_TX2_BUFFER_SIZE
	#define UART_TX2_BUFFER_SIZE 128 /**< Size of the circular transmit buffer, must be power of 2 */
#endif
#ifndef UART_TX3_BUFFER_SIZE
	#define UART_TX3_BUFFER_SIZE 128 /**< Size of the circular transmit buffer, must be power of 2 */
#endif

/* Check buffer sizes are not too large for 8-bit positioning */

#if (UART_RX0_BUFFER_SIZE > 256 & !defined(USART0_LARGE_BUFFER))
	#error "Buffer too large, please use -DUSART0_LARGE_BUFFER switch in compiler options"
#endif

#if (UART_RX1_BUFFER_SIZE > 256 & !defined(USART1_LARGE_BUFFER))
	#error "Buffer too large, please use -DUSART1_LARGE_BUFFER switch in compiler options"
#endif

#if (UART_RX2_BUFFER_SIZE > 256 & !defined(USART2_LARGE_BUFFER))
	#error "Buffer too large, please use -DUSART2_LARGE_BUFFER switch in compiler options"
#endif

#if (UART_RX3_BUFFER_SIZE > 256 & !defined(USART3_LARGE_BUFFER))
	#error "Buffer too large, please use -DUSART3_LARGE_BUFFER switch in compiler options"
#endif

/* Check buffer sizes are not too large for *_LARGE_BUFFER operation (16-bit positioning) */

#if (UART_RX0_BUFFER_SIZE > 65536)
	#error "Buffer too large, maximum allowed is 65536 bytes"
#endif

#if (UART_RX1_BUFFER_SIZE > 65536)
	#error "Buffer too large, maximum allowed is 65536 bytes"
#endif

#if (UART_RX2_BUFFER_SIZE > 65536)
	#error "Buffer too large, maximum allowed is 65536 bytes"
#endif

#if (UART_RX3_BUFFER_SIZE > 65536)
	#error "Buffer too large, maximum allowed is 65536 bytes"
#endif

/** @brief  UART Baudrate Expression
 *  @param  xtalCpu  system clock in Mhz, e.g. 4000000L for 4Mhz          
 *  @param  baudRate baudrate in bps, e.g. 1200, 2400, 9600     
 */
#define UART_BAUD_SELECT(baudRate,xtalCpu) (((xtalCpu)+8UL*(baudRate))/(16UL*(baudRate))-1UL)

/** @brief  UART Baudrate Expression for ATmega double speed mode
 *  @param  xtalCpu  system clock in Mhz, e.g. 4000000L for 4Mhz           
 *  @param  baudRate baudrate in bps, e.g. 1200, 2400, 9600     
 */
#define UART_BAUD_SELECT_DOUBLE_SPEED(baudRate,xtalCpu) ((((xtalCpu)+4UL*(baudRate))/(8UL*(baudRate))-1)|0x8000)

/* test if the size of the circular buffers fits into SRAM */

#if defined(USART0_ENABLED) && ( (UART_RX0_BUFFER_SIZE+UART_TX0_BUFFER_SIZE) >= (RAMEND-0x60 ) )
#error "size of UART_RX0_BUFFER_SIZE + UART_TX0_BUFFER_SIZE larger than size of SRAM"
#endif

#if defined(USART1_ENABLED) && ( (UART_RX1_BUFFER_SIZE+UART_TX1_BUFFER_SIZE) >= (RAMEND-0x60 ) )
#error "size of UART_RX1_BUFFER_SIZE + UART_TX1_BUFFER_SIZE larger than size of SRAM"
#endif

#if defined(USART2_ENABLED) && ( (UART_RX2_BUFFER_SIZE+UART_RX2_BUFFER_SIZE) >= (RAMEND-0x60 ) )
#error "size of UART_RX2_BUFFER_SIZE + UART_TX2_BUFFER_SIZE larger than size of SRAM"
#endif

#if defined(USART3_ENABLED) && ( (UART_RX3_BUFFER_SIZE+UART_RX3_BUFFER_SIZE) >= (RAMEND-0x60 ) )
#error "size of UART_RX3_BUFFER_SIZE + UART_TX3_BUFFER_SIZE larger than size of SRAM"
#endif

/* 
** high byte error return code of uart_getc()
*/
#define UART_FRAME_ERROR      0x0800              /**< Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400              /**< Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200              /**< receive ringbuffer overflow */
#define UART_NO_DATA          0x0100              /**< no receive data available   */

/* Macros, to allow use of legacy names */

#define uart_init(b)      uart0_init(b)
#define uart_getc()       uart0_getc()
#define uart_putc(d)      uart0_putc(d)
#define uart_puts(s)      uart0_puts(s)
#define uart_puts_p(s)    uart0_puts_p(s)
#define uart_available()  uart0_available()
#define uart_flush()      uart0_flush()


/*
 *  constants and macros
 */

/* size of RX/TX buffers */
#define UART_RX0_BUFFER_MASK ( UART_RX0_BUFFER_SIZE - 1)
#define UART_RX1_BUFFER_MASK ( UART_RX1_BUFFER_SIZE - 1)
#define UART_RX2_BUFFER_MASK ( UART_RX2_BUFFER_SIZE - 1)
#define UART_RX3_BUFFER_MASK ( UART_RX3_BUFFER_SIZE - 1)

#define UART_TX0_BUFFER_MASK ( UART_TX0_BUFFER_SIZE - 1)
#define UART_TX1_BUFFER_MASK ( UART_TX1_BUFFER_SIZE - 1)
#define UART_TX2_BUFFER_MASK ( UART_TX2_BUFFER_SIZE - 1)
#define UART_TX3_BUFFER_MASK ( UART_TX3_BUFFER_SIZE - 1)

#if ( UART_RX0_BUFFER_SIZE & UART_RX0_BUFFER_MASK )
	#error RX0 buffer size is not a power of 2
#endif
#if ( UART_TX0_BUFFER_SIZE & UART_TX0_BUFFER_MASK )
	#error TX0 buffer size is not a power of 2
#endif

#if ( UART_RX1_BUFFER_SIZE & UART_RX1_BUFFER_MASK )
	#error RX1 buffer size is not a power of 2
#endif
#if ( UART_TX1_BUFFER_SIZE & UART_TX1_BUFFER_MASK )
	#error TX1 buffer size is not a power of 2
#endif

#if ( UART_RX2_BUFFER_SIZE & UART_RX2_BUFFER_MASK )
	#error RX2 buffer size is not a power of 2
#endif
#if ( UART_TX2_BUFFER_SIZE & UART_TX2_BUFFER_MASK )
	#error TX2 buffer size is not a power of 2
#endif

#if ( UART_RX3_BUFFER_SIZE & UART_RX3_BUFFER_MASK )
	#error RX3 buffer size is not a power of 2
#endif
#if ( UART_TX3_BUFFER_SIZE & UART_TX3_BUFFER_MASK )
	#error TX3 buffer size is not a power of 2
#endif

#if defined(__AVR_AT90S2313__) \
 || defined(__AVR_AT90S4414__) || defined(__AVR_AT90S4434__) \
 || defined(__AVR_AT90S8515__) || defined(__AVR_AT90S8535__) \
 || defined(__AVR_ATmega103__)
	/* old AVR classic or ATmega103 with one UART */
	#define AT90_UART
	#define UART0_RECEIVE_INTERRUPT   UART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
	#define UART0_STATUS   USR
	#define UART0_CONTROL  UCR
	#define UART0_DATA     UDR
	#define UART0_UDRIE    UDRIE
#elif defined(__AVR_AT90S2333__) || defined(__AVR_AT90S4433__)
	/* old AVR classic with one UART */
	#define AT90_UART
	#define UART0_RECEIVE_INTERRUPT   UART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
	#define UART0_STATUS   UCSRA
	#define UART0_CONTROL  UCSRB
	#define UART0_DATA     UDR
	#define UART0_UDRIE    UDRIE
#elif  defined(__AVR_ATmega8__)  || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) \
  || defined(__AVR_ATmega323__)
	/* ATmega with one USART */
	#define ATMEGA_USART
	#define UART0_RECEIVE_INTERRUPT   USART_RXC_vect
	#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
	#define UART0_STATUS   UCSRA
	#define UART0_CONTROL  UCSRB
	#define UART0_DATA     UDR
	#define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega16U4__) || \
      defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega32U6__)
	/* ATmega with one USART, but is called USART1 (untested) */
	#define ATMEGA_USART1
	#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
	#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
	#define UART1_STATUS   UCSR1A
	#define UART1_CONTROL  UCSR1B
	#define UART1_DATA     UDR1
	#define UART1_UDRIE    UDRIE1
#elif  defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__)
	/* ATmega with one USART */
	#define ATMEGA_USART
	#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
	#define UART0_STATUS   UCSRA
	#define UART0_CONTROL  UCSRB
	#define UART0_DATA     UDR
	#define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega163__)
	/* ATmega163 with one UART */
	#define ATMEGA_UART
	#define UART0_RECEIVE_INTERRUPT   UART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
	#define UART0_STATUS   UCSRA
	#define UART0_CONTROL  UCSRB
	#define UART0_DATA     UDR
	#define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega162__)
	/* ATmega with two USART */
	#define ATMEGA_USART0
	#define ATMEGA_USART1
	#define UART0_RECEIVE_INTERRUPT   USART0_RXC_vect
	#define UART1_RECEIVE_INTERRUPT   USART1_RXC_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
	#define UART1_STATUS   UCSR1A
	#define UART1_CONTROL  UCSR1B
	#define UART1_DATA     UDR1
	#define UART1_UDRIE    UDRIE1
#elif defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
	/* ATmega with two USART */
	#define ATMEGA_USART0
	#define ATMEGA_USART1
	#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
	#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
	#define UART1_STATUS   UCSR1A
	#define UART1_CONTROL  UCSR1B
	#define UART1_DATA     UDR1
	#define UART1_UDRIE    UDRIE1
#elif defined(__AVR_ATmega161__)
	/* ATmega with UART */
	#error "AVR ATmega161 currently not supported by this libaray !"
#elif defined(__AVR_ATmega169__)
	/* ATmega with one USART */
	#define ATMEGA_USART
	#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART0_STATUS   UCSRA
	#define UART0_CONTROL  UCSRB
	#define UART0_DATA     UDR
	#define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega48__) ||defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega48P__) ||defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
      defined(__AVR_ATmega328P__)
	/* TLS-Added 48P/88P/168P/328P */
	/* ATmega with one USART */
	#define ATMEGA_USART0
	#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)
	#define ATMEGA_USART
	#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
	#define UART0_STATUS   UCSRA
	#define UART0_CONTROL  UCSRB
	#define UART0_DATA     UDR
	#define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega329__) ||\
      defined(__AVR_ATmega649__) ||\
      defined(__AVR_ATmega325__) ||defined(__AVR_ATmega3250__) ||\
      defined(__AVR_ATmega645__) ||defined(__AVR_ATmega6450__)
	/* ATmega with one USART */
	#define ATMEGA_USART0
	#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATmega3290__) ||\
      defined(__AVR_ATmega6490__)
	/* TLS-Separated these two from the previous group because of inconsistency in the USART_RX */
	/* ATmega with one USART */
	#define ATMEGA_USART0
	#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__)
	/* ATmega with four USART */
	#define ATMEGA_USART0
	#define ATMEGA_USART1
	#define ATMEGA_USART2
	#define ATMEGA_USART3
	#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
	#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
	#define UART2_RECEIVE_INTERRUPT   USART2_RX_vect
	#define UART3_RECEIVE_INTERRUPT   USART3_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
	#define UART2_TRANSMIT_INTERRUPT  USART2_UDRE_vect
	#define UART3_TRANSMIT_INTERRUPT  USART3_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
	#define UART1_STATUS   UCSR1A
	#define UART1_CONTROL  UCSR1B
	#define UART1_DATA     UDR1
	#define UART1_UDRIE    UDRIE1
	#define UART2_STATUS   UCSR2A
	#define UART2_CONTROL  UCSR2B
	#define UART2_DATA     UDR2
	#define UART2_UDRIE    UDRIE2
	#define UART3_STATUS   UCSR3A
	#define UART3_CONTROL  UCSR3B
	#define UART3_DATA     UDR3
	#define UART3_UDRIE    UDRIE3
#elif defined(__AVR_ATmega644__)
	/* ATmega with one USART */
	#define ATMEGA_USART0
	#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATmega2561__) || defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || \
      defined(__AVR_ATmega1284P__)
	/* ATmega with two USART */
	#define ATMEGA_USART0
	#define ATMEGA_USART1
	#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
	#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
	#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
	#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
	#define UART0_STATUS   UCSR0A
	#define UART0_CONTROL  UCSR0B
	#define UART0_DATA     UDR0
	#define UART0_UDRIE    UDRIE0
	#define UART1_STATUS   UCSR1A
	#define UART1_CONTROL  UCSR1B
	#define UART1_DATA     UDR1
	#define UART1_UDRIE    UDRIE1
#else
	#error "no UART definition for MCU available"
#endif

/*
 *  Module global variables
 */

#if defined( USART0_ENABLED )
	#if defined( ATMEGA_USART ) || defined( ATMEGA_USART0 )
		static volatile uint8_t UART_TxBuf[UART_TX0_BUFFER_SIZE];
		static volatile uint8_t UART_RxBuf[UART_RX0_BUFFER_SIZE];

		#if defined( USART0_LARGE_BUFFER )
			static volatile uint16_t UART_TxHead;
			static volatile uint16_t UART_TxTail;
			static volatile uint16_t UART_RxHead;
			static volatile uint16_t UART_RxTail;
			static volatile uint8_t UART_LastRxError;
		#else
			static volatile uint8_t UART_TxHead;
			static volatile uint8_t UART_TxTail;
			static volatile uint8_t UART_RxHead;
			static volatile uint8_t UART_RxTail;
			static volatile uint8_t UART_LastRxError;
		#endif

	#endif
#endif

#if defined( USART1_ENABLED )
	#if defined( ATMEGA_USART1 )
		static volatile uint8_t UART1_TxBuf[UART_TX1_BUFFER_SIZE];
		static volatile uint8_t UART1_RxBuf[UART_RX1_BUFFER_SIZE];

		#if defined( USART1_LARGE_BUFFER )
			static volatile uint16_t UART1_TxHead;
			static volatile uint16_t UART1_TxTail;
			static volatile uint16_t UART1_RxHead;
			static volatile uint16_t UART1_RxTail;
			static volatile uint8_t UART1_LastRxError;
		#else
			static volatile uint8_t UART1_TxHead;
			static volatile uint8_t UART1_TxTail;
			static volatile uint8_t UART1_RxHead;
			static volatile uint8_t UART1_RxTail;
			static volatile uint8_t UART1_LastRxError;
		#endif
	#endif
#endif

#if defined( USART2_ENABLED )
	#if defined( ATMEGA_USART2 )
		static volatile uint8_t UART2_TxBuf[UART_TX2_BUFFER_SIZE];
		static volatile uint8_t UART2_RxBuf[UART_RX2_BUFFER_SIZE];

		#if defined( USART2_LARGE_BUFFER )
			static volatile uint16_t UART2_TxHead;
			static volatile uint16_t UART2_TxTail;
			static volatile uint16_t UART2_RxHead;
			static volatile uint16_t UART2_RxTail;
			static volatile uint8_t UART2_LastRxError;
		#else
			static volatile uint8_t UART2_TxHead;
			static volatile uint8_t UART2_TxTail;
			static volatile uint8_t UART2_RxHead;
			static volatile uint8_t UART2_RxTail;
			static volatile uint8_t UART2_LastRxError;
		#endif
	#endif
#endif

#if defined( USART3_ENABLED )
	#if defined( ATMEGA_USART3 )
		static volatile uint8_t UART3_TxBuf[UART_TX3_BUFFER_SIZE];
		static volatile uint8_t UART3_RxBuf[UART_RX3_BUFFER_SIZE];

		#if defined( USART3_LARGE_BUFFER )
			static volatile uint16_t UART3_TxHead;
			static volatile uint16_t UART3_TxTail;
			static volatile uint16_t UART3_RxHead;
			static volatile uint16_t UART3_RxTail;
			static volatile uint8_t UART3_LastRxError;
		#else
			static volatile uint8_t UART3_TxHead;
			static volatile uint8_t UART3_TxTail;
			static volatile uint8_t UART3_RxHead;
			static volatile uint8_t UART3_RxTail;
			static volatile uint8_t UART3_LastRxError;
		#endif

	#endif
#endif





/*
** function prototypes
*/

/**
   @brief   Initialize UART and set baudrate 
   @param   baudrate Specify baudrate using macro UART_BAUD_SELECT()
   @return  none
*/
extern void uart0_init(uint16_t baudrate);


/**
 *  @brief   Get received byte from ringbuffer
 *
 * Returns in the lower byte the received character and in the 
 * higher byte the last receive error.
 * UART_NO_DATA is returned when no data is available.
 *
 *  @return  lower byte:  received byte from ringbuffer
 *  @return  higher byte: last receive status
 *           - \b 0 successfully received data from UART
 *           - \b UART_NO_DATA           
 *             <br>no receive data available
 *           - \b UART_BUFFER_OVERFLOW   
 *             <br>Receive ringbuffer overflow.
 *             We are not reading the receive buffer fast enough, 
 *             one or more received character have been dropped 
 *           - \b UART_OVERRUN_ERROR     
 *             <br>Overrun condition by UART.
 *             A character already present in the UART UDR register was 
 *             not read by the interrupt handler before the next character arrived,
 *             one or more received characters have been dropped.
 *           - \b UART_FRAME_ERROR       
 *             <br>Framing Error by UART
 */
extern uint16_t uart0_getc(void);

/**
 *  @brief   Peek at next byte in ringbuffer
 *
 * Returns the next byte (character) of incoming UART data without removing it from the
 * internal ring buffer. That is, successive calls to uartN_peek() will return the same
 * character, as will the next call to uartN_getc().
 *
 * UART_NO_DATA is returned when no data is available.
 *
 *  @return  lower byte:  next byte in ringbuffer
 *  @return  higher byte: last receive status
 *           - \b 0 successfully received data from UART
 *           - \b UART_NO_DATA           
 *             <br>no receive data available
 *           - \b UART_BUFFER_OVERFLOW   
 *             <br>Receive ringbuffer overflow.
 *             We are not reading the receive buffer fast enough, 
 *             one or more received character have been dropped 
 *           - \b UART_OVERRUN_ERROR     
 *             <br>Overrun condition by UART.
 *             A character already present in the UART UDR register was 
 *             not read by the interrupt handler before the next character arrived,
 *             one or more received characters have been dropped.
 *           - \b UART_FRAME_ERROR       
 *             <br>Framing Error by UART
 */
extern uint16_t uart0_peek(void);

/**
 *  @brief   Put byte to ringbuffer for transmitting via UART
 *  @param   data byte to be transmitted
 *  @return  none
 */
extern void uart0_putc(uint8_t data);


/**
 *  @brief   Put string to ringbuffer for transmitting via UART
 *
 *  The string is buffered by the uart library in a circular buffer
 *  and one character at a time is transmitted to the UART using interrupts.
 *  Blocks if it can not write the whole string into the circular buffer.
 * 
 *  @param   s string to be transmitted
 *  @return  none
 */
extern void uart0_puts(const char *s );


/**
 * @brief    Put string from program memory to ringbuffer for transmitting via UART.
 *
 * The string is buffered by the uart library in a circular buffer
 * and one character at a time is transmitted to the UART using interrupts.
 * Blocks if it can not write the whole string into the circular buffer.
 *
 * @param    s program memory string to be transmitted
 * @return   none
 * @see      uart0_puts_P
 */
extern void uart0_puts_p(const char *s );

/**
 * @brief    Macro to automatically put a string constant into program memory
 * \param    __s string in program memory
 */
#define uart_puts_P(__s)       uart0_puts_p(PSTR(__s))
#define uart0_puts_P(__s)      uart0_puts_p(PSTR(__s))

/**
 *  @brief   Return number of bytes waiting in the receive buffer
 *  @return  bytes waiting in the receive buffer
 */
extern uint16_t uart0_available(void);

/**
 *  @brief   Flush bytes waiting in receive buffer
 */
extern void uart0_flush(void);


/** @brief  Initialize USART1 (only available on selected ATmegas) @see uart_init */
extern void uart1_init(uint16_t baudrate);
/** @brief  Get received byte of USART1 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern uint16_t uart1_getc(void);
/** @brief  Peek at next byte in USART1 ringbuffer */
extern uint16_t uart1_peek(void);
/** @brief  Put byte to ringbuffer for transmitting via USART1 (only available on selected ATmega) @see uart_putc */
extern void uart1_putc(uint8_t data);
/** @brief  Put string to ringbuffer for transmitting via USART1 (only available on selected ATmega) @see uart_puts */
extern void uart1_puts(const char *s );
/** @brief  Put string from program memory to ringbuffer for transmitting via USART1 (only available on selected ATmega) @see uart_puts_p */
extern void uart1_puts_p(const char *s );
/** @brief  Macro to automatically put a string constant into program memory */
#define uart1_puts_P(__s)       uart1_puts_p(PSTR(__s))
/** @brief   Return number of bytes waiting in the receive buffer */
extern uint16_t uart1_available(void);
/** @brief   Flush bytes waiting in receive buffer */
extern void uart1_flush(void);


/** @brief  Initialize USART2 (only available on selected ATmegas) @see uart_init */
extern void uart2_init(uint16_t baudrate);
/** @brief  Get received byte of USART2 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern uint16_t uart2_getc(void);
/** @brief  Peek at next byte in USART2 ringbuffer */
extern uint16_t uart2_peek(void);
/** @brief  Put byte to ringbuffer for transmitting via USART2 (only available on selected ATmega) @see uart_putc */
extern void uart2_putc(uint8_t data);
/** @brief  Put string to ringbuffer for transmitting via USART2 (only available on selected ATmega) @see uart_puts */
extern void uart2_puts(const char *s );
/** @brief  Put string from program memory to ringbuffer for transmitting via USART2 (only available on selected ATmega) @see uart_puts_p */
extern void uart2_puts_p(const char *s );
/** @brief  Macro to automatically put a string constant into program memory */
#define uart2_puts_P(__s)       uart2_puts_p(PSTR(__s))
/** @brief   Return number of bytes waiting in the receive buffer */
extern uint16_t uart2_available(void);
/** @brief   Flush bytes waiting in receive buffer */
extern void uart2_flush(void);


/** @brief  Initialize USART3 (only available on selected ATmegas) @see uart_init */
extern void uart3_init(uint16_t baudrate);
/** @brief  Get received byte of USART3 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern uint16_t uart3_getc(void);
/** @brief  Peek at next byte in USART3 ringbuffer */
extern uint16_t uart3_peek(void);
/** @brief  Put byte to ringbuffer for transmitting via USART3 (only available on selected ATmega) @see uart_putc */
extern void uart3_putc(uint8_t data);
/** @brief  Put string to ringbuffer for transmitting via USART3 (only available on selected ATmega) @see uart_puts */
extern void uart3_puts(const char *s );
/** @brief  Put string from program memory to ringbuffer for transmitting via USART3 (only available on selected ATmega) @see uart_puts_p */
extern void uart3_puts_p(const char *s );
/** @brief  Macro to automatically put a string constant into program memory */
#define uart3_puts_P(__s)       uart3_puts_p(PSTR(__s))
/** @brief   Return number of bytes waiting in the receive buffer */
extern uint16_t uart3_available(void);
/** @brief   Flush bytes waiting in receive buffer */
extern void uart3_flush(void);

/**@}*/

#endif // UART_H 

