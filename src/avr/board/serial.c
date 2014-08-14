/*
 * Serial.c
 *
 * Created: 27/06/2012 18:34:03
 *  Author: Juan
 */

#include "serial.h"
#include "serial_buffer.h"
#include "avr_io.h"
#include <avr/interrupt.h>
//#if DEBUG_SERIAL || DEBUG_SERIAL_WRITE
#include <stdio.h>
//#endif
#include <stdbool.h>


// Standard XON/XOFF flow control special codes.
#define XON 0x11
#define XOFF 0x13

// USART0 Rx Complete - interrupt handler.
// Executed when the RXC0 bit (USART Receive Complete) is set: there is unread data present in the USART Receive Buffer.
// The RXC0 bit is cleared when the received data in UDR0 is read.
// ISR(USART0_RX_vect);

// USART0 Tx Complete - interrupt handler.
// Executed when the UDRE0 bit (USART Data Register Empty) is set: the USART Transmit Buffer is ready to receive new data to be sent.
// The UDRE0 bit is cleared when new data is written into UDR0.
// ISR(USART0_UDRE_vect);

static bool send_xoff;
static bool xoff_sent;
static SerialBuffer RxBuf;
static SerialBuffer TxBuf;


void serial_init(uint8_t baud)
{
	send_xoff = false;
	xoff_sent = false;
	sb_init(&RxBuf);
	sb_init(&TxBuf);

	// UART0 configuration.
	BIT_CLEAR(PRR0, PRUSART0);  // Disable Power Reduction.
	BIT_SET(UCSR0B, RXCIE0);  // "Rx Complete" Interrupt Enable.
	BIT_SET(UCSR0B, RXEN0);  // Receiver Enable.
	BIT_SET(UCSR0B, TXEN0);  // Transmitter Enable.
	
	BIT_CLEAR(UCSR0C, UMSEL01);
	BIT_CLEAR(UCSR0C, UMSEL00);  // Mode: Asynchronous.
	
	BIT_CLEAR(UCSR0C, UPM01);
	BIT_CLEAR(UCSR0C, UPM00);  // Parity: None.
	
	BIT_CLEAR(UCSR0C, USBS0);  // Stop Bits: 1.
	
	BIT_CLEAR(UCSR0B, UCSZ02);
	BIT_SET(UCSR0C, UCSZ01);
	BIT_SET(UCSR0C, UCSZ00);  // Character Size: 8 data bits.
	
	UBRR0H = (uint8_t)(baud >> 8);
	UBRR0L = (uint8_t)baud;  // Set baud rate.
	
	// UART1 configuration.
	BIT_SET(PRR0, PRUSART1);  // Enable Power Reduction.
}

uint16_t serial_available(void)
{
#if DEBUG_SERIAL
	uint16_t ret = sb_count_used(&RxBuf);
	if (ret > 0)
		printf("SERIAL: serial_available(): %u\r\n", ret);
	return ret;
#else
	return sb_count_used(&RxBuf);
#endif
}

uint8_t serial_read(void)
{
#if DEBUG_SERIAL
	printf("SERIAL: serial_read(): wait for input\r\n");
#endif
	// Wait for some data to arrive at RxBuffer.
	// Use "volatile" to avoid removal by optimization.
	while (sb_is_empty((SerialBuffer *)(volatile SerialBuffer *)&RxBuf));  // BLOCK here.

	// Get data from RxBuffer.
#if DEBUG_SERIAL
	uint8_t ret = sb_read(&RxBuf);
	printf("SERIAL: serial_read(): %#x ('%c')\r\n", ret, ret);
	return ret;
#else
	return sb_read(&RxBuf);
#endif
}

// WARNING TODO - deadlock if (size > SB_BUFFER_SIZE-1).
uint16_t serial_write(const uint8_t* data, uint16_t size)
{
	// WARNING
	// Be careful when printing debug text from this function:
	// all characters will arrive at once, and might exhaust the transmit buffer.

#if DEBUG_SERIAL || DEBUG_SERIAL_WRITE
	static volatile bool print = true;
#endif

#if DEBUG_SERIAL_WRITE
	if (print) {
		print = false;
		printf("-%u\r\n", sb_count_free(&TxBuf));
		print = true;
	}
#endif

	if (size > SB_BUFFER_SIZE-1) {
		// Serial Buffer Excess!!
		// This situation is a no-no.
		printf("SBE\r\n");
		return 0;
	}

	// If buffer is getting full, try to warn.
	// Sing with me: "sending out an S.O.S. ..."
	if (sb_count_free(&TxBuf) < 32) {
		printf("MIB\r\n");
	}

	// The worst happened: wait for free space in TxBuffer.
	// Use "volatile" to avoid removal by optimization.
	while (sb_count_free((SerialBuffer *)(volatile SerialBuffer *)&TxBuf) < size);  // BLOCK here.

	// Store data to be transmitted in TxBuffer.
	for (uint16_t i = 0; i < size; i++) {
		sb_write(&TxBuf, data[i]);
	}
	BIT_SET(UCSR0B, UDRIE0);  // "Tx Complete" Interrupt Enable.

	return size;
}

ISR(USART0_RX_vect)
{
	volatile uint8_t data;

	// Receiver Error Flags.
	// Must be read before the USART Receive Buffer (UDR0).
	data = UCSR0A;

	// Check error conditions.
	//if (bit_is_set(data, FE0))
	// Frame Error: the stop bit was incorrect (zero).
	//if (bit_is_set(data, DOR0))
	// Data OverRun: the USART Receive Buffer is full (two characters).

	// Read the received data from UDR0. This also clears the RXC0 flag.
	data = UDR0;

	// Wait for free space in RxBuffer.
	// while (cb_is_full(&RxBuf));  // BLOCK here.
	// XON/XOFF protocol should avoid this situation.

	// Store data received in RxBuffer.
	sb_write(&RxBuf, data);

	// If RxBuffer is almost full, send XOFF command.
	if ((sb_count_free(&RxBuf) < 16) && !xoff_sent) {
		send_xoff = true;  // Tell "Tx Complete" interrupt to send XOFF.
		BIT_SET(UCSR0B, UDRIE0);  // "Tx Complete" Interrupt Enable.
	}
}

ISR(USART0_UDRE_vect)
{
	if (send_xoff && !xoff_sent) {
		// We were told to send XOFF.
		UDR0 = XOFF;
		send_xoff = false;
		xoff_sent = true;
	}
	else if (xoff_sent && (sb_count_used(&RxBuf) < 8)) {
		// If RxBuffer is almost empty after sending XOFF: send XON.
		UDR0 = XON;
		xoff_sent = false;
	}
	else {
		// No XON/XOFF is being sent: normal operation.
		if (!sb_is_empty(&TxBuf)) {
			// There is data waiting for transmission.

			// Send data from TxBuffer.
			UDR0 = sb_read(&TxBuf);
		}
		else {
			// There is no data waiting for transmission.
			BIT_CLEAR(UCSR0B, UDRIE0);  // "Tx Complete" Interrupt Disable.
		}
	}
}
