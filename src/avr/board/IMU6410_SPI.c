/**************************************************************

   Program Title:  IMU6410 Monitor

   Program Name:  IMU6410_SPI.c	   	  Version: 0.92

   Author:  SS	  					  Date:  June 11, 2010

   Compiled with AVR Studio 4.19.

   This file contains routines that implement an SPI Master controller
   for the IMU6410.

   Copyright 2004-2011, SOC Robotics, Inc.  All rights reserved.

****************************************************************/
/*
 * Copyright 2004-2011, SOC Robotics, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Redistribution is only allowed in combination with SOC Robotics,
 *    Inc. hardware products.
 *
 * THIS SOFTWARE IS PROVIDED BY SOC ROBOTICS, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SOC ROBOTICS,
 * INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.soc-robotics.com
 */

/*
 * $Log: IMU6410_SPI.c,v $
 * Revision 0.86  2006/02/07 16:40:20  SS
 *    2-wire interface, 8MHz clock, AVR Flash writer, XON/XOFF, 38,700baud
 * Revision 0.52  2005/06/04 18:36:42  SS
 * Preview release
 *
 */
// Include header files for GCC/IAR
//J #include "avr_compiler.h"
#include <avr/io.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//J #include "IMU6410_Uart.h"
//J #include "IMU6410_Defines.h"
//J #include "IMU6410_Extern.h"

// Initialize SPI Master - note that ADXL345 needs Mode 3
// and uSD usually needs Mode 0 but most SD disk seem to
// to work fine with Mode 3 so leave as Mode 3.  Also note
// that uSD's don't like SPI receive with transmit register
// containing 0x00 - set to 0xFF - otherwise CMD8 returns 0x82.
void Init_SPI_Master(void)
{
    // Set MOSI, SCK and SS output, all others input
	//   PB7-SCK, PB6-MISO, PB5-MOSI, PB4-SS, PB3-uSD CS
	DDRB |= ((1<<PB7)|(1<<PB5)|(1<<PB4)|(1<<PB3));
	PORTB |= 0xBC;  // Set bits MOSI & SCK

	// Enable SPI, Master, set clock rate fck
	//    Select SPI Mode 3 - CPOL=1, CPHA=1  clk/8
//	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR0);
	//    Select SPI Mode 3 - CPOL=1, CPHA=1  clk/2
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);

    //  Select SPI Mode 0 - CPOL=0, CPHA=0  clk/64
//	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);
    //  Select SPI Mode 0 - CPOL=0, CPHA=0  clk/2
//	SPCR = (1<<SPE)|(1<<MSTR);
    SPSR = (1 << SPI2X);

	// Chan's code
//	SPCR = 0x50;
//    SPSR = (1 << SPI2X);
}

//  Free SPI pins
void Release_SPI(void)
{
	// Disable SPI, Master
	SPCR = 0x00;

 	// Set MOSI and SCK input, all others as input or output
	DDRB &= ~((1<<PB7)|(1<<PB5));
}

// TODO - break if no reception
void SPI_MasterTransmit(unsigned char cData)
{
    volatile long cnt;

	cnt = 0;
    // Start transmisstion
	SPDR = cData;
	// Wait for transmission to complete
	while(!(SPSR&0x80)) ;
	cData = SPDR;  // clear SPIF flag
}

// TODO - break if no reception
unsigned char SPI_MasterReceive(void)
{
    // Wait for reception to complete
//	SPDR = 0x00;
	SPDR = 0xFF;
	while(!(SPSR & 0x80)) ;

	return SPDR;
}
