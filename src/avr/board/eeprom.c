/*
 * eeprom.c
 *
 * Created: 29/06/2012 17:36:47
 *  Author: Juan
 */

#include "eeprom.h"
#include "avr_io.h"
#include <avr/interrupt.h>


uint8_t eeprom_read(uint16_t uiAddress)
{
	volatile uint8_t inbyte;

	// Wait for completion of previous write
	while(EECR&(1<<EEPE)) ;

	// Setup address
	EEAR = uiAddress;
	cli();
	// Start eeprom read by writing EERE
	//EECR = (1<<EERE);
	asm("ldi	r24, 0x01");
	asm("out	0x1f, r24");

	//inbyte = EEDR;
	asm("in	r24, 0x20");
	asm("std	Y+1, r24");
	//inbyte = 0x56;
	sei();
	// Return data from data register
	return inbyte;
}

// TODO - turn interrupts off during write start
void eeprom_write(uint16_t uiAddress, uint8_t ucData)
{
	//  Wait for completion of previous write
	while(EECR&(1<<EEPE)) ;

	// Set up address and Data registers
	EEAR = uiAddress;
	//EEARH = 0;
	//EEARL = 1;
	EEDR = ucData;
	cli();
	// turn interrupts off here

	// Start eeprom write by setting EEMPE
	//EECR = (1<<EEMPE);
	asm("ldi	r24, 0x04");
	asm("out	0x1f, r24");
	//asm("sbi EECR,EEMPE");

	// Start eeprom write by setting EEPE
	//EECR = (1<<EEPE);
	//asm("ldi	r24, 0x02");
	//asm("out	0x1f, r24");
	asm("sbi	0x1f, 0x01");

	sei();
}
