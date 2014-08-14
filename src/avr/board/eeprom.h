/*
 * eeprom.h
 *
 * Created: 29/06/2012 17:37:08
 * Author: Juan Navarro
 */ 

#ifndef EEPROM_H_
#define EEPROM_H_

#include <stdint.h>


uint8_t eeprom_read(uint16_t uiAddress);
void eeprom_write(uint16_t uiAddress, uint8_t ucData);

#endif // EEPROM_H_
