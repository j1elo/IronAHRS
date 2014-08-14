/*
 * avr_io.h
 *
 * Created: 27/06/2012 21:35:14
 * Author: Juan Navarro
 */

#ifndef AVR_IO_H_
#define AVR_IO_H_

//#ifdef TARGET
//#define IOMEM_BASE 0x2FF
//#else
//int32_t fake_registers[IO_MEM_RANGE];
//uint32_t IOMEM_BASE = (int32_t)&fake_registers;
//#endif

#include <avr/io.h>

// Own implementation of _BV().
#define BIT(b) (1 << (b))

// Bit twiddling.
#define BIT_SET(port, bit) ((port) |= BIT(bit))
#define BIT_CLEAR(port, bit) ((port) &= ~(BIT(bit)))
#define BIT_TOGGLE(port, bit) ((port) ^= BIT(bit))

#endif // AVR_IO_H_
