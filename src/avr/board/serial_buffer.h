/*
 * serial_buffer.h
 *
 * High efficiency implementation of a circular buffer (aka. "ring buffer").
 * "Full Buffer" condition is detected by always keeping one empty slot.
 *
 * Created: 28/06/2012 20:21:58
 * Author: Juan Navarro
 */

#ifndef SERIAL_BUFFER_H_
#define SERIAL_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

#define SB_BUFFER_SIZE 256  // (Bytes) must be power of 2: 2,4,8,16,32,64,128,256, etc.

// Circular buffer object.
typedef struct {
	uint16_t idx_write; // Write Index (Mod buffer size). Points to next empty slot.
	uint16_t idx_read; // Read Index (Mod buffer size). Points to the oldest used slot.
	uint8_t buffer[SB_BUFFER_SIZE]; // Data slots.
} SerialBuffer;


void sb_init(SerialBuffer* sb);
bool sb_is_empty(const SerialBuffer* sb);
bool sb_is_full(const SerialBuffer* sb);

// Returns the number of used slots in the buffer.
uint16_t sb_count_used(const SerialBuffer* sb);

// Returns the number of empty (free) slots in the buffer.
uint16_t sb_count_free(const SerialBuffer* sb);

// Write a new data element.
// Overwrite oldest element if buffer is full.
// sb_is_full() should be used to check if overwriting will happen.
void sb_write(SerialBuffer* sb, uint8_t data);

// Read oldest element.
// Return 0 when buffer is empty.
// sb_is_empty() should be used to ensure there is valid data in the buffer.
uint8_t sb_read(SerialBuffer* sb);

#endif // SERIAL_BUFFER_H_
