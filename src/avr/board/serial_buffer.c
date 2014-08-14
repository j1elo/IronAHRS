/*
 * serial_buffer.c
 *
 * Created: 28/06/2012 20:21:47
 *  Author: Juan
 */

#include "serial_buffer.h"

// Used for Power-of-2 modulus arithmetic.
#define BUFFER_MASK (SB_BUFFER_SIZE - 1)

// Fast increment of an index value, modulus buffer size.
uint16_t increment_idx_mod(uint16_t* idx);


void sb_init(SerialBuffer* sb)
{
	sb->idx_write = 0;
	sb->idx_read = 0;
}

bool sb_is_empty(const SerialBuffer* sb)
{
	return sb->idx_write == sb->idx_read;
}

bool sb_is_full(const SerialBuffer* sb)
{
	return ((sb->idx_write + 1) & BUFFER_MASK) == sb->idx_read;
}

uint16_t sb_count_used(const SerialBuffer* sb)
{
	// Circular buffer arithmetic:
	// if write >= read
	//   used = write - read
	//   free = size - (write - read) - 1
	// else
	//   free = read - write - 1
	//   used = size - (read - write)

	return (sb->idx_write >= sb->idx_read) ?
		(sb->idx_write - sb->idx_read)
		: (SB_BUFFER_SIZE + sb->idx_write - sb->idx_read);
}

uint16_t sb_count_free(const SerialBuffer* sb)
{
	// "Full Buffer" condition is detected by always keeping one empty slot.
	// This is a simple, robust, approach that only requires two pointers, at the expense of one buffer slot.

	return (sb->idx_write >= sb->idx_read) ?
		(SB_BUFFER_SIZE + sb->idx_read - sb->idx_write - 1)
		: (sb->idx_read - sb->idx_write - 1);
}

void sb_write(SerialBuffer* sb, uint8_t data)
{
	if (sb_is_full(sb)) {
		// Full buffer: "push forward" the Read Index.
		increment_idx_mod(&sb->idx_read);
	}

	sb->buffer[sb->idx_write] = data;
	increment_idx_mod(&sb->idx_write);
}

uint8_t sb_read(SerialBuffer* sb)
{
	if (sb_is_empty(sb)) {
		// Empty buffer: return 0 and don't update Read Index.
		return 0;
	}
	else {
		uint8_t data = sb->buffer[sb->idx_read];
		increment_idx_mod(&sb->idx_read);
		return data;
	}
}

uint16_t increment_idx_mod(uint16_t* idx)
{
	return (*idx = (*idx + 1) & BUFFER_MASK);
}
