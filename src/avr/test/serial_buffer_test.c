/*
 * serial_buffer_test.c
 *
 * Created: 18/09/2012 0:23:22
 * Author: Juan Navarro
 */

#include "tests.h"
#include "board/serial_buffer.h"
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>


void serial_buffer_test(void)
{
	SerialBuffer sb;
	const int CAPACITY = SB_BUFFER_SIZE - 1;
    int i;

    // Check empty buffer.
	sb_init(&sb);
	assert(sb_is_empty(&sb));
	assert(!sb_is_full(&sb));
	assert(0 == sb_count_used(&sb));
	assert(CAPACITY == sb_count_free(&sb));

    // Put 1 item.
	sb_write(&sb, 'A');
	assert(!sb_is_empty(&sb));
	assert(!sb_is_full(&sb));
	assert(1 == sb_count_used(&sb));
	assert(CAPACITY - 1 == sb_count_free(&sb));

    // Get 1 item. Empty again.
	assert('A' == sb_read(&sb));
	assert(sb_is_empty(&sb));
	assert(!sb_is_full(&sb));
	assert(0 == sb_count_used(&sb));
	assert(CAPACITY == sb_count_free(&sb));

    // Completely fill the buffer.
	for (i = 0; i < CAPACITY; i++) {
		sb_write(&sb, i % 256);
    }
	assert(!sb_is_empty(&sb));
	assert(sb_is_full(&sb));
	assert(CAPACITY == sb_count_used(&sb));
	assert(0 == sb_count_free(&sb));

    // Put 1 item in an already full buffer.
	sb_write(&sb, 'C');
	assert(!sb_is_empty(&sb));
	assert(sb_is_full(&sb));
	assert(CAPACITY == sb_count_used(&sb));
	assert(0 == sb_count_free(&sb));

    // Get all but last items. Check oldest item was lost.
	for (i = 0; i < CAPACITY - 1; i++) {
		assert((i + 1) % 256 == sb_read(&sb));
    }
	assert(!sb_is_empty(&sb));
	assert(!sb_is_full(&sb));
	assert(1 == sb_count_used(&sb));
	assert(CAPACITY - 1 == sb_count_free(&sb));

    // Get last item. Check it's the same as what was inserted.
	assert('C' == sb_read(&sb));
	assert(sb_is_empty(&sb));
	assert(!sb_is_full(&sb));
	assert(0 == sb_count_used(&sb));
	assert(CAPACITY == sb_count_free(&sb));
}
