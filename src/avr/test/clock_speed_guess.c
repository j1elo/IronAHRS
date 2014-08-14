/*
 * clock_speed_test.c
 *
 * Created: 15/10/2012 1:16:40
 * Author: Juan Navarro
 */

#include "tests.h"
#include "board/serial.h"
#include "board/timer_delay_x.h"
#include <stdint.h>
#include <stdio.h>  // scanf()

//#include "board/timer_delay_x.h"
//#include "board/board.h"
//#include "board/avr_io.h"
//#include <stdbool.h>


void clock_speed_guess(void)
{
    while (1)
    {
        //printf("serial_available()\r\n");
        // Block until data arrives.
        while (!serial_available());

        // long is 32 bits.
        unsigned long int guess;
        //printf("scanf()\r\n");
        if (scanf("%lu", &guess) == 1) {
            //printf("_delay_cycles()\r\n");
            _delay_cycles(guess);
            printf("CHECK\r\n");
        }
        else {
            printf("ERROR\r\n");
        }
    }
}
