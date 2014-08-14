/*
 *
 * SOC Robotics, Inc
 *
 * File - TWI_Master.c
 *
 * Version: 0.92
 * Author:  SS	  					  Date:  June 11, 2010
 * Compiled with AVR Studio 4.19.
 *
 *
 * Descrition:
 *	This TWI module implements an interupt driven TWI Master.
 *
 * 	This Module is based on Atmel's application note AVR315
 *
 * Revision: 1.0 -> Date: October 23rd, 2008
 *		Original										BR
 *
 * History:
 *
 * Copyright 2008-2011, SOC Robotics, Inc.  All rights reserved.
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
 * 4. Redistribution is only allowed in combination with SOC ROBOTICS,
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
 * For additional information see http://www.soc-machines.com/
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWI_Master.h"


#if defined (TWI_TIMER)
#include "Timer.h"
#endif

#if defined(DEBUG1)
#include "Serial.h"
#include "Utilities.h"
unsigned char text[20];
#endif

//Transceiver buffer
static unsigned char TWI_Buffer[ TWI_BUFFER_SIZE ];

// Number of bytes to be transmitted.
static unsigned char TWI_Message_Size;

// State byte. Default set to TWI_NO_STATE.
static unsigned char TWI_state = TWI_NO_STATE;

//Set to one when the last tansaction was ok
unsigned char TWI_LastTransOK = 0;

/*
 * Call this function to test if the TWI_ISR is busy transmitting/receiving.
 * If the TWI Interrupt is enabled then the Transceiver is busy
 * If the TWSTO flag is set then the Transceiver is busy sending a stop condition
 */
#define TWI_Transceiver_Busy() ( TWCR & ( (1<<TWIE) | (1<<TWSTO) ) )


/*
 * Call this function to set up the TWI master to its initial standby state.
 * Remember to enable interrupts from the main application after initializing the TWI.
 */
void TWI_Master_Initialize(void)
{

#if ((MCK/(2*SCL_FREQ))-8 < 255 )
	TWBR =	(MCK/(2*SCL_FREQ))-8; // Set bit rate register. This sets the fastest SCL frequency. See header file.
	//prescaler = 1
#elif ((MCK/(8*SCL_FREQ))-2 < 255 )
	TWBR =	(MCK/(8*SCL_FREQ))-2; // Set bit rate register). This sets the fastest SCL frequency. See header file.
	TWSR = (1<<TWPS0);//prescaler = 4
#elif ((MCK/(32*SCL_FREQ)) < 255 )
    TWBR =	(MCK/(32*SCL_FREQ)); // Set bit rate register). This sets the fastest SCL frequency. See header file.
	TWSR = (1<<TWPS1);//prescaler = 16
#elif ((MCK/(128*SCL_FREQ)) < 255 )
    TWBR =	(MCK/(128*SCL_FREQ)); // Set bit rate register). This sets the fastest SCL frequency. See header file.
	TWSR = (1<<TWPS1) | (1<<TWPS0);//prescaler = 64
#endif

	TWCR =	(1<<TWEN)|                        	// Enable TWI-interface
			(0<<TWIE)|(0<<TWINT)|              	// Interupt disabled
			(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|	// Reset the TWI module and release the SDA SCL lines.  This is in case we are already in the middle for a transaction
			(0<<TWWC);

	TWDR = 	0xFF; // Default content = SDA released.

	TWCR = 	(1<<TWEN)|                        // Enable TWI-interface and release TWI pins.
         	(0<<TWIE)|(0<<TWINT)|             // Disable Interupt.
         	(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|  // No Signal requests.
         	(0<<TWWC);

	TWI_LastTransOK = 0;
}

/*
 * Call this function to disable the TWI interface
 */
void TWI_Master_Disable(void)
{
	TWCR =	(1<<TWEN)|                         	// Enable TWI-interface
			(0<<TWIE)|(1<<TWINT)|            	// Interupt disabled
			(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|	// Reset the TWI module and release the SDA SCL lines.  This is in case we are already in the middle for a transaction
			(0<<TWWC);
	TWCR =	(0<<TWEN);//Turn off the TWI Interface
}

/*
 * Call this function to fetch the state information of the previous operation.
 * The function will hold execution (loop) until the TWI has completed with the
 * previous transaction or until the timer expires. If there was an error,
 * then the function will return the TWI State code.
 *
 * Returns:	TWI_state
 *			TWI_TIMEOUT_ERROR if there was a timeout while waiting to get the state info
 */
unsigned char TWI_Get_State_Info( void )
{
#if defined(TWI_TIMER)
    TWI_ERROR_TIMER = TWI_TIMEOUT;//Load the timer with the max time we wish to wait for

	while ( TWI_Transceiver_Busy() && (TWI_ERROR_TIMER != 0)); // Wait until TWI is ready for next transmission.
	if (!TWI_ERROR_TIMER)//If we stopped waiting due to the timer counting down, return an error code
    {
    	return TWI_TIMEOUT_ERROR;
    }
#else
	while ( TWI_Transceiver_Busy() ); // Wait until TWI is ready for next transmission.
#endif

	return ( TWI_state );// Return error state.
}


/*
 * Call this function to read msgSize bytes from the slave with address, address
 *
 * Warning the function will hold execution (loop) until the TWI has completed
 * the previous transaction or the timer has elapsed.  If the timer has elapsed
 * and error code will be returned.
 *
 * Returns: TWI_TIMEOUT 			if there was a timeout while waiting
 *			TWI_NO_ERROR 			if everything was ok
 *			TWI_TRANSACTION_ERROR 	if there was a problem i.e. there was no device on the bus with that address
 *
 */
unsigned char  TWI_Start_Read( unsigned char address, unsigned char msgSize )
{
	unsigned char temp;
#if defined(TWI_TIMER)
    TWI_ERROR_TIMER = TWI_TIMEOUT;//Load the timer with the max time we wish to wait for

	while ( TWI_Transceiver_Busy() && (TWI_ERROR_TIMER != 0)); // Wait until TWI is ready for next transmission.


    if (!TWI_ERROR_TIMER)//If we stopped waiting due to the timer counting down, return an error code
    {
    	return TWI_TIMEOUT_ERROR;
    }
#else
	while ( TWI_Transceiver_Busy() ); // Wait until TWI is ready for next transmission.
#endif

	TWI_Buffer[0]  = (address<<1)|1; // Store the slaves address with Read setting
	TWI_Message_Size = msgSize+1; // Number of data bytes to transmit + 1 (address)

	temp = TWI_LastTransOK;

	TWI_LastTransOK = 0;

	TWCR =	(1<<TWEN)|                       	// TWI Interface enabled.
         	(1<<TWIE)|(1<<TWINT)|              	// Enable TWI Interupt and clear the flag.
         	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|	// Initiate a START condition.
         	(0<<TWWC);

    if( temp ) //return if the last trans was successful
	{
        return TWI_NO_ERROR;
	}
    else
    {
    	return TWI_TRANSACTION_ERROR;
    }
}

/*
 * Call this function to read out the requested data from the TWI transceiver buffer.
 * i.e. first call TWI_Start_Read to send a request for data to the slave.
 * Then call this function to collect the data when it arrives.
 *
 * Include a pointer to where to place the data and the number of bytes you wish to read
 *
 * Warning the function will hold execution (loop) until the TWI has completed
 * the previous transaction or the timer has elapsed.  If the timer has elapsed
 * and error code will be returned. If there was an error in the previous
 * transmission the function will return an error code.
 *
 * Returns: TWI_TIMEOUT 			if there was a timeout while waiting
 *			TWI_NO_ERROR 			if everything was ok
 *			TWI_TRANSACTION_ERROR 	if there was a problem i.e. there was not device on the bus with that address

 *
 */
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize )
{
	unsigned char i;

#if defined(TWI_TIMER)
    TWI_ERROR_TIMER = TWI_TIMEOUT;//Load the timer

	while ( TWI_Transceiver_Busy() && (TWI_ERROR_TIMER != 0)); // Wait until TWI is ready for next transmission.


    if (!TWI_ERROR_TIMER)
    {
    	return TWI_TIMEOUT_ERROR;
    }
#else
	while ( TWI_Transceiver_Busy() ); // Wait until TWI is ready for next transmission.
#endif

	if( TWI_LastTransOK ) // Copy data if last transmission competed successfully.
	{
		for ( i=1; i<=msgSize; i++ ) // Copy data from Transceiver buffer. Ignores the TWI Address
    	{
     		msg[ i-1 ] = TWI_Buffer[ i ];
    	}
        return TWI_NO_ERROR;
	}
    else
    {
    	return TWI_TRANSACTION_ERROR;
    }
}

/*
 * Call this function to write out data to the TWI slave whos address is address..
 *
 * Include a pointer to where to get the data and the number of bytes you wish to write
 *
 * Warning the function will hold execution (loop) until the TWI has completed
 * the previous transaction or the timer has elapsed.  If the timer has elapsed
 * and error code will be returned. Otherwise the TWI will be started.
 *
 * Returns: TWI_TIMEOUT 			if there was a timeout while waiting
 *			TWI_NO_ERROR 			if everything was ok
 *			TWI_TRANSACTION_ERROR 	if there was a problem i.e. there was not device on the bus with that address
 *
 */

unsigned char TWI_Start_Write( unsigned char address, unsigned char *msg, unsigned char msgSize )
{
	unsigned char i;

#if defined(TWI_TIMER)
    TWI_ERROR_TIMER = TWI_TIMEOUT;//load the timer

	while ( TWI_Transceiver_Busy() && (TWI_ERROR_TIMER != 0)); // Wait until TWI is ready for next transmission.


    if (!TWI_ERROR_TIMER)
    {
    	return TWI_TIMEOUT_ERROR;
    }
#else
	while ( TWI_Transceiver_Busy()); // Wait until TWI is ready for next transmission.
#endif

	TWI_Message_Size = msgSize + 1; // Number of data bytes to transmit.
	TWI_Buffer[0]  = (address<<1); // Store slave address with W setting.

	//copy the data over to my buffer.
	for (i = 1; i <= msgSize; i++ )
	{
		TWI_Buffer[ i ] = msg[ i - 1 ];
	}

  	i = TWI_LastTransOK ? TWI_NO_ERROR : TWI_TRANSACTION_ERROR;

	TWI_LastTransOK = 0;
	TWI_state = TWI_NO_STATE ;
	TWCR = 	(1<<TWEN)|                        // TWI Interface enabled.
         	(1<<TWIE)|(1<<TWINT)|             // Enable TWI Interupt and clear the flag.
         	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|  // Initiate a START condition.
         	(0<<TWWC);

   	return i;

}


/*
 *
 * Call this function to determine if the last transaction on the TWI bus was successfull
 *
 * Returns: TWI_TIMEOUT 			if there was a timeout while waiting
 *			TWI_NO_ERROR 			if everything was ok
 *			TWI_TRANSACTION_ERROR 	if there was a problem i.e. there was not device on the bus with that address
 */
unsigned char TWI_LastTransOk( void )
{
#if defined(TWI_TIMER)
    TWI_ERROR_TIMER = TWI_TIMEOUT;//load the timer

	while ( TWI_Transceiver_Busy() && (TWI_ERROR_TIMER != 0)); // Wait until TWI is ready for next transmission.


    if (!TWI_ERROR_TIMER)
    {
    	return TWI_TIMEOUT_ERROR;
    }
#else
	while ( TWI_Transceiver_Busy() ); // Wait until TWI is ready for next transmission.
#endif

    return TWI_LastTransOK ? TWI_NO_ERROR : TWI_TRANSACTION_ERROR;
}

/*								********** Interrupt Handler **********
 *
 * This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
 * that is whenever a TWI event has occurred. This function should not be called directly from the main
 * application.
 */
//#pragma interrupt_handler TWI_Interrupt:iv_TWI
//void TWI_Interrupt(void)
ISR(TWI_vect)
{
	static unsigned char TWI_Buffer_Ptr;//Points to the next byte in the buffer

    TWI_state = TWSR; // Store TWSR.

	switch (TWSR)
	{
    	case TWI_START: 		// START has been transmitted
    	case TWI_REP_START: 	// Repeated START has been transmitted
      		TWI_Buffer_Ptr = 0;	// Set buffer pointer to the TWI Address location
    	case TWI_MTX_ADR_ACK: 	// SLA+W has been tramsmitted and ACK received
    	case TWI_MTX_DATA_ACK: 	// Data byte has been tramsmitted and ACK received
			if (TWI_Buffer_Ptr < TWI_Message_Size)
			{
				TWDR = TWI_Buffer[TWI_Buffer_Ptr++];	//Select the next byte to be transmitted
				TWCR =	(1<<TWEN)|                      // TWI Interface enabled
						(1<<TWIE)|(1<<TWINT)|           // Enable TWI Interupt and clear the interupt flag
						(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
						(0<<TWWC);
			}
			else // Send STOP after last byte
			{
				TWI_LastTransOK = 1; // Transaction completed successfully.
        		TWCR =	(1<<TWEN)|                        // TWI Interface enabled
              		 	(0<<TWIE)|(1<<TWINT)|             // Disable TWI Interrupt and clear the interupt flag
               			(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|  // Initiate a STOP condition.
               			(0<<TWWC);
			}

			break;

	case TWI_MRX_DATA_ACK: // Data byte has been received and ACK transmitted
		TWI_Buffer[TWI_Buffer_Ptr++] = TWDR; //Read in the data

    case TWI_MRX_ADR_ACK: // SLA+R has been tramsmitted and ACK received

		if (TWI_Buffer_Ptr < (TWI_Message_Size-1) ) // Detect the last byte so you can NACK it.
      	{
        	TWCR = 	(1<<TWEN)|                        // TWI Interface enabled
               		(1<<TWIE)|(1<<TWINT)|             // Enable TWI Interupt and clear the interupt flag to read next byte
               		(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|  // Send ACK after reception
               		(0<<TWWC);
      	}
        else // Send NACK after next reception
      	{
        	TWCR = 	(1<<TWEN)|                        // TWI Interface enabled
               		(1<<TWIE)|(1<<TWINT)|             // Enable TWI Interupt and clear the interupt flag to read next byte
               		(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|  // Send NACK after reception
               		(0<<TWWC);
      	}

      	break;

    case TWI_MRX_DATA_NACK:// Data byte has been received and NACK tramsmitted

      	TWI_Buffer[TWI_Buffer_Ptr] = TWDR;//Read in the data
      	TWI_LastTransOK = 1;// transaction completed successfully.

      	TWCR = 	(1<<TWEN)|                       // TWI Interface enabled
             	(0<<TWIE)|(1<<TWINT)|            // Disable TWI Interrupt and clear the interupt flag
             	(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)| // Initiate a STOP condition.
             	(0<<TWWC);

      	break;

    case TWI_ARB_LOST: // Arbitration lost

        TWCR = 	(1<<TWEN)|                        // TWI Interface enabled
             	(1<<TWIE)|(1<<TWINT)|             // Enable TWI Interupt and clear the intertupt flag
             	(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|  // Initiate a (RE)START condition.
             	(0<<TWWC);

      	break;

    case TWI_MTX_ADR_NACK: // SLA+W has been tramsmitted and NACK received
    case TWI_MRX_ADR_NACK: // SLA+R has been tramsmitted and NACK received
    case TWI_MTX_DATA_NACK: // Data byte has been tramsmitted and NACK received
//  case TWI_NO_STATE       // No relevant state information available; TWINT = “0”
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:

#if defined(DEBUG1)
	  	uart0_puts("\nTWI ERROR - ");
	  	uart0_puts(Print_Int(TWI_state,text,HEX));
        uart0_puts("\n");
#endif

	    // Reset TWI Interface
      	TWCR = (1<<TWEN)|                       // Enable TWI-interface and release TWI pins
             	(0<<TWIE)|(1<<TWINT)|            // Disable Interupt and clear the intertupt flag (to release the TWI lines)
             	(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)| // No Signal requests
             	(0<<TWWC);
	}
}
