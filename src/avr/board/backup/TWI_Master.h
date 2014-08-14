/*
 *
 * SOC Robotics, Inc
 *
 * File - TWI_Master.h
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
 * Copyright 2008, SOC Robotics, Inc.  All rights reserved.
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

#ifndef TWI_MASTER_H_
#define TWI_MASTER_H_

/****************************************************************************
  TWI Status/Control definitions
****************************************************************************/

// Set this to the largest message size that will be sent/recieved including address byte.
#define TWI_BUFFER_SIZE 21

//this defines the internal clock frequency of the device
#define MCK 10000000U

//Set this to the MAX clock frequency you want the TWI to operate at.  Note that the target slave should have a clk freq of x 16 this value
#define SCL_FREQ 100000U

//This code signifies that no error occured
#define TWI_NO_ERROR 0
//This code signifies that there was an error in the last TWI transaction.
//The user should call TWI_Get_State_Info for the exact issue
#define TWI_TRANSACTION_ERROR 2


//uncomment this if you do not wish to use a timer to prevent you endlessly looping
//#define TWI_TIMER
#if defined(TWI_TIMER)
//This should be set to a timer that we can ensure that we do not get stuck waiting for the TWI to finish a transaction
#define TWI_ERROR_TIMER mseccountdown2
//This is how long we will be willing to wait for last transaction to complete
#define TWI_TIMEOUT 700
//This is an error code to state that a time out has occured.  If the user gets this error they should reset the TWI
#define TWI_TIMEOUT_ERROR 1
#endif

//Uncomment this if you want to print to the serial port if an error occurs
//#define DEBUG

/****************************************************************************
  Global definitions
****************************************************************************/
//This is set to a 1 to signify that the last transaction was successful
extern unsigned char TWI_LastTransOK;

/****************************************************************************
  Function definitions
****************************************************************************/
/*
 * Call this function to set up the TWI master to its initial standby state.
 * Remember to enable interrupts from the main application after initializing the TWI.
 */
void TWI_Master_Initialize(void);

/*
 * Call this function to disable the TWI interface
 */
void TWI_Master_Disable(void);

/*
 * Call this function to fetch the state information of the previous operation.
 * The function will hold execution (loop) until the TWI has completed with the
 * previous transaction or until the timer expires. If there was an error,
 * then the function will return the TWI State code. 
 *
 * Returns:	TWI_state
 *			TWI_TIMEOUT_ERROR if there was a timeout while waiting to get the state info
 */
unsigned char TWI_Get_State_Info( void );


/*
 * Call this function to read msgSize bytes from the slave with address, address
 *
 * Warning the function will hold execution (loop) until the TWI has completed
 * the previous transaction or the timer has elapsed.  If the timer has elapsed
 * and error code will be returned.
 *
 * Returns: TWI_TIMEOUT 			if there was a timeout while waiting
 *			TWI_NO_ERROR 			if everything was ok
 *			TWI_TRANSACTION_ERROR 	if there was a problem i.e. there was not device on the bus with that address
 *
 */
unsigned char  TWI_Start_Read( unsigned char address, unsigned char msgSize );

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
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize );

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

unsigned char TWI_Start_Write( unsigned char address, unsigned char *msg, unsigned char msgSize );
/*
 *
 * Call this function to determine if the last transaction on the TWI bus was successfull
 *
 * Returns: TWI_TIMEOUT 			if there was a timeout while waiting
 *			TWI_NO_ERROR 			if everything was ok
 *			TWI_TRANSACTION_ERROR 	if there was a problem i.e. there was not device on the bus with that address
 */
unsigned char TWI_LastTransOk( void );

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master staus codes                      
#define TWI_START                  0x08  // START has been transmitted  
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter staus codes                      
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been tramsmitted and NACK received 
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been tramsmitted and NACK received 

// TWI Master Receiver staus codes  
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been tramsmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK tramsmitted

// TWI Slave Transmitter staus codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received

// TWI Slave Receiver staus codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = “0”
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#endif // TWI_MASTER_H_
