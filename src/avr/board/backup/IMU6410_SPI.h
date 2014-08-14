/*
 * IMU6410_SPI.h
 *
 * Created: 29/06/2012 1:46:07
 * Author: Juan Navarro
 */

#ifndef IMU6410_SPI_H_
#define IMU6410_SPI_H_

void Release_SPI(void);
void Init_SPI_Master(void);
void SPI_MasterTransmit(unsigned char cData);
unsigned char SPI_MasterReceive(void);

#endif // IMU6410_SPI_H_
