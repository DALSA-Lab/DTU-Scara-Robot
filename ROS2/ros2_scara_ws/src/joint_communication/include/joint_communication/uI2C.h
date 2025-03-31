#ifndef USERIAL_H
#define USERIAL_H
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#define ACK 'O'
#define NACK 'N'
#define RFLAGS_SIZE 1
#define MAX_BUFFER 4  // Bytes


/**
 * Initiates an I2C device on the bus
 * @param dev_addr 7-bit device adress [0 - 0x7F]
 * @return the device handle, negative on error.
 */
int openI2CDevHandle(const int dev_addr);

/**
 * reads block of bytes from device to buffer
 * @param dev_handle device handle obtained from `openI2CDevHandle`
 * @param reg the command/data register
 * @param buffer pointer to data buffer to hold received values
 * @param data_length number of bytes to read
 * @return number of bytes read, negative on error.
 */
int readFromI2CDev(const int dev_handle, const int reg, char *buffer, const int data_length);

/**
 * writes block of bytes from buffer to device
 * @param dev_handle device handle obtained from `openI2CDevHandle`
 * @param reg the command/data register
 * @param tx_buffer pointer to data buffer holding the data to send
 * @param data_length number of bytes to send
 * @param RFLAGS_buffer buffer to hold returned flags
 * @return 0 on OK, negative on error.
 */
int writeToI2CDev(const int dev_handle, const int reg, char *tx_buffer, const int data_length, char *RFLAGS_buffer);

/**
 * close an I2C device on the bus
 * @param dev_handle device handle obtained from `openI2CDevHandle`
 * @return 0 on OK, negative on error.
 */
int closeI2CDevHandle(const int dev_handle);

/**
 * Compute the two' complement checksum of the `buffer` according to SAE J1708
 * @param buffer Pointer to buffer to compute checksum off
 * @param length Length of the buffer
 * @return Two's complement checksum.
 */
u_int8_t generateChecksum(const u_int8_t *buffer, size_t length);

#endif