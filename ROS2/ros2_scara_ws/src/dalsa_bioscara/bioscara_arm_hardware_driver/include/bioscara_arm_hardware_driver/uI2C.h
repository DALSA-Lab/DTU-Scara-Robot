/**
 * @file uI2C.h
 * @author Sebastian Storz
 * @brief Low level utility for I2C communication on Raspberry Pi using lgpio library
 * @version 0.1
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * lgpio needs to be installed and linked!
 * Installation:
 * ```
 * 
 * cd ~
 * sudo apt update
 * sudo apt install -y swig
 * wget https://github.com/joan2937/lg/archive/master.zip
 * unzip master.zip
 * cd lg-master
 * make
 * sudo make install
 * cd ..
 * sudo rm -rf lg-master
 * rm master.zip
 * ```bash
 *
 */
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

/**
 * @copydoc RFLAGS_SIZE
 */
#define RFLAGS_SIZE 1

/**
 * @copydoc MAX_BUFFER
 */
#define MAX_BUFFER 4 // Bytes

/**
 * @brief Initiates an I2C device on the bus
 * @param dev_addr 7-bit device adress [0 - 0x7F]
 * @return the device handle, negative on error.
 */
int openI2CDevHandle(const int dev_addr);

/**
 * @brief reads block of bytes from device to buffer
 * @param dev_handle device handle obtained from `openI2CDevHandle`
 * @param reg the command/data register
 * @param buffer pointer to data buffer to hold received values
 * @param data_length number of bytes to read
 * @return number of bytes read, negative on error.
 */
int readFromI2CDev(const int dev_handle, const int reg, char *buffer, const int data_length);

/**
 * @brief writes block of bytes from buffer to device
 * @param dev_handle device handle obtained from `openI2CDevHandle`
 * @param reg the command/data register
 * @param tx_buffer pointer to data buffer holding the data to send
 * @param data_length number of bytes to send
 * @param RFLAGS_buffer buffer to hold returned flags
 * @return 0 on OK, negative on error.
 */
int writeToI2CDev(const int dev_handle, const int reg, char *tx_buffer, const int data_length, char *RFLAGS_buffer);

/**
 * @brief close an I2C device on the bus
 * @param dev_handle device handle obtained from `openI2CDevHandle`
 * @return 0 on OK, negative on error.
 */
int closeI2CDevHandle(int &dev_handle);


#endif