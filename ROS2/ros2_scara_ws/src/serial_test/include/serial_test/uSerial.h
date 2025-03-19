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

// Function to open the serial port
int openSerialPort(const char* portname);

// Function to configure the serial port
bool configureSerialPort(int fd, int speed);

/**
 * Function to read from the serial port.
 * @param fd file descripter of open serial port.
 * @param buffer pointer to buffer to store received bytes
 * @param size size of the buffer to store received bytes
 * @param timeout_ms timeout in ms for the first byte to arrive, timeout between bytes is set in `tty.c_cc[VTIME]` of the configuration function.
 * @return the total number of bytes read.
 */
int readFromSerialPort(int fd, u_int8_t* buffer, size_t size, int timeout_ms);

// Function to write data to the serial port
int writeToSerialPort(int fd, const u_int8_t* buffer, size_t size);

// Function to close the serial port
void closeSerialPort(int fd);

/**
 * Compute the two' complement checksum of the `buffer` according to SAE J1708
 * @param buffer Pointer to buffer to compute checksum off
 * @param length Length of the buffer
 * @return Two's complement checksum.
 */
u_int8_t generateChecksum(const u_int8_t *buffer, size_t length);

#endif