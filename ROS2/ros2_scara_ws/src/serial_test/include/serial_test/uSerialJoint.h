#ifndef SERIAL_NODE_H
#define SERIAL_NODE_H
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// Function to open the serial port
int openSerialPort(const char* portname);

// Function to configure the serial port
bool configureSerialPort(int fd, int speed);

// Function to read data from the serial port
int readFromSerialPort(int fd, char* buffer, size_t size);

// Function to write data to the serial port
int writeToSerialPort(int fd, const char* buffer,
                      size_t size);

// Function to close the serial port
void closeSerialPort(int fd);

#endif