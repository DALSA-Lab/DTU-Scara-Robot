#include "serial_test/uSerial.h"
#include <poll.h>

using namespace std;

// Function to open the serial port
int openSerialPort(const char *portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        cerr << "Error opening " << portname << ": "
             << strerror(errno) << endl;
        return -1;
    }
    return fd;
}

// Function to configure the serial port
bool configureSerialPort(int fd, int speed)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        cerr << "Error from tcgetattr: " << strerror(errno)
             << endl;
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // Clear all the size bits, then set 8 bits per byte (most common)
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing

    tty.c_oflag = 0;     // no remapping, no delays
    tty.c_cc[VMIN] = 1;  // waits for 1 bytes
    tty.c_cc[VTIME] = 1; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // No parity
    tty.c_cflag &= ~CSTOPB;            // 1 Stop Bit
    tty.c_cflag &= ~CRTSCTS;           // No flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        cerr << "Error from tcsetattr: " << strerror(errno)
             << endl;
        return false;
    }
    return true;
}

int readFromSerialPort(int fd, u_int8_t *buffer, size_t size, int timeout_ms)
{
    struct pollfd fds;
    fds.fd = fd;
    fds.events = POLLIN;

    size_t totalBytesRead = 0;
    while (totalBytesRead < size)
    {
        char byte;

        int pollResult = poll(&fds, 1, timeout_ms);
        if (pollResult < 0)
        {
            cerr << "Error from polling Serial: " << strerror(errno) << endl;
            return -1;
        }
        else if (pollResult == 0)
        {
            std::cout << "Timeout occurred, no data received in " << timeout_ms << " ms." << std::endl;
            return 0;
        }
        else
        {
            int bytesRead = read(fd, &byte, 1); // Read one byte at a time
            if (bytesRead < 0)
            {
                perror("read");
                return -1; // Error occurred
            }
            else if (bytesRead == 0)
            {
                // No data read, possibly due to timeout
                break;
            }

            // if (byte == '\n')
            // {
            //     break; // Stop reading if newline is received
            // }

            // Store the byte in the buffer
            buffer[totalBytesRead++] = byte;

            // Check for newline character
        }
    }

    // Null-terminate the buffer
    // if (totalBytesRead < size) {
    //     buffer[totalBytesRead] = '\0'; // Null-terminate if there's space
    // } else {
    //     buffer[size - 1] = '\0'; // Ensure null-termination if buffer is full
    // }

    return totalBytesRead; // Return the number of bytes read
}

// Function to write data to the serial port
int writeToSerialPort(int fd, const u_int8_t *buffer, size_t size)
{
    return write(fd, buffer, size);
}

// Function to close the serial port
void closeSerialPort(int fd) { close(fd); }


u_int8_t generateChecksum(const u_int8_t *buffer, size_t length)
{
    u_int32_t sum = 0; // Use a larger type to avoid overflow

    // Sum all bytes in the buffer
    for (size_t i = 0; i < length; ++i)
    {
        sum += buffer[i];
    }

    // Calculate the two's complement
    return static_cast<u_int8_t>(~sum + 1);
}
