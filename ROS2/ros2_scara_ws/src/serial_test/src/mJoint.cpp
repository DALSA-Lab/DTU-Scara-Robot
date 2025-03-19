#include "serial_test/uSerial.h"
#include "serial_test/mJoint.h"


// int write(u_int8_t address, u_int8_t register, const u_int8_t *data)
// {
//     // send adr + register
//     // await ACK
//     // send data
//     // await ACK
// }

// int read(u_int8_t address, u_int8_t register, const u_int8_t *data)
// {
//     // send adr + register
//     // await ACK
//     // await data
//     // send NACK
// }
Joint::Joint(u_int8_t address, std::string name)
{
    this->address = address;
    this->name = name;
}

int Joint::init(int fd)
{
    std::cout << "initializing " << this->name << std::endl;
    this->fd = fd;
    return checkCom();
}

int Joint::printInfo(void)
{
    std::cout << "Name: " << this->name << " address: " << this->address << " fd: " << this->fd << std::endl;
    return 0;
}

int Joint::getAngle(float &angle)
{
    u_int8_t buf[4];
    int32_t int_angle;
    if (read(ANGLEMOVED, buf, 4) < 4)
    {
        return -1;
    }

    memcpy(&int_angle, buf, 4);
    angle = 1.0 * int_angle / 100;

    return 0;
}

int Joint::setAngle(float angle)
{
    u_int8_t buf[4];
    int32_t int_angle = angle*100;
    memcpy(buf, &int_angle, 4);
    return write(MOVETOANGLE, buf, 4);
}

int Joint::moveSteps(int32_t steps)
{
    u_int8_t buf[4];
    memcpy(buf, &steps, 4);
    return write(MOVESTEPS, buf, 4);
}

int Joint::checkCom(void)
{
    u_int8_t buf;
    read(PING,&buf,1);
    if(buf == 'O'){
        return 0;
    }
    return -1;
}


int Joint::read(const stp_reg_t reg, u_int8_t *data, const size_t data_length)
{
    // std::cout << "DEBUG: Sending header" << std::endl;
    u_int8_t header_buf[3] = {this->address, reg, 0};
    tcflush(this->fd, TCIOFLUSH);
    if (writeToSerialPort(this->fd, header_buf, 3) < 0)
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
    }

    u_int8_t ack_buf;
    int n = readFromSerialPort(this->fd, &ack_buf, 1, 100);
    if (n <= 0)
    {
        std::cerr << "ERROR: No ACK received" << std::endl;
        return -1;
    }
    // std::cout << "DEBUG: Received header" << std::endl;

    // std::cout << "Read from serial port: " << std::string((char *)&ack_buf, n) << std::endl;
    // std::cout << "Read from serial port: " << ack_buf << std::endl;

    if(ack_buf != ACK){
        std::cerr << "ERROR: NACK received" << std::endl;
        return -2; // NACK
    }

    // Add one byte for checksum
    u_int8_t *rx_buf = new u_int8_t[data_length + 1];
    n = readFromSerialPort(this->fd, rx_buf, data_length + 1, 100);
    if (n <= 0)
    {
        std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
        return -1;
    }
    // std::cout << "Read from serial port: " << std::string((char *)rx_buf, n) << std::endl;

    // TODO: check checksum HERE


    memcpy(data, rx_buf, data_length);

    delete[] rx_buf;

    return n;
}

int Joint::write(const stp_reg_t reg, u_int8_t *data, const size_t data_length)
{
    u_int8_t header_buf[3] = {this->address, reg, (u_int8_t)data_length};
    tcflush(this->fd, TCIOFLUSH);
    if (writeToSerialPort(this->fd, header_buf, 3) < 0)
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        return -1; // R/W-error
    }

    u_int8_t ack_buf;
    int n = readFromSerialPort(this->fd, &ack_buf, 1, 100);
    if (n <= 0)
    {
        std::cerr << "ERROR: No ACK received" << std::endl;
        return -1; // R/W-error
    }
    // std::cout << "DEBUG: Received header" << std::endl;

    // std::cout << "Read from serial port: " << std::string((char *)&ack_buf, n) << std::endl;

    if(ack_buf != ACK){
        std::cerr << "ERROR: NACK received" << std::endl;
        return -2; // NACK
    }
    
    // Add one byte for checksum
    u_int8_t *tx_buf = new u_int8_t[data_length + 1];
    memcpy(tx_buf, data, data_length);

    // TODO: Implement Checksum here
    // u_int8_t checksum = '!';
    u_int8_t checksum = generateChecksum(tx_buf,data_length);

    tx_buf[data_length] = checksum;
    
    if (writeToSerialPort(this->fd, tx_buf, data_length + 1) < 0)
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
        return -1; // R/W-error
    }
    // std::cout << "Read from serial port: " << std::string((char *)tx_buf, n) << std::endl;

    n = readFromSerialPort(this->fd, &ack_buf, 1, 100);
    if (n <= 0)
    {
        std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
        return -1; // R/W-error
    }

    if(ack_buf != ACK){
        return -2; // NACK
    }

    delete[] tx_buf;

    return 0;
}
