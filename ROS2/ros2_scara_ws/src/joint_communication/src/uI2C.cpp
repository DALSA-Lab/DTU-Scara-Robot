#include "joint_communication/uI2C.h"
#include "joint_communication/common.h"

#include <lgpio.h>


int openI2CDevHandle(const int dev_addr)
{
    int rc = lgI2cOpen(1, dev_addr, 0);
    if (rc < 0) {
        std::cerr << "I2C OPEN ERROR: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

int readFromI2CDev(const int dev_handle, const int reg, char *buffer, const int data_length)
{
    // char cmnd[] = {5, 1, reg, 4, 4,0};
    // int rc = lgI2cZip(dev_handle,cmnd,8,txBuf,5);
    int rc = lgI2cReadI2CBlockData(dev_handle, reg, buffer, data_length);
    if (rc < 0) {
        std::cerr << "I2C READ ERROR: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

int writeToI2CDev(const int dev_handle, const int reg, char *buffer, const int data_length)
{
    int rc = lgI2cWriteI2CBlockData(dev_handle, reg, buffer, data_length);
    if (rc < 0) {
        std::cerr << "I2C WRITE ERROR: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

int closeI2CDevHandle(const int dev_handle)
{
    int rc = lgI2cClose(dev_handle);
    if (rc < 0) {
        std::cerr << "I2C CLOSE ERROR: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

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
