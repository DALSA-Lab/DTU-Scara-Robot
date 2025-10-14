#include "bioscara_hardware_driver/uI2C.h"
#include "bioscara_hardware_driver/common.h"

#include <lgpio.h>

// #include <chrono> // remove when done timing


int openI2CDevHandle(const int dev_addr)
{
    int rc = lgI2cOpen(1, dev_addr, 0);
    if (rc < 0)
    {
        std::cerr << "I2C OPEN ERROR: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

int readFromI2CDev(const int dev_handle, const int reg, char *buffer, const int data_length)
{
    int rc;
    for (size_t i = 0; i < 3; i++)
    {
        rc = lgI2cReadI2CBlockData(dev_handle, reg, buffer, data_length);
        if(rc < 0 && i+1 < 3){
            usleep(50);
        }else{
            break;
        }
    }

    if (rc < 0)
    {
        std::cerr << "[ERROR] I2C read error: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

int writeToI2CDev(const int dev_handle, const int reg, char *tx_buffer, const int data_length, char *RFLAGS_buffer)
{

    char cmnd[MAX_BUFFER + 6];
    cmnd[0] = 5;                                  // CMD: Write
    cmnd[1] = 1 + static_cast<char>(data_length); // N Bytes: 1 (reg) + data_length
    cmnd[2] = reg;                                // Data: register
    memcpy(&cmnd[3], tx_buffer, data_length);
    cmnd[3 + data_length] = 4;           // CMD: Read
    cmnd[4 + data_length] = RFLAGS_SIZE; // N Bytes: RFLAGS_SIZE
    cmnd[5 + data_length] = 0;           // Terminate Buffer

    int rc;
    for (size_t i = 0; i < 3; i++)
    {
        // auto start = std::chrono::high_resolution_clock::now();
        /* There is a bug in the lgpio library that requires `rxCount` to be set n+1 higher*/
        rc = lgI2cZip(dev_handle, cmnd, 6 + data_length, RFLAGS_buffer, RFLAGS_SIZE + 1);
        // rc = lgI2cWriteBlockData(dev_handle, reg, tx_buffer, data_length);
        // rc |= lgI2cReadDevice(dev_handle, RFLAGS_buffer, RFLAGS_SIZE);
        // auto now = std::chrono::high_resolution_clock::now();
        // const std::chrono::duration<double, std::micro> t = now - start;
        // printf("lgI2cZip, tx size: %d, rx size: %d, took: %.3f\n",data_length,RFLAGS_SIZE,t.count());

        if(rc < 0 && i+1 < 3){
            std::cerr << "[WARN] I2C write unsuccessfull, retrying. Error: \'" << lguErrorText(rc) << "\'" << std::endl;
            usleep(50);
        }else{
            break;
        }
    }

    if (rc < 0)
    {
        std::cerr << "[ERROR] I2C write error: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

int closeI2CDevHandle(const int dev_handle)
{
    int rc = lgI2cClose(dev_handle);
    if (rc < 0)
    {
        std::cerr << "I2C CLOSE ERROR: \'" << lguErrorText(rc) << "\'" << std::endl;
    }
    return rc;
}

