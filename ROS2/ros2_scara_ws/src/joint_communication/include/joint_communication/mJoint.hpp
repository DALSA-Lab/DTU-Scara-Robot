#include "joint_communication/mJoint.h"
#include "joint_communication/uI2C.h"
#include "joint_communication/common.h"


template <typename T>
int Joint::read(const stp_reg_t reg, T &data, u_int8_t &flags)
{
    size_t size = sizeof(T) + RFLAGS_SIZE;
    char *buf = new char[size];
    int n = readFromI2CDev(this->handle, reg, buf, size);
    if (n != static_cast<int>(size))
    {
        delete[] buf;
        return -1;
    }
    memcpy(&data, buf, size - RFLAGS_SIZE);
    memcpy(&flags, buf + size - RFLAGS_SIZE, RFLAGS_SIZE);
    delete[] buf;
    return 0;
}

template <typename T>
int Joint::write(const stp_reg_t reg, T data, u_int8_t &flags)
{
    size_t size = sizeof(T) + RFLAGS_SIZE;
    char *buf = new char[size];
    memcpy(buf, &data, size - RFLAGS_SIZE);
    int rc = writeToI2CDev(this->handle, reg, buf, size - RFLAGS_SIZE, buf + size - RFLAGS_SIZE);
    rc = rc > 0 ? 0 : rc;

    memcpy(&flags, buf + size - RFLAGS_SIZE, RFLAGS_SIZE);
    delete[] buf;
    return rc;
}

