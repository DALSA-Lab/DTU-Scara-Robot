#include "joint_communication/mJoint.h"
#include "joint_communication/uI2C.h"

template <typename T>
int Joint::read(const stp_reg_t reg, T &data)
{
    size_t size = sizeof(T);
    char *buf = new char[size];
    int n = readFromI2CDev(this->handle, reg, buf, size);
    if (n != static_cast<int>(size))
    {
        delete[] buf;
        return -1;
    }
    memcpy(&data, buf, size);
    delete[] buf;
    return 0;
}

template <typename T>
int Joint::write(const stp_reg_t reg, T data)
{
    size_t size = sizeof(T);
    char *buf = new char[size];
    memcpy(buf, &data, size);
    int rc = writeToI2CDev(this->handle, reg, buf, size);
    delete[] buf;
    return rc;
}

