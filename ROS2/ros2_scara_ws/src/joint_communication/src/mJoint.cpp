#include "joint_communication/uI2C.h"
#include "joint_communication/mJoint.h"

Joint::Joint(const int address, std::string name)
{
    this->address = address;
    this->name = name;
}

int Joint::init(void)
{
    std::cout << "INFO: Initializing " << this->name << std::endl;
    this->handle = openI2CDevHandle(this->address);
    if(this->handle < 0){
        return this->handle;
    }
    return checkCom();
}

int Joint::deinit(void)
{

    int rc = 0;
    rc |= this->stop(1);
    rc |= this->disableCL();
    rc |= this->setHoldCurrent(0);
    rc |= this->setBrakeMode(0);
    return rc;
}

int Joint::printInfo(void)
{
    std::cout << "Name: " << this->name << " address: " << this->address << " handle: " << this->handle << std::endl;
    return 0;
}

int Joint::getPosition(float &angle)
{
    return this->read(ANGLEMOVED,angle);
}

int Joint::setPosition(float angle)
{
    return this->write(MOVETOANGLE,angle);
}

int Joint::moveSteps(int32_t steps)
{
    return this->write(MOVESTEPS,steps);
}

int Joint::getVelocity(float &degps)
{
    return this->read(GETENCODERRPM,degps);
    degps *= 6.0;
}

int Joint::setVelocity(float degps)
{
    return this->write(SETRPM,degps/6);
}

int Joint::checkOrientation(float angle)
{
    return this->write(CHECKORIENTATION,angle);
}

int Joint::stop(bool mode)
{
    return this->write(STOP,mode);
}

int Joint::disableCL(void)
{
    u_int8_t buf = 0;
    return this->write(DISABLECLOSEDLOOP,buf);
}

int Joint::setDriveCurrent(u_int8_t current)
{
    return this->write(SETCURRENT,current);
}

int Joint::setHoldCurrent(u_int8_t current)
{
    return this->write(SETHOLDCURRENT,current);
}

int Joint::setBrakeMode(u_int8_t mode)
{
    return this->write(SETBRAKEMODE,mode);
}

int Joint::checkCom(void)
{
    u_int8_t buf;
    int rc = this->read(PING,buf);

    if (buf == 'O' && rc == 0)
    {
        return 0;
    }
    return -1;
}

