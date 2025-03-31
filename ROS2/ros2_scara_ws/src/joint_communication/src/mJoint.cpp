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
    u_int8_t flags;
    return this->read(ANGLEMOVED,angle, flags);
}

int Joint::setPosition(float angle)
{
    int rc;
    u_int8_t flags;
    rc = this->write(MOVETOANGLE,angle, flags);
    printf("Flags: %#x\n",flags);
    if(rc < 0){
        return rc;
    }
    if(flags & (1 << 0)){
        return 1; // STALLED
    }
    return 0;
}

int Joint::moveSteps(int32_t steps)
{
    u_int8_t flags;
    return this->write(MOVESTEPS,steps, flags);
}

int Joint::getVelocity(float &degps)
{
    u_int8_t flags;
    return this->read(GETENCODERRPM,degps, flags);
    degps *= 6.0;
}

int Joint::setVelocity(float degps)
{
    u_int8_t flags;
    return this->write(SETRPM,degps/6, flags);
}

int Joint::checkOrientation(float angle)
{
    u_int8_t flags;
    return this->write(CHECKORIENTATION,angle, flags);
}

int Joint::stop(bool mode)
{
    u_int8_t flags;
    return this->write(STOP,mode, flags);
}

int Joint::disableCL(void)
{
    u_int8_t flags;
    u_int8_t buf = 0;
    return this->write(DISABLECLOSEDLOOP,buf, flags);
}

int Joint::setDriveCurrent(u_int8_t current)
{
    u_int8_t flags;
    return this->write(SETCURRENT,current, flags);
}

int Joint::setHoldCurrent(u_int8_t current)
{
    u_int8_t flags;
    return this->write(SETHOLDCURRENT,current, flags);
}

int Joint::setBrakeMode(u_int8_t mode)
{
    u_int8_t flags;
    return this->write(SETBRAKEMODE,mode, flags);
}

int Joint::getStall(u_int8_t &stall)
{
    u_int8_t flags;
    return this->read(ISSTALLED,stall, flags);
}

int Joint::enableStallguard(int8_t threshold)
{
    u_int8_t flags;
    return this->write(ENABLESTALLGUARD,threshold, flags);
}

int Joint::checkCom(void)
{
    u_int8_t flags;
    u_int8_t buf;
    int rc = this->read(PING, buf, flags);

    if (buf == 'O' && rc == 0)
    {
        return 0;
    }
    return -1;
}

