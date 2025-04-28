#include "joint_communication/uI2C.h"
#include "joint_communication/mJoint.h"

Joint::Joint(const int address, const std::string name, const float gearRatio, const int offset)
{
    this->address = address;
    this->name = name;
    this->gearRatio = gearRatio;
    this->offset = offset;
}

int Joint::init(void)
{
    std::cout << "INFO: Initializing " << this->name << std::endl;
    this->handle = openI2CDevHandle(this->address);
    if (this->handle < 0)
    {
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

int Joint::setup(u_int8_t driveCurrent, u_int8_t holdCurrent)
{
    u_int32_t buf = 0;                  // Initialize buf to 0
    buf |= (driveCurrent & 0xFF);       // Copy driveCurrent to the least significant byte
    buf |= ((holdCurrent & 0xFF) << 8); // Copy holdCurrent to the next byte

    this->getIsHomed();
    return this->write(SETUP, buf, this->flags);
}

int Joint::home(u_int8_t direction, u_int8_t rpm, int8_t sensitivity, u_int8_t current)
{
    u_int32_t buf = 0;
    buf |= (direction & 0xFF);
    buf |= ((rpm & 0xFF) << 8);
    buf |= ((sensitivity & 0xFF) << 16);
    buf |= ((current & 0xFF) << 24);

    return this->write(HOME, buf, this->flags);
}

int Joint::printInfo(void)
{
    std::cout << "Name: " << this->name << " address: " << this->address << " handle: " << this->handle << std::endl;
    return 0;
}

int Joint::getPosition(float &angle)
{
    int rc = this->read(ANGLEMOVED, angle, this->flags);
    angle = ENCODER2JOINTANGLE(angle, this->gearRatio, this->offset);
    return rc;
}

int Joint::setPosition(float angle)
{
    if(!this->homed){
        return 2; // not homed
    }
    int rc;
    rc = this->write(MOVETOANGLE, JOINT2ENCODERANGLE(angle, this->gearRatio, this->offset), this->flags);
    if (rc < 0)
    {
        return rc;
    }

    printf("Flags: %#x\n", this->flags);
    if (this->flags & (1 << 0))
    {
        return 1; // STALLED
    }
    return 0;
}

int Joint::moveSteps(int32_t steps)
{
    int rc;
    rc = this->write(MOVESTEPS, steps, this->flags);
    if (rc < 0)
    {
        return rc;
    }

    printf("Flags: %#x\n", this->flags);
    if (this->flags & (1 << 0))
    {
        return 1; // STALLED
    }
    return 0;
}

int Joint::getVelocity(float &degps)
{
    int rc = this->read(GETENCODERRPM, degps, this->flags);
    degps = ENCODER2JOINTANGLE(degps, this->gearRatio, 0);
    degps *= 6.0;
    return rc;
}

int Joint::setVelocity(float degps)
{
    if(!homed){
        return 2; // not homed
    }
    int rc;
    rc = this->write(SETRPM, JOINT2ENCODERANGLE(degps, this->gearRatio, 0) / 6, this->flags);
    if (rc < 0)
    {
        return rc;
    }
    printf("Flags: %#x\n", this->flags);
    if (this->flags & (1 << 0))
    {
        return 1; // STALLED
    }
    return 0;
}

int Joint::checkOrientation(float angle)
{
    return this->write(CHECKORIENTATION, angle, this->flags);
}

int Joint::stop(bool mode)
{
    return this->write(STOP, mode, this->flags);
}

int Joint::disableCL(void)
{
    u_int8_t buf = 0;
    return this->write(DISABLECLOSEDLOOP, buf, this->flags);
}

int Joint::setDriveCurrent(u_int8_t current)
{
    return this->write(SETCURRENT, current, this->flags);
}

int Joint::setHoldCurrent(u_int8_t current)
{
    return this->write(SETHOLDCURRENT, current, this->flags);
}

int Joint::setBrakeMode(u_int8_t mode)
{
    return this->write(SETBRAKEMODE, mode, this->flags);
}

int Joint::getStall(u_int8_t &stall)
{
    return this->read(ISSTALLED, stall, this->flags);
}

int Joint::enableStallguard(int8_t threshold)
{
    return this->write(ENABLESTALLGUARD, threshold, this->flags);
}

int Joint::getIsHomed(u_int8_t &homed){
    int rc = this->read(ISHOMED, homed, this->flags);
    return rc;
}

int Joint::getIsHomed(void){
    int rc = this->read(ISHOMED, this->homed, this->flags);
    return rc;
}

bool Joint::isHomed(void){
    return this->homed;
}

int Joint::checkCom(void)
{
    u_int8_t buf;
    int rc = this->read(PING, buf, this->flags);

    if (buf == 'O' && rc == 0)
    {
        return 0;
    }
    return -1;
}

u_int8_t Joint::getFlags(void)
{
    u_int8_t buf;
    this->read(PING, buf, this->flags);
    return this->flags;
}