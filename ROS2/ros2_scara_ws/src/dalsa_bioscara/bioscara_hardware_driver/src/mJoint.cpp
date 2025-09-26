#include "bioscara_hardware_driver/uI2C.h"
#include "bioscara_hardware_driver/mJoint.h"

Joint::Joint(const int address, const std::string name, const float gearRatio, const float offset)
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
        return -1;
    }
    return checkCom();
}

int Joint::deinit(void)
{
    int rc = closeI2CDevHandle(this->handle);
    return rc < 0 ? -1 : 0;
}
int Joint::disable(void)
{
    int rc = 0;
    rc |= this->stop(1);
    usleep(100000);
    rc |= this->disableCL();
    usleep(10000);
    rc |= this->setHoldCurrent(0);
    usleep(10000);
    rc |= this->setBrakeMode(0);
    usleep(10000);
    return rc < 0 ? -1 : 0;
}

int Joint::enable(u_int8_t driveCurrent, u_int8_t holdCurrent)
{
    u_int32_t buf = 0;                  // Initialize buf to 0
    buf |= (driveCurrent & 0xFF);       // Copy driveCurrent to the least significant byte
    buf |= ((holdCurrent & 0xFF) << 8); // Copy holdCurrent to the next byte

    if (this->write(SETUP, buf, this->flags) < 0)
    {
        return -1;
    }
    do
    {
        usleep(10 * 1000);
    } while (this->getFlags() & (1 << 1));

    if (!this->isEnabled())
    {
        return -3; // NOT HOMED
    }
    return 0;
}

int Joint::home(u_int8_t direction, u_int8_t rpm, u_int8_t sensitivity, u_int8_t current)
{
    if (!this->isEnabled())
    {
        return -3;
    }
    u_int32_t buf = 0;
    buf |= (direction & 0xFF);
    buf |= ((rpm & 0xFF) << 8);
    buf |= ((sensitivity & 0xFF) << 16);
    buf |= ((current & 0xFF) << 24);

    int rc = this->write(HOME, buf, this->flags);
    if (rc < 0)
    {
        return -1;
    }
    usleep(100 * 1000);
    while (this->getFlags() & (1 << 1))
    {
        usleep(10 * 1000);
    }
    if (!this->isHomed())
    {
        return -2; // NOT HOMED
    }
    return 0;
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
    return rc < 0 ? -1 : 0;
}

int Joint::setPosition(float angle)
{
    if (!this->isEnabled())
    {
        return -3; // not enabled
    }
    if (!this->isHomed())
    {
        return -2; // not homed
    }
    int rc;
    rc = this->write(MOVETOANGLE, JOINT2ENCODERANGLE(angle, this->gearRatio, this->offset), this->flags);
    if (rc < 0)
    {
        return -1;
    }
    if (this->isStalled())
    {
        return -4; // STALLED
    }
    return 0;
}

int Joint::moveSteps(int32_t steps)
{
    if (!this->isEnabled())
    {
        return -3; // not enabled
    }
    int rc;
    rc = this->write(MOVESTEPS, steps, this->flags);
    if (rc < 0)
    {
        return -1;
    }
    if (this->isStalled())
    {
        return -4; // STALLED
    }
    return 0;
}

int Joint::getVelocity(float &degps)
{
    int rc = this->read(GETENCODERRPM, degps, this->flags);
    degps = ENCODER2JOINTANGLE(degps, this->gearRatio, 0);
    degps *= 6.0;
    return rc < 0 ? -1 : 0;
}

int Joint::setVelocity(float degps)
{
    if (!this->isEnabled())
    {
        return -3; // not enabled
    }
    if (!this->isHomed())
    {
        return -2; // not homed
    }
    int rc;
    rc = this->write(SETRPM, JOINT2ENCODERANGLE(degps, this->gearRatio, 0) / 6, this->flags);
    if (rc < 0)
    {
        return -1;
    }
    if (this->isStalled())
    {
        return -4; // STALLED
    }
    return 0;
}

int Joint::checkOrientation(float angle)
{
    if (!this->isEnabled())
    {
        return -3; // not enabled
    }
    int rc = this->write(CHECKORIENTATION, angle, this->flags);
    if (rc < 0)
    {
        return -1;
    }
    usleep(10 * 1000);
    while (this->getFlags() & (1 << 1))
    {
        usleep(10 * 1000);
    }

    if (this->isStalled())
    {
        return -4; // STALLED
    }

    return 0;
}

int Joint::stop(bool mode)
{
    int rc = this->write(STOP, mode, this->flags);
    return rc < 0 ? -1 : 0;
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

int Joint::setMaxAcceleration(float maxAccel)
{
    return this->write(SETMAXACCELERATION, JOINT2ENCODERANGLE(maxAccel, this->gearRatio, 0), this->flags);
}

int Joint::setMaxVelocity(float maxVel)
{
    return this->write(SETMAXVELOCITY, JOINT2ENCODERANGLE(maxVel, this->gearRatio, 0), this->flags);
}

int Joint::enableStallguard(u_int8_t sensitivity)
{
    return this->write(ENABLESTALLGUARD, sensitivity, this->flags);
}

bool Joint::isHomed(void)
{
    return ~this->flags & (1 << 2);
}

bool Joint::isEnabled(void)
{
    return ~this->flags & (1 << 3);
}

bool Joint::isStalled(void)
{
    return this->flags & (1 << 0);
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