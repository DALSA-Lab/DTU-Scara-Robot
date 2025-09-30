#include "bioscara_hardware_driver/uI2C.h"
#include "bioscara_hardware_driver/mJoint.h"

Joint::Joint(const std::string name, const int address, const float reduction, const float offset)
{
    this->address = address;
    this->name = name;
    this->reduction = reduction;
    this->offset = offset;
}

int Joint::init(void)
{
    std::cout << "INFO: Initializing " << this->name << std::endl;
    this->handle = openI2CDevHandle(this->address);
    if (this->handle < 0)
    {
        return -2;
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

int Joint::getPosition(float &pos)
{
    if (!this->isHomed())
    {
        return -2; // not homed
    }
    int rc = this->read(ANGLEMOVED, pos, this->flags);
    pos = ACTUATOR2JOINT(DEG2RAD(pos), this->reduction, this->offset);
    return rc < 0 ? -1 : 0;
}

int Joint::setPosition(float pos)
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

    // inline expansion of the macro did not work, convert before sending to function.
    pos = JOINT2ACTUATOR(RAD2DEG(pos), this->reduction, this->offset);
    rc = this->write(MOVETOANGLE, pos, this->flags);
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

int Joint::getVelocity(float &vel)
{
    if (!this->isHomed())
    {
        return -2; // not homed
    }
    int rc = this->read(GETENCODERRPM, vel, this->flags);
    vel = ACTUATOR2JOINT(DEG2RAD(vel), this->reduction, 0);
    vel *= 6.0; // convert from rpm to rad/s
    return rc < 0 ? -1 : 0;
}

int Joint::setVelocity(float vel)
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

    // inline expansion of the macro did not work, convert before sending to function.
    vel = JOINT2ACTUATOR(RAD2DEG(vel), this->reduction, 0)/6;
    rc = this->write(SETRPM, vel, this->flags);
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
    return this->write(STOP, mode, this->flags) < 0 ? -1 : 0;
}

int Joint::disableCL(void)
{
    u_int8_t buf = 0;
    return this->write(DISABLECLOSEDLOOP, buf, this->flags) < 0 ? -1 : 0;
}

int Joint::setDriveCurrent(u_int8_t current)
{
    return this->write(SETCURRENT, current, this->flags) < 0 ? -1 : 0;
}

int Joint::setHoldCurrent(u_int8_t current)
{
    return this->write(SETHOLDCURRENT, current, this->flags) < 0 ? -1 : 0;
}

int Joint::setBrakeMode(u_int8_t mode)
{
    return this->write(SETBRAKEMODE, mode, this->flags) < 0 ? -1 : 0;
}

int Joint::setMaxAcceleration(float maxAccel)
{
    maxAccel = JOINT2ACTUATOR(RAD2DEG(maxAccel), this->reduction, 0);
    return this->write(SETMAXACCELERATION, maxAccel, this->flags) < 0 ? -1 : 0;
}

int Joint::setMaxVelocity(float maxVel)
{
    maxVel =  JOINT2ACTUATOR(RAD2DEG(maxVel), this->reduction, 0);
    return this->write(SETMAXVELOCITY,maxVel, this->flags) < 0 ? -1 : 0;
}

int Joint::enableStallguard(u_int8_t sensitivity)
{
    return this->write(ENABLESTALLGUARD, sensitivity, this->flags) < 0 ? -1 : 0;
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