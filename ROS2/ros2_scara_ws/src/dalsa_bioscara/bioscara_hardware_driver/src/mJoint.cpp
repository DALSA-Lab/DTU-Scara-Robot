#include "bioscara_hardware_driver/uI2C.h"
#include "bioscara_hardware_driver/mJoint.h"
#include <cmath>

Joint::Joint(const std::string name, const int address, const float reduction, const float min, const float max)
{
    this->address = address;
    this->name = name;
    this->reduction = reduction;
    this->min = min;
    this->max = max;
}

Joint::~Joint(void)
{
    this->disable();
    this->deinit();
}

int Joint::init(void)
{
    std::cout << "[INFO] Initializing " << this->name << std::endl;
    this->handle = openI2CDevHandle(this->address);
    if (this->handle < 0)
    {
        return -2;
    }

    /* Check if communication can be established */
    int rc = checkCom();
    if (rc < 0)
    {
        return rc;
    }

    /* If joint is homed, retrieve the homing position stored on the joint */
    if (this->isHomed())
    {
        float offset;
        rc = this->getHomingOffset(offset);
        if (rc < 0)
        {
            return rc;
        }
        this->offset = offset;
    }

    return 0;
}

int Joint::deinit(void)
{
    if (this->handle < 0)
    {
        return -5;
    }
    int rc = closeI2CDevHandle(this->handle);
    return rc < 0 ? -1 : 0;
}

int Joint::enable(u_int8_t driveCurrent, u_int8_t holdCurrent)
{
    if (this->handle < 0)
    {
        return -5;
    }
    u_int32_t buf = 0;                  // Initialize buf to 0
    buf |= (driveCurrent & 0xFF);       // Copy driveCurrent to the least significant byte
    buf |= ((holdCurrent & 0xFF) << 8); // Copy holdCurrent to the next byte
    std::cout << "[INFO] enabling " << this->name << std::endl;
    if (this->write(SETUP, buf, this->flags) < 0)
    {
        return -1;
    }
    this->wait_while_busy(10.0);

    if (!this->isEnabled())
    {
        return -3;
    }
    return 0;
}

int Joint::disable(void)
{
    if (this->handle < 0)
    {
        return -5;
    }
    int rc = 0;
    rc |= this->stop();
    usleep(100000);
    rc |= this->disableCL();
    usleep(10000);
    rc |= this->setHoldCurrent(0);
    usleep(10000);
    rc |= this->setBrakeMode(0);
    usleep(10000);
    return rc < 0 ? -1 : 0;
}

int Joint::home(float velocity, u_int8_t sensitivity, u_int8_t current)
{
    int rc = this->startHoming(velocity, sensitivity, current);
    if (rc < 0)
    {
        return rc;
    }
    this->wait_while_busy(100.0);
    rc = this->postHoming();
    if (rc < 0)
    {
        return rc;
    }
    return 0;
}

int Joint::startHoming(float velocity, u_int8_t sensitivity, u_int8_t current)
{
    if (this->current_b_cmd != NONE)
    {
        return -109; // INCORRECT STATE
    }
    int rc = this->_home(velocity, sensitivity, current);
    if (rc < 0)
    {
        this->current_b_cmd = NONE;
        return rc;
    }
    this->current_b_cmd = HOME;
    return 0;
}

int Joint::postHoming(void)
{
    if (this->current_b_cmd != HOME)
    {
        return -109; // INCORRECT STATE
    }
    this->current_b_cmd = NONE;
    int rc = this->getFlags();
    if (rc < 0)
    {
        return rc;
    }
    if (!this->isHomed())
    {
        return -2; // NOT HOMED
    }
    /* Save the homing position stored on the joint.*/
    rc = this->setHomingOffset(this->offset);
    if (rc < 0)
    {
        return rc;
    }
    return 0;
}

int Joint::_home(float velocity, u_int8_t sensitivity, u_int8_t current)
{
    if (this->handle < 0)
    {
        return -5;
    }
    if (!this->isEnabled())
    {
        return -3;
    }

    velocity = RAD2DEG(JOINT2ACTUATOR(velocity, this->reduction, 0)) / 6;
    if (velocity == 0)
    {
        return -101;
    }
    if (fabs(velocity) > 250.0 || fabs(velocity) < 1.0)
    {
        return -102;
    }

    u_int8_t direction = 0, rpm = 0;
    if (velocity > 0)
    {
        this->offset = this->max;
        direction = 1;
    }
    else
    {
        this->offset = this->min;
        direction = 0;
    }
    velocity = fabs(velocity);
    rpm = static_cast<u_int8_t>(velocity);

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

    return 0;
}

int Joint::printInfo(void)
{
    std::cout << "Name: " << this->name << " address: " << this->address << " handle: " << this->handle << std::endl;
    return 0;
}

int Joint::getPosition(float &pos)
{
    if (this->handle < 0)
    {
        return -5;
    }

    int rc = this->read(ANGLEMOVED, pos, this->flags);
    pos = ACTUATOR2JOINT(DEG2RAD(pos), this->reduction, this->offset);
    if (!this->isHomed())
    {
        pos = 0.0;
    }
    return rc < 0 ? -1 : 0;
}

int Joint::setPosition(float pos)
{
    if (this->handle < 0)
    {
        return -5;
    }
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
    pos = RAD2DEG(JOINT2ACTUATOR(pos, this->reduction, this->offset));
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
    if (this->handle < 0)
    {
        return -5;
    }
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
    if (this->handle < 0)
    {
        return -5;
    }

    int rc = this->read(GETENCODERRPM, vel, this->flags);
    vel = ACTUATOR2JOINT(DEG2RAD(vel), this->reduction, 0);
    vel *= 6.0; // convert from rpm to rad/s
    return rc < 0 ? -1 : 0;
}

int Joint::setVelocity(float vel)
{
    if (this->handle < 0)
    {
        return -5;
    }
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
    vel = RAD2DEG(JOINT2ACTUATOR(vel, this->reduction, 0)) / 6;
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
    if (this->handle < 0)
    {
        return -5;
    }
    if (!this->isEnabled())
    {
        return -3; // not enabled
    }
    int rc = this->write(CHECKORIENTATION, angle, this->flags);
    if (rc < 0)
    {
        return -1;
    }
    this->wait_while_busy(10.0);

    if (this->isStalled())
    {
        return -4; // STALLED
    }

    return 0;
}

int Joint::stop(void)
{
    if (this->handle < 0)
    {
        return -5;
    }
    return this->write(STOP, (u_int8_t)0x00, this->flags) < 0 ? -1 : 0;
}

int Joint::disableCL(void)
{
    if (this->handle < 0)
    {
        return -5;
    }
    u_int8_t buf = 0;
    return this->write(DISABLECLOSEDLOOP, buf, this->flags) < 0 ? -1 : 0;
}

int Joint::setDriveCurrent(u_int8_t current)
{
    if (this->handle < 0)
    {
        return -5;
    }
    return this->write(SETCURRENT, current, this->flags) < 0 ? -1 : 0;
}

int Joint::setHoldCurrent(u_int8_t current)
{
    if (this->handle < 0)
    {
        return -5;
    }
    return this->write(SETHOLDCURRENT, current, this->flags) < 0 ? -1 : 0;
}

int Joint::setBrakeMode(u_int8_t mode)
{
    if (this->handle < 0)
    {
        return -5;
    }
    return this->write(SETBRAKEMODE, mode, this->flags) < 0 ? -1 : 0;
}

int Joint::setMaxAcceleration(float maxAccel)
{
    if (this->handle < 0)
    {
        return -5;
    }
    maxAccel = RAD2DEG(JOINT2ACTUATOR(maxAccel, this->reduction, 0));
    return this->write(SETMAXACCELERATION, maxAccel, this->flags) < 0 ? -1 : 0;
}

int Joint::setMaxVelocity(float maxVel)
{
    if (this->handle < 0)
    {
        return -5;
    }
    maxVel = RAD2DEG(JOINT2ACTUATOR(maxVel, this->reduction, 0));
    return this->write(SETMAXVELOCITY, maxVel, this->flags) < 0 ? -1 : 0;
}

int Joint::enableStallguard(u_int8_t sensitivity)
{
    if (this->handle < 0)
    {
        return -5;
    }
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

bool Joint::isBusy(void)
{
    return this->flags & (1 << 1);
}

int Joint::checkCom(void)
{
    if (this->handle < 0)
    {
        return -5;
    }
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
    if (this->handle < 0)
    {
        return -5;
    }
    u_int8_t buf;
    this->read(PING, buf, this->flags);
    return this->flags;
}

int Joint::getHomingOffset(float &offset)
{
    if (this->handle < 0)
    {
        return -5;
    }
    if (!this->isHomed())
    {
        return -2; // NOT HOMED
    }

    return this->read(HOMEOFFSET, offset, this->flags) < 0 ? -1 : 0;
}

int Joint::setHomingOffset(const float offset)
{
    if (this->handle < 0)
    {
        return -5;
    }
    if (!this->isHomed())
    {
        return -2; // NOT HOMED
    }

    return this->write(HOMEOFFSET, offset, this->flags) < 0 ? -1 : 0;
}

Joint::stp_reg_t Joint::getCurrentBCmd(void)
{
    return this->current_b_cmd;
}

void Joint::wait_while_busy(const float period_ms)
{
    do
    {
        usleep(period_ms * 1000);
        this->getFlags();
    } while (this->isBusy());
}
