#include "bioscara_hardware_driver/mBaseJoint.h"
#include <unistd.h>

BaseJoint::BaseJoint(const std::string name)
{
    this->name = name;
}

BaseJoint::~BaseJoint(void)
{
    this->disable();
    this->deinit();
}

int BaseJoint::init(void)
{
    return 0;
}

int BaseJoint::deinit(void)
{
    return 0;
}

int BaseJoint::enable(u_int8_t /*driveCurrent*/, u_int8_t /*holdCurrent*/)
{
    return 0;
}

int BaseJoint::disable(void)
{
    int rc = this->stop();
    if (rc < 0)
    {
        return rc;
    }
    usleep(100000);
    
    rc = this->setHoldCurrent(0);
    if (rc < 0)
    {
        return rc;
    }
    usleep(10000);

    rc = this->setBrakeMode(0);
    if (rc < 0)
    {
        return rc;
    }
    return 0;
}

int BaseJoint::home(float velocity, u_int8_t sensitivity, u_int8_t current)
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

int BaseJoint::startHoming(float velocity, u_int8_t sensitivity, u_int8_t current)
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

int BaseJoint::postHoming(void)
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
    return 0;
}

int BaseJoint::setPosition(float /*pos*/)
{
    return 0;
}

int BaseJoint::moveSteps(int32_t /*steps*/)
{
    return 0;
}

int BaseJoint::setVelocity(float /*vel*/)
{
    return 0;
}

int BaseJoint::checkOrientation(float /*angle*/)
{
    return 0;
}

int BaseJoint::stop(void)
{
    return 0;
}

int BaseJoint::disableCL(void)
{
    return 0;
}

int BaseJoint::setDriveCurrent(u_int8_t /*current*/)
{
    return 0;
}

int BaseJoint::setHoldCurrent(u_int8_t /*current*/)
{
    return 0;
}

int BaseJoint::setBrakeMode(u_int8_t /*mode*/)
{
    return 0;
}

int BaseJoint::setMaxAcceleration(float /*maxAccel*/)
{
    return 0;
}

int BaseJoint::setMaxVelocity(float /*maxVel*/)
{
    return 0;
}

int BaseJoint::enableStallguard(u_int8_t /*sensitivity*/)
{
    return 0;
}

bool BaseJoint::isHomed(void)
{
    return ~this->flags & (1 << 2);
}

bool BaseJoint::isEnabled(void)
{
    return ~this->flags & (1 << 3);
}

bool BaseJoint::isStalled(void)
{
    return this->flags & (1 << 0);
}

bool BaseJoint::isBusy(void)
{
    return this->flags & (1 << 1);
}

u_int8_t BaseJoint::getFlags(void)
{
    return this->flags;
}

BaseJoint::stp_reg_t BaseJoint::getCurrentBCmd(void)
{
    return this->current_b_cmd;
}

void BaseJoint::wait_while_busy(const float period_ms)
{
    do
    {
        usleep(period_ms * 1000);
        this->getFlags();
    } while (this->isBusy());
}
