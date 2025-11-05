#include "bioscara_hardware_driver/mBaseJoint.h"
#include <unistd.h>

namespace bioscara_hardware_driver
{
    BaseJoint::BaseJoint(const std::string name)
    {
        this->name = name;
    }

    BaseJoint::~BaseJoint(void)
    {
        this->disable();
        this->deinit();
    }

    err_type_t BaseJoint::init(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::deinit(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::enable(u_int8_t /*driveCurrent*/, u_int8_t /*holdCurrent*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::disable(void)
    {

        RETURN_ON_ERROR(this->stop());
        usleep(100000);

        RETURN_ON_ERROR(this->setHoldCurrent(0));
        usleep(10000);

        RETURN_ON_ERROR(this->setBrakeMode(0));
        usleep(10000);
        return err_type_t::OK;
    }

    err_type_t BaseJoint::home(float velocity, u_int8_t sensitivity, u_int8_t current)
    {
        RETURN_ON_ERROR(this->startHoming(velocity, sensitivity, current));

        this->wait_while_busy(100.0);
        RETURN_ON_ERROR(this->postHoming());
        return err_type_t::OK;
    }

    err_type_t BaseJoint::startHoming(float velocity, u_int8_t sensitivity, u_int8_t current)
    {
        if (this->current_b_cmd != NONE)
        {
            return err_type_t::INCORRECT_STATE;
        }
        err_type_t rc = this->_home(velocity, sensitivity, current);
        if (rc != err_type_t::OK)
        {
            this->current_b_cmd = NONE;
            return rc;
        }
        this->current_b_cmd = HOME;
        return err_type_t::OK;
    }

    err_type_t BaseJoint::postHoming(void)
    {
        if (this->current_b_cmd != HOME)
        {
            return err_type_t::INCORRECT_STATE;
        }
        this->current_b_cmd = NONE;
        RETURN_ON_ERROR(this->getFlags());
        RETURN_ON_FALSE(this->isHomed(), err_type_t::NOT_HOMED);

        return err_type_t::OK;
    }

    err_type_t BaseJoint::setPosition(float /*pos*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::moveSteps(int32_t /*steps*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::setVelocity(float /*vel*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::checkOrientation(float /*angle*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::stop(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::disableCL(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::setDriveCurrent(u_int8_t /*current*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::setHoldCurrent(u_int8_t /*current*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::setBrakeMode(u_int8_t /*mode*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::setMaxAcceleration(float /*maxAccel*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::setMaxVelocity(float /*maxVel*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseJoint::enableStallguard(u_int8_t /*sensitivity*/)
    {
        return err_type_t::OK;
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

    err_type_t BaseJoint::getFlags(u_int8_t &flags)
    {
        RETURN_ON_ERROR(this->getFlags());
        flags = this->flags;
        return err_type_t::OK;
    }

    err_type_t BaseJoint::getFlags(void)
    {
        return err_type_t::OK;
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
}