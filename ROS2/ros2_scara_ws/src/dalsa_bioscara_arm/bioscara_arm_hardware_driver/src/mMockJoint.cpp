#include "bioscara_arm_hardware_driver/mMockJoint.h"
#include <unistd.h>

namespace bioscara_hardware_drivers
{
    MockJoint::MockJoint(const std::string name) : BaseJoint(name)
    {
    }

    err_type_t MockJoint::enable(u_int8_t driveCurrent, u_int8_t holdCurrent)
    {
        BaseJoint::enable(driveCurrent, holdCurrent);
        this->flags &= ~(1 << 3);
        return err_type_t::OK;
    }

    err_type_t MockJoint::disable(void)
    {
        BaseJoint::disable();
        this->flags |= (1 << 3);
        return err_type_t::OK;
    }

    err_type_t MockJoint::getPosition(float &pos)
    {
        switch (op_mode)
        {
        case SETRPM:
            /* integrate the latest speed since the last update,
            dont update the last_set_velocity timestamp,
            This is only done when setting a new speed*/
            pos = this->q + this->qd * this->getDeltaT(last_set_velocity, false);
            break;
        case MOVETOANGLE:
            pos = this->q;
        default:
            break;
        }
        return err_type_t::OK;
    }

    err_type_t MockJoint::setPosition(float pos)
    {
        this->qd = (pos - this->q) / this->getDeltaT(last_set_position);
        /* Reset last_set_velocity to now */
        getDeltaT(last_set_velocity);
        this->q = pos;
        op_mode = MOVETOANGLE;
        return err_type_t::OK;
    }

    err_type_t MockJoint::getVelocity(float &vel)
    {
        vel = this->qd;
        return err_type_t::OK;
    }

    err_type_t MockJoint::setVelocity(float vel)
    {
        /* Compute the new position after
         running dt seconds at the constant previous speed */
        this->q = this->q + this->qd * this->getDeltaT(last_set_velocity);
        this->qd = vel;
        op_mode = SETRPM;
        return err_type_t::OK;
    }

    err_type_t MockJoint::checkOrientation(float /*angle*/)
    {
        sleep(1);
        return err_type_t::OK;
    }

    err_type_t MockJoint::stop(void)
    {
        this->qd = 0.0;
        return err_type_t::OK;
    }

    err_type_t MockJoint::_home(float /*velocity*/, u_int8_t /*sensitivity*/, u_int8_t /*current*/)
    {
        getDeltaT(async_start_time);
        this->flags |= (1 << 1); // set BUSY
        this->flags |= (1 << 2); // set NOTHOMED
        return err_type_t::OK;
    }

    err_type_t MockJoint::getFlags(void)
    {
        return BaseJoint::getFlags();
    }

    bool MockJoint::isHomed(void)
    {
        /* If we started homing, and more than 2 seconds have passed, simulate succesfull homing
        by resetting BUSY and NOTHOMED flag */
        if (getCurrentBCmd() == HOME && getDeltaT(async_start_time, false) >= 5.0)
        {
            this->flags &= ~(1 << 1); // reset BUSY
            this->flags &= ~(1 << 2); // reset NOTHOMED
            return 1;
        }

        return BaseJoint::isHomed();
    }

    float MockJoint::getDeltaT(std::chrono::_V2::system_clock::time_point &last_call,
                               bool update)
    {
        auto now = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<float> elapsed = now - last_call;
        if (update)
        {
            last_call = now;
        }
        return elapsed.count(); // elapsed time in seconds
    }
}