#include "bioscara_arm_hardware_driver/uI2C.h"
#include "bioscara_arm_hardware_driver/mJoint.h"
#include <cmath>

namespace bioscara_hardware_drivers
{
    Joint::Joint(const std::string name,
                 const int address,
                 const float reduction,
                 const float min,
                 const float max) : BaseJoint(name)
    {
        this->address = address;
        this->reduction = reduction;
        this->min = min;
        this->max = max;
    }

    Joint::~Joint(void)
    {
        this->disable();
        this->deinit();
    }

    err_type_t Joint::init(void)
    {
        std::cout << "[INFO] Initializing " << this->name << std::endl;
        this->handle = openI2CDevHandle(this->address);

        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        /* Check if communication can be established */
        RETURN_ON_ERROR(this->checkCom());

        /* If joint is homed, retrieve the homing position stored on the joint */
        if (this->isHomed())
        {
            float offset;
            RETURN_ON_ERROR(this->getHomingOffset(offset));
            this->offset = offset;
        }

        return err_type_t::OK;
    }

    err_type_t Joint::deinit(void)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        RETURN_ON_NEGATIVE(closeI2CDevHandle(this->handle), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::enable(u_int8_t driveCurrent, u_int8_t holdCurrent)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        u_int32_t buf = 0;                  // Initialize buf to 0
        buf |= (driveCurrent & 0xFF);       // Copy driveCurrent to the least significant byte
        buf |= ((holdCurrent & 0xFF) << 8); // Copy holdCurrent to the next byte
        std::cout << "[INFO] enabling " << this->name << std::endl;

        RETURN_ON_NEGATIVE(this->write(SETUP, buf, this->flags), err_type_t::COMM_ERROR);
        this->wait_while_busy(10.0);

        RETURN_ON_FALSE(this->isEnabled(), err_type_t::NOT_ENABLED);
        return err_type_t::OK;
    }

    err_type_t Joint::postHoming(void)
    {
        RETURN_ON_ERROR(BaseJoint::postHoming());

        /* Save the homing position stored on the joint.*/
        RETURN_ON_ERROR(this->setHomingOffset(this->offset));

        return err_type_t::OK;
    }

    err_type_t Joint::_home(float velocity, u_int8_t sensitivity, u_int8_t current)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_FALSE(this->isEnabled(), err_type_t::NOT_ENABLED);

        /* Set the offset to min or max, depending on the direction we are homing. */
        this->offset = velocity > 0.0 ? this->max : this->min;

        velocity = RAD2DEG(JOINT2ACTUATOR(velocity, this->reduction, 0)) / 6;
        if (velocity == 0)
        {
            return err_type_t::INVALID_ARGUMENT;
        }
        if (fabs(velocity) > 250.0 || fabs(velocity) < 1.0)
        {
            return err_type_t::INVALID_ARGUMENT;
        }

        u_int8_t direction = velocity > 0.0 ? 1 : 0;

        velocity = fabs(velocity);
        u_int8_t rpm = static_cast<u_int8_t>(velocity);

        u_int32_t buf = 0;
        buf |= (direction & 0xFF);
        buf |= ((rpm & 0xFF) << 8);
        buf |= ((sensitivity & 0xFF) << 16);
        buf |= ((current & 0xFF) << 24);

        RETURN_ON_NEGATIVE(this->write(HOME, buf, this->flags), err_type_t::COMM_ERROR);

        return err_type_t::OK;
    }

    err_type_t Joint::getPosition(float &pos)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_NEGATIVE(this->read(ANGLEMOVED, pos, this->flags), err_type_t::COMM_ERROR);
        pos = ACTUATOR2JOINT(DEG2RAD(pos), this->reduction, this->offset);
        if (!this->isHomed())
        {
            pos = 0.0;
        }
        return err_type_t::OK;
    }

    err_type_t Joint::setPosition(float pos)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        RETURN_ON_FALSE(this->isEnabled(), err_type_t::NOT_ENABLED);

        if (!this->isHomed())
        {
            return err_type_t::NOT_HOMED; // not homed
        }

        pos = RAD2DEG(JOINT2ACTUATOR(pos, this->reduction, this->offset));
        RETURN_ON_NEGATIVE(this->write(MOVETOANGLE, pos, this->flags), err_type_t::COMM_ERROR);
        RETURN_ON_FALSE(!this->isStalled(), err_type_t::STALLED);
        return err_type_t::OK;
    }

    err_type_t Joint::moveSteps(int32_t steps)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        RETURN_ON_FALSE(this->isEnabled(), err_type_t::NOT_ENABLED);

        RETURN_ON_NEGATIVE(this->write(MOVESTEPS, steps, this->flags), err_type_t::COMM_ERROR);
        RETURN_ON_FALSE(!this->isStalled(), err_type_t::STALLED);
        return err_type_t::OK;
    }

    err_type_t Joint::getVelocity(float &vel)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_NEGATIVE(this->read(GETENCODERRPM, vel, this->flags), err_type_t::COMM_ERROR);
        vel = ACTUATOR2JOINT(DEG2RAD(vel), this->reduction, 0);
        vel *= 6.0; // convert from rpm to rad/s
        return err_type_t::OK;
    }

    err_type_t Joint::setVelocity(float vel)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        RETURN_ON_FALSE(this->isEnabled(), err_type_t::NOT_ENABLED);
        RETURN_ON_FALSE(this->isHomed(), err_type_t::NOT_HOMED);

        vel = RAD2DEG(JOINT2ACTUATOR(vel, this->reduction, 0)) / 6;
        RETURN_ON_NEGATIVE(this->write(SETRPM, vel, this->flags), err_type_t::COMM_ERROR);
        RETURN_ON_FALSE(!this->isStalled(), err_type_t::STALLED);
        return err_type_t::OK;
    }

    err_type_t Joint::checkOrientation(float angle)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        RETURN_ON_FALSE(this->isEnabled(), err_type_t::NOT_ENABLED);

        RETURN_ON_NEGATIVE(this->write(CHECKORIENTATION, angle, this->flags), err_type_t::COMM_ERROR);
        this->wait_while_busy(10.0);

        RETURN_ON_FALSE(!this->isStalled(), err_type_t::STALLED);

        return err_type_t::OK;
    }

    err_type_t Joint::stop(void)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_NEGATIVE(this->write(STOP, (u_int8_t)0x00, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::disableCL(void)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        u_int8_t buf = 0;

        RETURN_ON_NEGATIVE(this->write(DISABLECLOSEDLOOP, buf, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::setDriveCurrent(u_int8_t current)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_NEGATIVE(this->write(SETCURRENT, current, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::setHoldCurrent(u_int8_t current)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_NEGATIVE(this->write(SETHOLDCURRENT, current, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::setBrakeMode(u_int8_t mode)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_NEGATIVE(this->write(SETBRAKEMODE, mode, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::setMaxAcceleration(float maxAccel)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        maxAccel = RAD2DEG(JOINT2ACTUATOR(maxAccel, this->reduction, 0));

        RETURN_ON_NEGATIVE(this->write(SETMAXACCELERATION, maxAccel, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::setMaxVelocity(float maxVel)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        maxVel = RAD2DEG(JOINT2ACTUATOR(maxVel, this->reduction, 0));

        RETURN_ON_NEGATIVE(this->write(SETMAXVELOCITY, maxVel, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::enableStallguard(u_int8_t sensitivity)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);

        RETURN_ON_NEGATIVE(this->write(ENABLESTALLGUARD, sensitivity, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::checkCom(void)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        u_int8_t buf;
        RETURN_ON_NEGATIVE(this->read(PING, buf, this->flags), err_type_t::COMM_ERROR);

        if (buf == 'O')
        {
            return err_type_t::OK;
        }
        return err_type_t::ERROR;
    }

    err_type_t Joint::getFlags(void)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        u_int8_t buf;
        RETURN_ON_NEGATIVE(this->read(PING, buf, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::getHomingOffset(float &offset)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        if (!this->isHomed())
        {
            return err_type_t::NOT_HOMED; // NOT HOMED
        }

        RETURN_ON_NEGATIVE(this->read(HOMEOFFSET, offset, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Joint::setHomingOffset(const float offset)
    {
        RETURN_ON_NEGATIVE(this->handle, err_type_t::NOT_INIT);
        if (!this->isHomed())
        {
            return err_type_t::NOT_HOMED; // NOT HOMED
        }

        RETURN_ON_NEGATIVE(this->write(HOMEOFFSET, offset, this->flags), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

} // namespace bioscara_hardware_drivers