#include "bioscara_gripper_hardware_driver/mGripper.h"
#include "bioscara_arm_hardware_driver/uTransmission.h"

namespace bioscara_hardware_drivers
{

    Gripper::Gripper(float reduction, float offset, float min, float max) : BaseGripper()
    {
        this->reduction = reduction;
        this->offset = offset;
        this->min = min;
        this->max = max;
    }

    err_type_t Gripper::enable(void)
    {
        RETURN_ON_NEGATIVE(this->pwm.start(0, this->freq, 0, 0), err_type_t::COMM_ERROR);
        return err_type_t::OK;
    }

    err_type_t Gripper::disable(void)
    {
        this->pwm.stop();
        return err_type_t::OK;
    }

    err_type_t Gripper::setPosition(float width)
    {
        width = width < this->min ? this->min : width;
        width = width > this->max ? this->max : width;

        float angle = JOINT2ACTUATOR(width, this->reduction, this->offset);
        return this->setServoPosition(angle);
    }

    err_type_t Gripper::setServoPosition(float angle)
    {
        float ton_us = angle / 90.0 * 500.0 + 1500.0;         // Ontime [us]
        float dc = ton_us / (1000 * 1000) * this->freq * 100; // dutycycle [%] = ontime [s] /period [s] * 100 %
        for (size_t i = 0; i < 3; i++)
        {
            if (this->pwm.setDutyCycle(dc) >= 0)
            {
                return err_type_t::OK;
            }
        }

        return err_type_t::COMM_ERROR;
    }

    void Gripper::setReduction(float reduction)
    {
        this->reduction = reduction;
    }

    void Gripper::setOffset(float offset)
    {
        this->offset = offset;
    }
}