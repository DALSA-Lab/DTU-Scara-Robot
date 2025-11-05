#include "bioscara_hardware_driver/mBaseGripper.h"
namespace bioscara_hardware_driver
{
    BaseGripper::BaseGripper(void)
    {
    }

    err_type_t BaseGripper::init(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseGripper::deinit(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseGripper::enable(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseGripper::disable(void)
    {
        return err_type_t::OK;
    }

    err_type_t BaseGripper::setPosition(float /*width*/)
    {
        return err_type_t::OK;
    }

    err_type_t BaseGripper::setServoPosition(float /*angle*/)
    {
        return err_type_t::OK;
    }

    void BaseGripper::setReduction(float /*reduction*/)
    {
    }

    void BaseGripper::setOffset(float /*offset*/)
    {
    }
}