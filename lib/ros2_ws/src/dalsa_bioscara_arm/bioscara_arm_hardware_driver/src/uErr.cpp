#include "bioscara_arm_hardware_driver/uErr.h"

namespace bioscara_hardware_drivers
{
    std::string error_to_string(err_type_t err)
    {
        std::string reason;
        switch (err)
        {
        case err_type_t::OK:
            reason = "no error";
            break;
        case err_type_t::ERROR:
            reason = "unspecified error";
            break;
        case err_type_t::NOT_HOMED:
            reason = "not homed";
            break;
        case err_type_t::NOT_ENABLED:
            reason = "not enabled";
            break;
        case err_type_t::STALLED:
            reason = "stalled";
            break;
        case err_type_t::NOT_INIT:
            reason = "not initialized";
            break;
        case err_type_t::COMM_ERROR:
            reason = "communication error";
            break;
        case err_type_t::INVALID_ARGUMENT:
            reason = "invlid argument";
            break;
        case err_type_t::INCORRECT_STATE:
            reason = "incorrect state (busy processing a blocking command)";
            break;
        default:
            reason = "unkown (" + std::to_string((int)err)+")";
        }
        return reason;
    }
} // namespace bioscara_hardware_drivers
