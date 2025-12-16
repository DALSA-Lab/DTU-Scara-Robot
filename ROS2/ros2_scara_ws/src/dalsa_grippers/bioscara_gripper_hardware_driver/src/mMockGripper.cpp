#include "bioscara_gripper_hardware_driver/mMockGripper.h"
namespace bioscara_hardware_drivers
{
    MockGripper::MockGripper(float reduction, float offset, float min, float max, float backup_init_pos) : BaseGripper(reduction, offset, min, max, backup_init_pos)
    {
    }
}
