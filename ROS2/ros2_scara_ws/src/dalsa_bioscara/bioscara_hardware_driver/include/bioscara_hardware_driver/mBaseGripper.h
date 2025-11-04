/**
 * @file mBaseGripper.h
 * @author Sebastian Storz
 * @brief File containing the BaseGripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025

 * Dont include this file directly, instead use one of the derived classes.
 *
 */
#ifndef MBASEGRIPPER_H
#define MBASEGRIPPER_H
#include "bioscara_hardware_driver/mBaseGripper.h"

/**
 * @brief Generic BaseGripper object to interact with the robot gripper.
 *
 * This class is a wrapper function to interact with the robot gripper either through a MockGripper
 * or the hardware Gripper.
 * 
 * An example application is shown below. Note that depending on the build toolchain the include path can differ. This
example assumes the bioscara_hardware_driver package is built with ROS2.
 *
 *
 *   \code{.cpp}
// #include "bioscara_hardware_driver/mGripper.h"
#include "bioscara_hardware_driver/mMockGripper.h"
int main(int argc, char **argv)
{
    MockGripper gripper;
    gripper.init();
    if(gripper.enable() != 0){
        cerr << "Failed to engage gripper" << endl;
        return -1;
    }

    if (gripper.setPosition(40) != 0)
    {
        cerr << "setting position failed" << endl;
        return -1;
    }

    if(gripper.disable() != 0){
        cerr << "Failed to disengage gripper" << endl;
        return -1;
    }

    gripper.deinit();
    return 0;
}
  \endcode
 *
 *
 */
class BaseGripper
{
public:
    BaseGripper(void);

    /**
     * @brief Placeholder, does nothing
     *
     * @return 0
     */
    virtual int init(void);

    /**
     * @brief Placeholder, does nothing
     *
     * @return 0
     */
    virtual int deinit(void);

    /**
     * @brief Prepares the servo for use.
     *
     * @return non-zero error code.
     */
    virtual int enable(void);

    /**
     * @brief Disables the servo.
     *
     * @return non-zero error code.
     */
    virtual int disable(void);

    /**
     * @brief Sets the gripper width in m from the closed position.
     *
     * Arguments outside the allowed range are bounded to limit min and max.
     * @param width width in m.
     */
    virtual int setPosition(float width);

protected:
private:
};
#endif // MBASEGRIPPER_H