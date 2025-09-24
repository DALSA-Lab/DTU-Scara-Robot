/**
 * @file mGripper.h
 * @author Sebastian Storz
 * @brief File containing the Gripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025

 * Include this file for API functions to interact with the gripper.
 *
 */
#ifndef MGRIPPER_H
#define MGRIPPER_H
#include "bioscara_hardware_driver/uPWM.h"

/**
 * @brief Gripper object to interact with the robot gripper.
 *
 * This class is a wrapper function to interact with a PWM servo gripper.
 * An example application is shown below. Note that depending on the build toolchain the include path can differ. This 
example assumes the bioscara_hardware_driver package is built with ROS2.
 * 
 * 
 *   \code{.cpp}
#include "bioscara_hardware_driver/mGripper.h"
int main(int argc, char **argv)
{
    Gripper gripper;
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
class Gripper
{
public:
    Gripper(void);

    /**
     * @brief Placeholder, does nothing
     *
     * @return 0
     */
    int init(void);

    /**
     * @brief Placeholder, does nothing
     *
     * @return 0
     */
    int deinit(void);

    /**
     * @brief Prepares the servo for use.
     *
     * Starts the PWM generation but does not set a position. Must be called before a position is set.
     * The PWM pin is GPIO18. PWM chip is 0, channel 0.     *
     *
     * @return non-zero error code.
     */
    int enable(void);

    /**
     * @brief Disables the servo.
     *
     * Stops the servo and disables the PWM generation.
     *
     * @return non-zero error code.
     */
    int disable(void);

    /**
     * @brief Sets the gripper width in mm from the closed position.
     *
     * Arguments outside the allowed range are bounded to limit.
     * @param width width in mm. 30 - 85 mm are currently allowed. With a new gripper this should be changed.
     */
    int setPosition(float width);

private:
    RPI_PWM pwm;
};
#endif // MGRIPPER_H