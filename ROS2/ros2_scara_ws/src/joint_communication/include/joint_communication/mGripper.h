/**
 * @file mGripper.h
 * @author Sebastian Storz
 * @brief File containg the Gripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef MGRIPPER_H
#define MGRIPPER_H
#include "joint_communication/uPWM.h"

/**
 * @brief Gripper object to interact with the robot gripper.
 *
 * The gripper must be a PWM controlled servo.
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