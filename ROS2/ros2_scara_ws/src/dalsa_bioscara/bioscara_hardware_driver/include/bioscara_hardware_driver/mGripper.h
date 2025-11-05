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
#include "bioscara_hardware_driver/mBaseGripper.h"
#include "bioscara_hardware_driver/uPWM.h"

/**
 * @brief Derviced class from the BaseGripper class to interact with the hardware gripper.
 *
 * This class is a wrapper function to interact with a PWM servo gripper.
 *
 */
class Gripper : public BaseGripper
{
public:

    /**
     * @brief Constructor of the hardware Gripper object. 
     * 
     * The gripper width in m is converted to a PWM dutycyle via the JOINT2ACTUATOR macro.
     * 
     * @param reduction 
     * @param offset 
     * @param min minimum width in m.
     * @param max maxmimum width in m.
     */
    Gripper(float reduction, float offset, float min, float max);

    /**
     * @brief Prepares the servo for use.
     *
     * Starts the PWM generation but does not set a position. Must be called before a position is set.
     * The PWM pin is GPIO18. PWM chip is 0, channel 0.
     *
     * @return non-zero error code.
     */
    int enable(void) override;

    /**
     * @brief Disables the servo.
     *
     * Stops the servo and disables the PWM generation.
     *
     * @return non-zero error code.
     */
    int disable(void) override;


    int setPosition(float width) override;

    int setServoPosition(float angle) override;

protected:
    float reduction = 1; ///< Joint to actuator reduction ratio
    float offset = 0;    ///< Joint position offset
    float min = 0;       ///< Joint lower limit
    float max = 0;       ///< Joint upper limit
private:
    RPI_PWM pwm;
    int freq = 50;
};
#endif // MGRIPPER_H