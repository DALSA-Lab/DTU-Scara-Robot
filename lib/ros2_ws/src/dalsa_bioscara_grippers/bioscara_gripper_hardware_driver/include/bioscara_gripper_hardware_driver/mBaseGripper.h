/**
 * @file mBaseGripper.h
 * @author sbstorz
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
#include "bioscara_gripper_hardware_driver/mBaseGripper.h"
#include "bioscara_arm_hardware_driver/uErr.h"

/**
 * @brief Generic BaseGripper object to interact with the robot gripper.
 *
 * This class is a wrapper function to interact with the robot gripper either through a MockGripper
 * or the hardware Gripper.
 *
 * An example application is shown below. Note that depending on the build toolchain the include path can differ. This
example assumes the bioscara_gripper_hardware_driver package is built with ROS2.
 *
 *
 *   \code{.cpp}
// #include "bioscara_gripper_hardware_driver/mGripper.h"
#include "bioscara_gripper_hardware_driver/mMockGripper.h"
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
namespace bioscara_hardware_drivers
{
    class BaseGripper
    {
    public:
        BaseGripper(float reduction, float offset, float min, float max, float backup_init_pos);

        ~BaseGripper(void);

        /**
         * @brief Placeholder, does nothing
         *
         * @return 0
         */
        virtual err_type_t init(void);

        /**
         * @brief Placeholder, does nothing
         *
         * @return 0
         */
        virtual err_type_t deinit(void);

        /**
         * @brief Prepares the servo for use.
         *
         * @return non-zero error code.
         */
        virtual err_type_t enable(void);

        /**
         * @brief Disables the servo.
         *
         * @return non-zero error code.
         */
        virtual err_type_t disable(void);

        /**
         * @brief Sets the gripper width in m from the closed position.
         *
         * Arguments outside the allowed range are bounded to limit min and max.
         * @param width width in m.
         */
        virtual err_type_t setPosition(float width);

        /**
         * @brief Gets the gripper as by the last command.
         *
         * @param width width in m.
         */
        virtual err_type_t getPosition(float &width);

        /**
         * @brief Sets the servo position of the gripper actuator in degrees.
         *
         * @param angle in degrees.
         */
        virtual err_type_t setServoPosition(float angle);

        /**
         * @brief Manually set reduction
         *
         * @param reduction
         */
        virtual void setReduction(float reduction);

        /**
         * @brief Manually set offset
         */
        virtual void setOffset(float offset);

    protected:
        /**
         * @brief Stores the latest position to the buffer file.
         *
         * @param pos
         * @return err_type_t
         */
        err_type_t save_last_position(float pos);

        /**
         * @brief Retrieves the stored position from the buffer file.
         *
         * @param pos
         * @return err_type_t
         */
        err_type_t retrieve_last_position(float &pos);

        float _reduction = 1;          ///< Joint to actuator reduction ratio
        float _offset = 0;             ///< Joint position offset
        float _min = 0;                ///< Joint lower limit
        float _max = 0;                ///< Joint upper limit
        float _backup_init_pos = 0.0;  ///< Initial position used if none can be retrieved from the buffer file
        float _pos = _backup_init_pos; ///< Last received command or stored position

    private:
    };
}
#endif // MBASEGRIPPER_H