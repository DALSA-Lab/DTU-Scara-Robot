/**
 * @file mMockGripper.h
 * @author Sebastian Storz
 * @brief File containing the MockGripper class
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025

 * Include this file for API functions to interact with the MockGripper.
 *
 */
#ifndef MMOCKGRIPPER_H
#define MMOCKGRIPPER_H
#include "bioscara_hardware_driver/mBaseGripper.h"
#include "bioscara_hardware_driver/uErr.h"
/**
 * @copydoc BaseGripper
 */

namespace bioscara_hardware_driver
{
    class MockGripper : public BaseGripper
    {
    public:
        MockGripper(void);

    protected:
    private:
    };
}
#endif // MMOCKGRIPPER_H