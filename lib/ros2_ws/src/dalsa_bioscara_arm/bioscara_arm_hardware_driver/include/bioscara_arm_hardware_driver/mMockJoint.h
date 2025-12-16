/**
 * @file mMockJoint.h
 * @author sbstorz
 * @brief File including the MockJoint class
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef MMOCKJOINT_H
#define MMOCKJOINT_H

#include <iostream>
#include "bioscara_arm_hardware_driver/mBaseJoint.h"
#include <chrono>

namespace bioscara_hardware_drivers
{
  class MockJoint : public BaseJoint
  {
  public:
    MockJoint(const std::string name);

    err_type_t enable(u_int8_t driveCurrent, u_int8_t holdCurrent) override;

    err_type_t disable(void) override;

    err_type_t getPosition(float &pos) override;

    err_type_t setPosition(float pos) override;

    err_type_t getVelocity(float &vel) override;

    err_type_t setVelocity(float vel) override;

    err_type_t checkOrientation(float angle = 10.0) override;

    err_type_t stop(void) override;

    err_type_t getFlags(void) override;

    bool isHomed(void) override;

  protected:
    err_type_t _home(float velocity, u_int8_t sensitivity, u_int8_t current);

  private:
    float q = 0.0;
    float qd = 0.0;

    std::chrono::_V2::system_clock::time_point last_set_position = std::chrono::high_resolution_clock::now();
    std::chrono::_V2::system_clock::time_point last_set_velocity = last_set_position;
    std::chrono::_V2::system_clock::time_point async_start_time = last_set_position;
    float getDeltaT(std::chrono::_V2::system_clock::time_point &last_call, bool update = true);

    stp_reg_t op_mode = NONE;
  };
}
#endif