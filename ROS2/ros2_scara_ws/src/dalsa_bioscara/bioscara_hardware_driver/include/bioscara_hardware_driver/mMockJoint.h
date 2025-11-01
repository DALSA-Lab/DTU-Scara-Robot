/**
 * @file mMockJoint.h
 * @author Sebastian Storz
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
#include "bioscara_hardware_driver/mBaseJoint.h"
#include <chrono>

class MockJoint : public BaseJoint
{
public:
  MockJoint(const std::string name);

  int enable(u_int8_t driveCurrent, u_int8_t holdCurrent) override;

  int disable(void) override;

  int getPosition(float &pos) override;

  int setPosition(float pos) override;

  int getVelocity(float &vel) override;

  int setVelocity(float vel) override;

  int checkOrientation(float angle = 10.0) override;

  int stop(void) override;

  u_int8_t getFlags(void) override;

protected:
  int _home(float velocity, u_int8_t sensitivity, u_int8_t current);

private:
  float q = 0.0;
  float qd = 0.0;

  std::chrono::_V2::system_clock::time_point last_set_position = std::chrono::high_resolution_clock::now();
  std::chrono::_V2::system_clock::time_point last_set_velocity = last_set_position;
  std::chrono::_V2::system_clock::time_point async_start_time = last_set_position;
  float getDeltaT(std::chrono::_V2::system_clock::time_point &last_call, bool update = true);

  stp_reg_t op_mode = NONE;
};

#endif