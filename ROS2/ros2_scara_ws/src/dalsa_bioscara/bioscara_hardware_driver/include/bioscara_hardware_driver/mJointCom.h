/**
 * @file mJointCom.h
 * @author Sebastian Storz
 * @brief File containing the Joint_comms class.
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 * Include this file for API functions to interact with the stepper motors.
 *
 */

#ifndef MJOINTCOM_H
#define MJOINTCOM_H

#include <vector>
#include <unordered_map>
#include <iostream>
#include "bioscara_hardware_driver/mJoint.h"

/**
 * @brief Communication object for all joints.
 *
 * Class handling interfacing with the joints.
 * 
 * The methods of this class are optimized for easy use in the ROS2_control hardware
 * interface methods.
 *
 */
class Joint_comms
{
public:
  Joint_comms(void);
  ~Joint_comms();

  /**
   * @brief Connects to all joints.
   *
   * Iterates over all joints and connects to them on the I2C bus and tests if they are responsive.
   *
   * @warning Add some joints using addJoint() before calling this function.
   *
   * @return 0 on success, non-zero otherwise
   */
  int init(void);

  /**
   * @brief Disconnects all joints from the I2C bus.
   *
   * Deinitializes all joints by removing them from the I2C bus.
   *
   * @return 0 on success, non-zero otherwise
   */
  int deinit(void);

  /**
   * @brief add a Joint.
   *
 * Appends a joint to internal map.
 * @param names string device name for output logs
 * @param addresses 1-byte I2C device adress (0x11 ... 0x14) for J1 ... J4
 * @param gearRatio gear ratio of joint. This is used to transform position
 * and velocity commands in joint units to the stepper units. Signed: sign depends if homed CW or CCW.
 * J1: 35; J2: -360/4 (4 mm per revolution); J3: 24; J4: 12;
 * @param offset offset between encoder zero and joint zero (in joint units).
 * J1: TBD; J2: -TBD (negative because homed at top); J3: TBD; J4: TBD;
 * @todo
 * - Measure joint ranges
 * - Investigate if possible to make independent of homing

 */
  void addJoint(const std::string name, const int address, const float gearRatio, const float offset);


  /**
   * @brief removes a joint.
   *
   * removes a joint from the internal map by name.
   * @param name string device name of the joint to remove
   */
  void removeJoint(const std::string name);

  /**
   * @brief removes all joints from the communication object.
   */
  void removeJoints(void);

//  /**
//  * @brief Engages the joints
//  *
//  * Sets the drive and hold currents for each joint and engages the motor.
//  * Currents are in percent of driver max. output (2.5A, check with datasheet)
//  *
//  * @param driveCurrent_v vector of drive currents 0-100.
//  * the i'th vector entry sets the current for the i'th added joint.
//  * @param holdCurrent_v vector of hold currents 0-100.
//  * the i'th vector entry sets the current for the i'th added joint.

//  * @return error code.
//  */
//   int enables(std::vector<u_int8_t> driveCurrent_v, std::vector<u_int8_t> holdCurrent_v);

  // /**
  //  * @brief Engages the joints with the same current settings for all joints.
  //  *
  //  * In this overload the same drive and hold currents are written to every joint.
  //  * @param driveCurrent drive current 0-100.
  //  * @param holdCurrent hold current 0-100.
  //  * @return error code.
  //  */
  // int enables(u_int8_t driveCurrent, u_int8_t holdCurrent);

     /**
 * @brief Engage a joint by name
 *
 * Sets the drive and hold currents for the specified joint and engages the motor.
 * Currents are in percent of driver max. output (2.5A, check with datasheet)
 *
 * @param driveCurrent drive current 0-100%.
 * @param holdCurrent hold current 0-100%.

 * @return error code.
 */
  int enable(const std::string name, const u_int8_t driveCurrent, const u_int8_t holdCurrent);


  /**
   * @brief Disenganges all joints without closing i2c handle
   *
   * Call this function when the joints should be in freedrive mode.
   * @return error code.
   */
  int disables(void);

  /**
   * @brief Executes the homing sequence of a joint.
   *
   * The joint will drive in the specified direction with the specified speed
   * until a resistance which drives the current above the specified threshold is encountered.
   * At this point the stepper stops and zeros the encoder.
   *
   * @param name joint name.
   * @param direction  CCW: 0, CW: 1.
   * @param rpm  speed of motor in rpm > 10
   * @param sensitivity PID error threshold, 0 to 255.
   * @param current homeing current, determines how easy it is to stop the motor and thereby provoke a stall
   * @return error code.
   */
  int home(const std::string name, const u_int8_t direction, const u_int8_t rpm, const u_int8_t sensitivity, const u_int8_t current);

  // /**
  //  * @brief Get the positions of all joints.
  //  *
  //  * The current positions of all joints are returned. The units are degrees and mm for
  //  * revolute and prismatic joints respectively.
  //  *
  //  * @param angle_v Reference to allocated vector of appropriate size to hold all joint positions.
  //  * @return error code.
  //  */
  // int getPositions(std::vector<float> &angle_v);

    /**
   * @brief Get the position of the joint by name.
   *
   * The current positions of the joint specified by the name is returned. The units are degrees and mm for
   * revolute and prismatic joints respectively.
   *
   * @param angle Reference to allocated variable to hold the joint position.
   * @return error code.
   */
  int getPosition(const std::string name, float &angle);

  // /**
  //  * @brief Set the positions of all joints.
  //  *
  //  * Set new target positons of all joints. The units are degrees and mm for
  //  * revolute and prismatic joints respectively.
  //  *
  //  * @param angle_v Vector of new target positions.
  //  * @return error code.
  //  */
  // int setPositions(std::vector<float> angle_v);

    /**
   * @brief Set the position of the joint by name.
   *
   * Set new target positons of the specified joint. The units are degrees and mm for
   * revolute and prismatic joints respectively.
   *
   * @param angle value of new target positions.
   * @return error code.
   */
  int setPosition(const std::string name, const float angle);

  // /**
  //  * @brief Get the velocities of all joints.
  //  *
  //  * The current velocities of all joints are returned. The units are degrees/s and mm/s for
  //  * revolute and prismatic joints respectively.
  //  *
  //  * @param degps_v Reference to allocated vector of appropriate size to hold all joint velocities.
  //  * @return error code.
  //  */
  // int getVelocities(std::vector<float> &degps_v);

    /**
   * @brief Get the velocity of a joint by name.
   *
   * The current velocity of the specified joint is returned. The units are degrees/s and mm/s for
   * revolute and prismatic joints respectively.
   *
   * @param degps Reference to variable to hold all the joint velocity.
   * @return error code.
   */
  int getVelocity(const std::string name, float &degps);

  // /**
  //  * @brief Set the velocities of all joints.
  //  *
  //  * Set new target velocities of all joints. The units are degrees/s and mm/s for
  //  * revolute and prismatic joints respectively.
  //  *
  //  * @param degps_v Vector of new target velocities.
  //  * @return error code.
  //  */
  // int setVelocities(std::vector<float> degps_v);
  
  /**
   * @brief Set the velocity of a joint by name.
   *
   * Set the new target velocity of the specified joint. The units are degrees/s and mm/s for
   * revolute and prismatic joints respectively.
   *
   * @param degps New target velocity.
   * @return error code.
   */
  int setVelocity(const std::string name, float degps);

  // /**
  //  * @brief Sequentially checks the orientations of each joint.
  //  *
  //  * This function should only be called after the joint has just been powered up.
  //  * This function must be called after the joint has been enabled with enables()
  //  * and before any movement.
  //  * @param angle_v vector of degrees to rotate to check the orientation. Should be small values of a few degrees.
  //  * @return error code.
  //  * @todo
  //  * - Only execute if not performed before
  //  * - save in private flag and inhibit movement if this has not been executed.
  //  */
  // int checkOrientations(std::vector<float> angle_v);

  /**
   * @brief Sequentially checks the orientations of each joint.
   *
   * This function should only be called after the joint has just been powered up.
   * This function must be called after the joint has been enabled with enables()
   * and before any movement.
   * @param angle degrees in motor units to rotate to check the orientation. Should be small values of a few degrees.
   * @return error code.
   * @todo
   * - Only execute if not performed before
   * - save in private flag and inhibit movement if this has not been executed.
   */
  int checkOrientations(float angle = 10.0);

    /**
   * @brief Checks the orientations of the specified joint. This function is automatically called when homing a joint.
   *
   * When checking the orientation the motor moves a few degrees and compares the encoder output. It then internally saves
   * the direction it is wired.
   * This function should only be called after the joint has just been powered up.
   * This function must be called after the joint has been enabled with enable()
   * and before any movement. 
   * @param name name of the joint to check the orientation.
   * @param angle degrees in motor units to rotate to check the orientation. Should be small values of a few degrees.
   * @return error code.
   */
  int checkOrientation(const std::string name, float angle = 10.0);

  /**
   * @brief Stops the motors
   *
   * Stops all motors either soft or hard.
   * @param mode Hard: 0, Soft: 1
   * @return error code.
   */
  int stops(bool mode);

  // /**
  //  * @brief Disables the Closed-Loop PID Controllers
  //  * @return error code.
  //  */
  // int disableCLs(void);

  // /**
  //  * @brief Set the drive Currents.
  //  * @warning This function is unreliable and not well tested. Use enables() instead!
  //  *
  //  *
  //  * @param current 0% - 100% of driver current
  //  * @return error code.
  //  */
  // int setDriveCurrents(std::vector<u_int8_t> current);

  // /**
  //  * @brief Overload to set all drive currents to the same value
  //  * @warning This function is unreliable and not well tested. Use enables() instead!

  //  * @param current 0% - 100% of driver current
  //  * @return error code.
  //  */
  // int setDriveCurrents(u_int8_t current);

  // /**
  //  * @brief Set the Hold Currents
  //  * @warning This function is unreliable and not well tested. Use enables() instead!

  //  * @param current 0% - 100% of driver current
  //  * @return error code.
  //  */
  // int setHoldCurrents(std::vector<u_int8_t> current);

  // /**
  //  * @brief Overload to set all hold currents to the same value
  //  * @warning This function is unreliable and not well tested. Use enables() instead!
  //  *
  //  * @param current 0% - 100% of driver current
  //  * @return error code.
  //  */
  // int setHoldCurrents(u_int8_t current);

  // /**
  //  * @brief Set Brake Modes.
  //  *
  //  * Applies the same brake modes to all joints. usefull to disengage all motors.
  //  * @param mode Freewheel: 0, Coolbrake: 1, Hardbrake: 2
  //  * @return error code.
  //  */
  // int setBrakeModes(u_int8_t mode);

  // /**
  //  * @brief Enable encoder stall detection.
  //  *
  //  * If the PID error exceeds the set threshold a stall is triggered and the motor disabled.
  //  * A detected stall can be reset by homeing.
  //  * @param thresholds Vector of thresholds. 0 - 255 where lower is more sensitive.
  //  */
  // int enableStallguards(std::vector<u_int8_t> thresholds);

    /**
   * @brief Enable encoder stall detection of specified joint.
   *
   * If the PID error exceeds the set threshold a stall is triggered and the motor disabled.
   * A detected stall can be reset by homeing.
   * @param name name of joint to enable stall detection
   * @param thresholds value of threshold. 0 - 255 where lower is more sensitive.
   */
  int enableStallguard(const std::string name, const u_int8_t threshold);

  /**
   * @brief unordered map storing the Joint objects.
   * 
   * an unordered map is chosen to simplify acces via the joint name, as this conforms well with the ROS2_control hardware interface
   * The map does not need to be ordered. Search, insertion, and removal of elements have average constant-time complexity. 
   *
   * A Joint can be added by invoking addJoint()
   * A joint can be removed by invoking remvoveJoint()
   */
  std::unordered_map<std::string,Joint> joints;


protected:
private:
};

#endif