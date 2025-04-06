#ifndef MJOINTCOM_H
#define MJOINTCOM_H

#include <vector>
#include <iostream>
#include "joint_communication/mJoint.h"

class Joint_comms
{
public:
  /**
   * Constructor to create a join communication object
   * @param n number of joints to connect to.
   * @param addresses n-sized array of 1-byte device adresses
   * @param names n-sized array of string device names
   * @return Joint_comms object.
   */
  Joint_comms(size_t n, const int addresses[], std::string names[]);
  ~Joint_comms();

  int init(void);
  int deinit(void);

  /**
 * Initializes the drivers
 * @param driveCurrent_v vector of drive currents 0-100.
 * @param holdCurrent_v vector of hold currents 0-100.

 * @return error code.
 */
  int setups(std::vector<u_int8_t> driveCurrent_v, std::vector<u_int8_t> holdCurrent_v);

  /**
   * Initializes the drivers, with the same current settings for all
   * @param driveCurrent drive current 0-100.
   * @param holdCurrent hold current 0-100.
   * @return error code.
   */
  int setups(u_int8_t driveCurrent, u_int8_t holdCurrent);

/**
* Homes a joint.
* @param name joint name.
* @param direction  CCW: 0, CW: 1.
* @return error code.
*/
  int home(std::string name, u_int8_t direction = 0);

  int getPositions(std::vector<float> &angle_v);
  int setPositions(std::vector<float> angle_v);
  int getVelocities(std::vector<float> &degps_v);
  int setVelocities(std::vector<float> degps_v);

  /**
   * Sequentially checks the orientations of each joint.
   * @param angle_v vector of degrees to rotate to check the orientation.
   * @return error code.
   */
  int checkOrientations(std::vector<float> angle_v);

  /**
   * Overload to use standard angle of 10 degrees
   * @return error code.
   */
  int checkOrientations(float angle = 10.0);

  /**
   * Stops the motors
   * @param mode Hard: 0, Soft: 1
   * @return error code.
   */
  int stops(bool mode);
  /**
   * Disables the Closed-Loop PID Controllers
   * @return error code.
   */
  int disableCLs(void);

  /**
   * Set the Drive Currents
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setDriveCurrents(std::vector<u_int8_t> current);

  /**
   * Overload to set all drive currents to the same value
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setDriveCurrents(u_int8_t current);

  /**
   * Set the Hold Currents
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setHoldCurrents(std::vector<u_int8_t> current);

  /**
   * Overload to set all hold currents to the same value
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setHoldCurrents(u_int8_t current);

  /**
   * Set Brake Modes
   * @param mode Freewheel: 0, Coolbrake: 1, Hardbrake: 2
   * @return error code.
   */
  int setBrakeModes(u_int8_t mode);

  /**
   * @brief Enable TMC5130 StallGuards for each driver
   * The threshold should be tuned as to trigger stallguard before a step is lost.
   * @param threshold stall sensitivity. A value between -64 and +63
   */
  int enableStallguards(std::vector<int8_t> thresholds);

  // int home();

  std::vector<Joint> joints;

protected:
private:
};

#endif