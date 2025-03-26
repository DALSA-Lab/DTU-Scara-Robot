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

  // int home();

  std::vector<Joint> joints;

protected:
private:
};

#endif