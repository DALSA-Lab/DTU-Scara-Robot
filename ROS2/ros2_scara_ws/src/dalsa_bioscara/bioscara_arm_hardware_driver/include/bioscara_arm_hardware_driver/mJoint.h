/**
 * @file mJoint.h
 * @author Sebastian Storz
 * @brief File including the Joint class
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef MJOINT_H
#define MJOINT_H

#include <iostream>
#include "bioscara_arm_hardware_driver/mBaseJoint.h"
#include "bioscara_arm_hardware_driver/uTransmission.h"

namespace bioscara_hardware_drivers
{
  /**
   * @brief Representing a single joint on the I2C bus
   *
   */
  class Joint : public BaseJoint
  {
  public:
    /**
     * @brief Create a Joint object
     *
     * The Joint object represents a single joint and its actuator.
     * Each Joint has a transmission with the following relationship: \n
     *
     * > actuator position = (joint position - offset) * reduction \n
     * > joint position = actuator position / reduction + offset
     *
     *
     * @param name string device name for identification
     * @param address 1-byte I2C device adress (0x11 ... 0x14) for J1 ... J4
     * @param reduction gear reduction of the joint. This is used to transform position
     * and velocity values between in joint units and actuator (stepper) units.
     * The sign depends on the direction the motor is mounted and is turning. Adjust such that the joint moves in the positive
     * direction on on positive joint commands. Cable polarity has no effect since the motors
     * automatically adjust to always run in the 'right' direction from their point of view. \n
     * J1: 35 \n
     * J2: -2*pi/0.004 (4 mm linear movement per stepper revolution) \n
     * J3: 24 \n
     * J4: 12
     * @param min lower joint limit in joint units. \n
     * J1: -3.04647 \n
     * J2: -0.0016 \n
     * J3: -2.62672 \n
     * J4: -3.01069
     * @param max upper joint limit in joint units. \n
     * J1: 3.04647 \n
     * J2: 0.3380 \n
     * J3: 2.62672 \n
     * J4: 3.01069
     */
    Joint(const std::string name, const int address, const float reduction, const float min, const float max);
    ~Joint(void);

    /**
     * @brief Established connection to a joint via I2C
     *
     * Adds the joint to the I2C bus and tests if is responsive by sending a PING.
     *
     * @return 0 on success,
     *  -1 on when no ACK is received from the joint,
     *  -2 if the I2C device could not be opened given the joint address.
     */
    err_type_t init(void) override;

    /**
     * @brief Disconnects from a joint.
     *
     * Removes the joint from the I2C bus.
     *
     * @return 0 on success,
     *  -1 when the joint could not be removed due to an I2C error,
     *  -5 if the joint is not initialized.
     */
    err_type_t deinit(void) override;

    /**
     * @brief Setup the joint and engages motor.
     *
     * This function prepares the motor for movement. After successfull execution the joint
     * is ready to accept Joint::setPosition() and Joint::setVelocity() commands. \n
     * The function ets the drive and hold current for the specified joint and engages the motor.
     * The currents are in percent of driver max. output (2.5A, check with TMC5130 datasheet or Ustepper documentation)
     * @param driveCurrent drive current in 0-100 % of 2.5A output (check uStepper doc.)
     * @param holdCurrent hold current in 0-100 % of 2.5A output (check uStepper doc.)
     * @return 0 on success,
      -1 on communication error,
      -3 when the motor is not enabled,
      -5 if the joint is not initialized.
     */
    err_type_t enable(u_int8_t driveCurrent, u_int8_t holdCurrent) override;

    // /**
    //  * @brief disenganges the joint motor without closing i2c handle
    //  * @return 0 on success,
    //  * -1 on communication error,
    //   -5 if the joint is not initialized.
    //  */
    // int disable(void);

    // /**
    //  * @brief Blocking implementation to home the joint.
    //  *
    //  * A blocking implementation which only returns after the the joint is no longer BUSY. See Joint::_home() for documentation.
    //  *
    //  * Additionally this method returns:
    //  * @return -2 when not homed succesfull (isHomed flag still not set),
    //  * -109 if the joint is already currently homing (for example from a call to Joint::startHoming()).
    //  */
    // int home(float velocity, u_int8_t sensitivity, u_int8_t current);

    // /**
    //  * @brief non-blocking implementation to home the joint.
    //  *
    //  * See Joint::_home() for documentation. The current_b_cmd flag is set to HOME
    //  * This method returns immediatly after starting the homing sequence. This should be used when the blocking implementation is not acceptable.
    //  * For example in the update loop of the bioscara_hardware_interfaces::BioscaraHardwareInterface::write().

    //  * Additionally this method returns:
    //  * @return -109 if the joint is already currently homing (for example from a call to Joint::startHoming()).
    //  */
    // int startHoming(float velocity, u_int8_t sensitivity, u_int8_t current);

    /**
     * @brief perform tasks after a non-blocking homing.
     *
     * This method resets the current_b_cmd to NONE, checks if the joint is homed,
     * and saves the homing offset to the joint.
     *
     * @return 0 on success,
     * -109 if the current_b_cmd is not HOME,
     * -1 on communication error,
     * -2 when not homed,
     * -5 if the joint is not initialized.
     *
     */
    err_type_t postHoming(void) override;

    /**
     * @brief get the current joint position in radians or m for
     * cylindrical and prismatic joints respectively.
     *
     * @warning If the joint is not homed this method does not return an error.
     * Instead `pos` will be 0.0.
     *
     * @param pos
     * @return 0 on success,
      -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t getPosition(float &pos) override;

    /**
     * @brief set the current joint position in radians or m for
     * cylindrical and prismatic joints respectively.
     *
     * @param pos in rad or m
     * @return 0 on success,
      -1 on communication error,
      -2 when not homed,
      -3 when the motor is not enabled,
      -4 when the motor is stalled,
      -5 if the joint is not initialized.
     */
    err_type_t setPosition(float pos) override;

    /**
     * @brief Move full steps.
     *
     * This function can be called even when not homed.
     *
     * @param steps number of full steps
     * @return 0 on success,
      -1 on communication error,
      -3 when the motor is not enabled,
      -4 when the motor is stalled,
      -5 if the joint is not initialized.
     */
    err_type_t moveSteps(int32_t steps) override;

    /**
     * @brief get the current joint velocity in radians/s or m/s for
     * cylindrical and prismatic joints respectively.
     *
     * @param vel
     * @return 0 on success,
      -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t getVelocity(float &vel) override;

    /**
     * @brief Set the current joint velocity in radians/s or m/s for
     * cylindrical and prismatic joints respectively.
     *
     * @param vel
     * @return 0 on success,
      -1 on communication error,
      -2 when not homed,
      -3 when the motor is not enabled,
      -4 when the motor is stalled,
      -5 if the joint is not initialized.
     */
    err_type_t setVelocity(float vel) override;

    /**
     * @brief Calls the checkOrientation method of the motor. Checks in which direction the motor is turning.
     *
     * As the orientation check is blocking on the motor, this this function returns when the isBusy flag is clear again.
     *
     * @param angle degrees how much the motor should turn. A few degrees is sufficient.
     * @return 0 on success,
      -1 on communication error,
      -3 when the motor is not enabled,
      -4 when the motor is stalled,
      -5 if the joint is not initialized.
     */
    err_type_t checkOrientation(float angle = 10.0) override;

    /**
     * @brief Stops the motor.
     *
     * Stops the motor by setting the maximum velocity to zero and the position setpoint
     * to the current position
     *
     * @return 0 on success,
     * -1 on communication error,
     * -5 if the joint is not initialized.
     */
    err_type_t stop(void) override;
    /**
     * @brief Disables the Closed-Loop PID Controller
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t disableCL(void) override;

    /**
     * @brief Set the Drive Current
     * @warning This function is unreliable and not well tested. Use Joint::enable() instead!
     * @param current 0% - 100% of driver current
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t setDriveCurrent(u_int8_t current) override;

    /**
     * @brief Set the Hold Current
     * @warning This function is unreliable and not well tested. Use Joint::enable() instead!
     * @param current 0% - 100% of driver current
     * @return 0 on success,
     * -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t setHoldCurrent(u_int8_t current) override;

    /**
     * @brief Set Brake Mode
     * @param mode Freewheel: 0, Coolbrake: 1, Hardbrake: 2
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t setBrakeMode(u_int8_t mode) override;

    /**
     * @brief Set the maximum permitted joint acceleration (and deceleration) in rad/s^2 or m/s^2 for cylindrical
     * and prismatic joints respectively.
     *
     * @param maxAccel maximum joint acceleration.
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t setMaxAcceleration(float maxAccel) override;

    /**
     * @brief Set the maximum permitted joint velocity in rad/s or m/s for cylindrical
     * and prismatic joints respectively.
     *
     * @param maxVel maximum joint velocity.
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t setMaxVelocity(float maxVel) override;

    /**
     * @brief Enable encoder stall detection of the joint.
     *
     * If the PID error exceeds the set threshold a stall is triggered and the motor disabled.
     * A detected stall can be reset by homing or by reenabling the stall guard.
     * @param sensitivity value of threshold. 0 - 255 where lower is more sensitive.
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    err_type_t enableStallguard(u_int8_t sensitivity) override;

    // /**
    //  * @brief Checks the state if the motor is homed.
    //  *
    //  * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
    //  *
    //  * @return true if the motor is homed,
    //  * false if not.
    //  */
    // bool isHomed(void);

    // /**
    //  * @brief Checks the state if the motor is enabled.
    //  *
    //  * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
    //  * If the motor actually can move depends on the state of the STALLED flag which can be checked using Joint::isStalled().
    //  *
    //  * @return true if the motor is enabled,
    //  * false if not.
    //  */
    // bool isEnabled(void);

    // /**
    //  * @brief Checks if the motor is stalled.
    //  *
    //  * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
    //  * @return true if the motor is stalled,
    //  * false if not.
    //  */
    // bool isStalled(void);

    // /**
    //  * @brief Checks if the joint controller is busy processing a blocking command.
    //  *
    //  * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
    //  * @return true if a blocking command is currently executing,
    //  * false if not.
    //  */
    // bool isBusy(void);

    /**
     * get driver state flags
     * @return 0 on success,
      -5 if the joint is not initialized.
     */
    err_type_t getFlags(void) override;

    /**
     * @brief Retrieves the homing position from the last homing.
     *
     * The homing position is stored on the joint to make it persistent as long as the joint is powered up.
     *
     * @return 0 on success,
     * -1 on communication error,
     * -2 when not homed,
     * -5 if the joint is not initialized.
     */
    err_type_t getHomingOffset(float &offset);

    /**
     * @brief Stores the homing position on the joint.
     *
     * The homing position is stored on the joint to make it persistent as long as the joint is powered up.
     *
     * @return 0 on success,
     * -1 on communication error,
     * -2 if not homed,
     * -5 if the joint is not initialized.
     */
    err_type_t setHomingOffset(const float offset);

    // /**
    //  * @brief get the currently active blocking command
    //  *
    //  * @return The the command of type stp_reg_t
    //  */
    // stp_reg_t getCurrentBCmd(void);

    std::string name;

  protected:
    // /**
    //  * @brief Blocking loop waiting for BUSY flag to reset.
    //  *
    //  * @param period_ms time in ms between polls.
    //  */
    // void wait_while_busy(const float period_ms);

    /**
     * @brief Call to start the homing sequence of a joint.
     *
     * First the joint will check the motor wiring by executing the checkOrientation internally. Then it will set the specified speed
     * until a resistance which drives the PID error above the specified threshold is encountered.
     * At this point the stepper stops and zeros the encoder.
     * @param velocity  signed velocity in rad/s or m/s. Must be between 1.0 < RAD2DEG(JOINT2ACTUATOR(velocity, reduction, 0)) / 6 < 250.0
     * @param sensitivity Encoder pid error threshold 0 to 255.
     * @param current homing current, determines how easy it is to stop the motor and thereby provoke a stall
     *
     * @return 0 on success,
      -1 on communication error,
      -3 when the motor is not enabled,
      -5 if the joint is not initialized,
      -101 if the velocity is zero,
      -102 if absolute value of the velocity is outside the specified limits.
   */
    err_type_t _home(float velocity, u_int8_t sensitivity, u_int8_t current);

    /**
     * @brief Check if communication to the joint is established
     *
     * Sends a PING to and expects a ACK from the joint.
     *
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
    */
    err_type_t checkCom(void);

    // /**
    //  * @brief State flags transmitted with every I2C transaction.
    //  *
    //  * The transmission flags purpose are to transmit the joints current state.
    //  * Note: They can not be used as error indication of the execution of a transmitted write command,
    //  * since commands are executed after the I2C transaction is completed. The status flags are one
    //  * byte with following structure: \n
    //  *
    //  * |BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0|
    //  * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
    //  * |reserved|reserved|reserved|reserved|NOTENABLED|NOTHOMED|BUSY|STALL|
    //  *
    //  * \b STALL is set if a stall from the stall detection is sensed and the joint is stopped.
    //  * The flag is cleared when the joint is homed or the Stallguard enabled. \n
    //  * \b BUSY is set if the slave is busy processing a previous command. \n
    //  * \b NOTHOMED is cleared if the joint is homed. Movement is only allowed if this flag is clear \n
    //  * \b NOTENABLED is cleared if the joint is enabled after calling Joint::enable()
    //  */
    // u_int8_t flags = 0x00;

    float reduction = 1; ///< Joint to actuator reduction ratio
    float offset = 0;    ///< Joint position offset
    float min = 0;       ///< Joint lower limit
    float max = 0;       ///< Joint upper limit

    // stp_reg_t current_b_cmd = NONE; ///< Keeps track if a blocking command is being executed

  private:
    template <typename T>
    int read(const stp_reg_t reg, T &data, u_int8_t &flags);

    template <typename T>
    int write(const stp_reg_t reg, T data, u_int8_t &flags);

    int address;     ///< I2C adress
    int handle = -1; ///< I2C bus handle
  };
}
#include "bioscara_arm_hardware_driver/mJoint.hpp"

#endif