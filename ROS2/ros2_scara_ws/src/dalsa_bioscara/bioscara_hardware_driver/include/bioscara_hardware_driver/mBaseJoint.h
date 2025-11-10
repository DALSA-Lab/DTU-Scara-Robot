/**
 * @file mBaseJoint.h
 * @author Sebastian Storz
 * @brief File including the BaseJoint class
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef MBASEJOINT_H
#define MBASEJOINT_H

#include <iostream>
#include "bioscara_hardware_driver/uErr.h"

namespace bioscara_hardware_driver
{
  /**
   * @brief TODO
   *
   */
  class BaseJoint
  {
  public:
    /**
     *
     * @brief register and command definitions
     *
     * a register can be read (R) or written (W), each register has a size in bytes.
     * The payload can be split into multiple values or just be a single value.
     * Note that not all functions are implemented.
     *
     */
    enum stp_reg_t
    {
      NONE = 0x00,                ///< Used for signalling purposes
      PING = 0x0f,                ///< R; Size: 1; [(char) ACK]
      SETUP = 0x10,               ///< W; Size: 2; [(uint8) holdCurrent, (uint8) driveCurrent]
      SETRPM = 0x11,              ///< W; Size: 4; [(float) RPM]
      GETDRIVERRPM = 0x12,        ///<
      MOVESTEPS = 0x13,           ///< W; Size: 4; [(int32) steps]
      MOVEANGLE = 0x14,           ///<
      MOVETOANGLE = 0x15,         ///< W; Size: 4; [(float) degrees]
      GETMOTORSTATE = 0x16,       ///<
      RUNCOTINOUS = 0x17,         ///<
      ANGLEMOVED = 0x18,          ///< R; Size: 4; [(float) degrees]
      SETCURRENT = 0x19,          ///< W; Size: 1; [(uint8) driveCurrent]
      SETHOLDCURRENT = 0x1A,      ///< W; Size: 1; [(uint8) holdCurrent]
      SETMAXACCELERATION = 0x1B,  ///<
      SETMAXDECELERATION = 0x1C,  ///<
      SETMAXVELOCITY = 0x1D,      ///<
      ENABLESTALLGUARD = 0x1E,    ///< W; Size: 1; [(uint8) threshold]
      DISABLESTALLGUARD = 0x1F,   ///<
      CLEARSTALL = 0x20,          ///<
      SETBRAKEMODE = 0x22,        ///< W; Size: 1; [(uint8) mode]
      ENABLEPID = 0x23,           ///<
      DISABLEPID = 0x24,          ///<
      ENABLECLOSEDLOOP = 0x25,    ///<
      DISABLECLOSEDLOOP = 0x26,   ///< W; Size: 1; [(uint8) 0]
      SETCONTROLTHRESHOLD = 0x27, ///<
      MOVETOEND = 0x28,           ///<
      STOP = 0x29,                ///< W; Size: 1; [(uint8) mode]
      GETPIDERROR = 0x2A,         ///<
      CHECKORIENTATION = 0x2B,    ///< W; Size: 4; [(float) degrees]
      GETENCODERRPM = 0x2C,       ///< R; Size: 4; [(float) RPM]
      HOME = 0x2D,                ///< W; Size: 4; [(uint8) current, (int8) sensitivity, (uint8) speed, (uint8) direction]
      HOMEOFFSET = 0x2E,          ///< R/W; Size: 4; [(float) -]
    };

    /**
     * @brief Create a Joint object
     *
     * The Joint object represents a single joint.
     *
     * @param name string device name for identification
     */
    BaseJoint(const std::string name);
    ~BaseJoint(void);

    /**
     * @brief Initialization, derived classes may override this.
     *
     */
    virtual err_type_t init(void);

    /**
     * @brief Deinitialization, derived classes may override this.
     *
     */
    virtual err_type_t deinit(void);

    /**
     * @brief Setup the joint and engages motor, derived classes may override this.
     *
     */
    virtual err_type_t enable(u_int8_t driveCurrent, u_int8_t holdCurrent);

    /**
     * @brief disenganges the joint motors, derived classes may override this.
     * @return 0 on success,
     * -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t disable(void);

    /**
     * @brief Blocking implementation to home the joint, derived classes may override this.
     *
     * A blocking implementation which only returns after the the joint is no longer BUSY. See Joint::_home() for documentation.
     *
     * Additionally this method returns:
     * @return -2 when not homed succesfull (isHomed flag still not set),
     * -109 if the joint is already currently homing (for example from a call to Joint::startHoming()).
     */
    virtual err_type_t home(float velocity, u_int8_t sensitivity, u_int8_t current);

    /**
     * @brief non-blocking implementation to home the joint, derived classes may override this.
     *
     * See Joint::_home() for documentation. The current_b_cmd flag is set to HOME
     * This method returns immediatly after starting the homing sequence. This should be used when the blocking implementation is not acceptable.
     * For example in the update loop of the bioscara_hardware_interfaces::BioscaraHardwareInterface::write().

     * Additionally this method returns:
     * @return -109 if the joint is already currently homing (for example from a call to Joint::startHoming()).
     */
    virtual err_type_t startHoming(float velocity, u_int8_t sensitivity, u_int8_t current);

    /**
     * @brief perform tasks after a non-blocking homing, derived classes may override this.
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
    virtual err_type_t postHoming(void);

    /**
     * @brief get the current joint position in radians or m for
     * cylindrical and prismatic joints respectively. Derived class must override this.
     *
     * @warning If the joint is not homed this method does not return an error.
     * Instead `pos` will be 0.0.
     *
     * @param pos
     * @return 0 on success,
      -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t getPosition(float &pos) = 0;

    /**
     * @brief get the current joint position in radians or m for
     * cylindrical and prismatic joints respectively. Derived class mayy override this.
     *
     * @param pos in rad or m
     * @return 0 on success,
      -1 on communication error,
      -2 when not homed,
      -3 when the motor is not enabled,
      -4 when the motor is stalled,
      -5 if the joint is not initialized.
     */
    virtual err_type_t setPosition(float pos);

    /**
     * @brief Move full steps. Derived class may override this.
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
    virtual err_type_t moveSteps(int32_t steps);

    /**
     * @brief get the current joint velocity in radians/s or m/s for
     * cylindrical and prismatic joints respectively. Derived class must override this.
     *
     * @param vel
     * @return 0 on success,
      -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t getVelocity(float &vel) = 0;

    /**
     * @brief Set the current joint velocity in radians/s or m/s for
     * cylindrical and prismatic joints respectively. Derived class may override this.
     *
     * @param vel
     * @return 0 on success,
      -1 on communication error,
      -2 when not homed,
      -3 when the motor is not enabled,
      -4 when the motor is stalled,
      -5 if the joint is not initialized.
     */
    virtual err_type_t setVelocity(float vel);

    /**
     * @brief Calls the checkOrientation method of the motor. Checks in which direction the motor is turning.
     * Derived class may override this.
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
    virtual err_type_t checkOrientation(float angle = 10.0);

    /**
     * @brief Stops the motor. Derived class may override this.
     *
     * Stops the motor by setting the maximum velocity to zero and the position setpoint
     * to the current position
     *
     * @return 0 on success,
     * -1 on communication error,
     * -5 if the joint is not initialized.
     */
    virtual err_type_t stop(void);

    /**
     * @brief Disables the Closed-Loop PID Controller Derived class may override this.
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t disableCL(void);

    /**
     * @brief Set the Drive Current. Derived class may override this.
     * @warning This function is unreliable and not well tested. Use Joint::enable() instead!
     * @param current 0% - 100% of driver current
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t setDriveCurrent(u_int8_t current);

    /**
     * @brief Set the Hold Current. Derived class may override this.
     * @warning This function is unreliable and not well tested. Use Joint::enable() instead!
     * @param current 0% - 100% of driver current
     * @return 0 on success,
     * -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t setHoldCurrent(u_int8_t current);

    /**
     * @brief Set Brake Mode. Derived class may override this.
     * @param mode Freewheel: 0, Coolbrake: 1, Hardbrake: 2
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t setBrakeMode(u_int8_t mode);

    /**
     * @brief Set the maximum permitted joint acceleration (and deceleration) in rad/s^2 or m/s^2 for cylindrical
     * and prismatic joints respectively. Derived class may override this.
     *
     * @param maxAccel maximum joint acceleration.
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t setMaxAcceleration(float maxAccel);

    /**
     * @brief Set the maximum permitted joint velocity in rad/s or m/s for cylindrical
     * and prismatic joints respectively. Derived class may override this.
     *
     * @param maxVel maximum joint velocity.
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t setMaxVelocity(float maxVel);

    /**
     * @brief Enable encoder stall detection of the joint. Derived class may override this.
     *
     * If the PID error exceeds the set threshold a stall is triggered and the motor disabled.
     * A detected stall can be reset by homing or by reenabling the stall guard.
     * @param sensitivity value of threshold. 0 - 255 where lower is more sensitive.
     * @return 0 on success, -1 on communication error,
      -5 if the joint is not initialized.
     */
    virtual err_type_t enableStallguard(u_int8_t sensitivity);

    /**
     * @brief Checks the state if the motor is homed.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
     *
     * @return true if the motor is homed,
     * false if not.
     */
    virtual bool isHomed(void);

    /**
     * @brief Checks the state if the motor is enabled.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
     * If the motor actually can move depends on the state of the STALLED flag which can be checked using Joint::isStalled().
     *
     * @return true if the motor is enabled,
     * false if not.
     */
    virtual bool isEnabled(void);

    /**
     * @brief Checks if the motor is stalled.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
     * @return true if the motor is stalled,
     * false if not.
     */
    virtual bool isStalled(void);

    /**
     * @brief Checks if the joint controller is busy processing a blocking command.
     *
     * Reads the internal state flags from the last transmission. If an update is neccessary call Joint::getFlags() before invoking this function.
     * @return true if a blocking command is currently executing,
     * false if not.
     */
    virtual bool isBusy(void);

    /**
     * ping the joint to get the latest driver state flags
     * @param flags if succesfull, populated with the latest flags
     * @return 0 on success,
      -5 if the joint is not initialized.
     */
    virtual err_type_t getFlags(u_int8_t &flags);

    /**
     * Overload of BaseJoint::getFlags(u_int8_t &flags)
     * @return 0 on success,
      -5 if the joint is not initialized.
     */
    virtual err_type_t getFlags(void);

    /**
     * @brief get the currently active blocking command
     *
     * @return The the command of type stp_reg_t
     */
    virtual stp_reg_t getCurrentBCmd(void);

    std::string name;

  protected:
    /**
     * @brief Blocking loop waiting for BUSY flag to reset.
     *
     * @param period_ms time in ms between polls.
     */
    virtual void wait_while_busy(const float period_ms);

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
    virtual err_type_t _home(float velocity, u_int8_t sensitivity, u_int8_t current) = 0;

    /**
     * @brief State flags transmitted with every I2C transaction.
     *
     * The transmission flags purpose are to transmit the joints current state.
     * Note: They can not be used as error indication of the execution of a transmitted write command,
     * since commands are executed after the I2C transaction is completed. The status flags are one
     * byte with following structure: \n
     *
     * |BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0|
     * | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
     * |reserved|reserved|reserved|reserved|NOTENABLED|NOTHOMED|BUSY|STALL|
     *
     * \b STALL is set if a stall from the stall detection is sensed and the joint is stopped.
     * The flag is cleared when the joint is homed or the Stallguard enabled. \n
     * \b BUSY is set if the slave is busy processing a previous command. \n
     * \b NOTHOMED is cleared if the joint is homed. Movement is only allowed if this flag is clear \n
     * \b NOTENABLED is cleared if the joint is enabled after calling Joint::enable()
     */
    u_int8_t flags = 0b00001100;

    stp_reg_t current_b_cmd = NONE; ///< Keeps track if a blocking command is being executed

  private:
  };
}
#endif