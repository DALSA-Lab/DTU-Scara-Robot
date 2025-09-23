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

#define JOINT2ENCODERANGLE(jointAngle, gearRatio, offset) (gearRatio * (jointAngle + offset))
#define ENCODER2JOINTANGLE(encoderAngle, gearRatio, offset) (encoderAngle / gearRatio - offset)

/**
 * @brief Representing a single joint on the I2C bus
 *
 */
class Joint
{
public:
  Joint(const int address, const std::string name, const float gearRatio, const float offset);
  // ~Joint();

  int init(void);
  int deinit(void);
  int printInfo(void);

  /**
   * @brief get the current joint position in degrees or mm for 
   * cylindrical and prismatic joints respectively.
   * 
   * @param angle 
   * @return error code 
   */
  int getPosition(float &angle);
  int setPosition(float angle);

    /**
   * @brief get the current joint velocity in degrees/s or mm/s for 
   * cylindrical and prismatic joints respectively.
   * 
   * @param angle 
   * @return error code 
   */
  int getVelocity(float &degps);
  int setVelocity(float degps);
  int checkOrientation(float angle = 10.0);

  /**
   * @brief Initialize the joint and engages motor.
   * @param driveCurrent drive current in 0-100 % of 2.5A output (check uStepper doc.)
   * @param holdCurrent hold current in 0-100 % of 2.5A output (check uStepper doc.)
   * @return error code.
   */
  int enable(u_int8_t driveCurrent, u_int8_t holdCurrent);

  /**
   * @brief disenganges the joint motor without closing i2c handle
   * @return error code.
   */
  int disable(void);

  /**
   * @brief Homes the motor.
   * @param direction  CCW: 0, CW: 1.
   * @param rpm  speed of motor in rpm > 10.
   * @param sensitivity Encoder pid error threshold 0 to 255.
   * @param current homeing current, determines how easy it is to stop the motor and thereby provoke a stall

   * @return error code.
   */
  int home(u_int8_t direction, u_int8_t rpm, u_int8_t sensitivity, u_int8_t current);

  /**
   * @brief Stops the motor.
   * @note When stopping the motor in soft mode, wait sufficiently long until the motor has stopped.
   * Since the stop() function in the motor controller is blocking.
   * Continously checking the busy flag also might interfere with the stop() function on the controller side.
   * @param mode Hard: 0, Soft: 1
   * @return error code.
   */
  int stop(bool mode);
  /**
   * @brief Disables the Closed-Loop PID Controller
   * @return error code.
   */
  int disableCL(void);

  /**
   * @brief Set the Drive Current
   * @warning This function is unreliable and not well tested. Use enable() instead!
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setDriveCurrent(u_int8_t current);

  /**
   * @brief Set the Hold Current
   * @warning This function is unreliable and not well tested. Use enable() instead!
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setHoldCurrent(u_int8_t current);

  /**
   * @brief Set Brake Mode
   * @param mode Freewheel: 0, Coolbrake: 1, Hardbrake: 2
   * @return error code.
   */
  int setBrakeMode(u_int8_t mode);

  /**
   * @brief checks if the motor is stalled
   * @param stall not stalled: 0, stalled: 1
   * @return error code.
   */
  int getStall(u_int8_t &stall);

  /**
   * @brief Enable encoder stall detection. A detected stall can be reset by homeing.
   * @param sensitivity Encoder stalldetect sensitivity - From -100 to 10 where lower number is less sensitive and higher is more sensitive
   */
  int enableStallguard(u_int8_t sensitivity);

  /**
   * @brief retrieves the status flags from the joint and checks if the joint is homed.
   * @param homed not homed: 0, homed: 1
   * @return error code.
   */
  int getIsHomed(u_int8_t &homed);

  /**
   * @brief retrieves the status flags from the joint.
   *
   * This overload does not return the value of isHomed variable, use Joint::isHomed() instead.
   * @return error code.
   */
  int getIsHomed(void);

  /**
   * @brief Get the isHomed state variable saved locally.
   *
   * To retrieve the actual state call Joint::getIsHomed()
   * @return local isHomed state variable.
   */
  bool isHomed(void);

  /**
   * @brief checks if the joint is setup from the joint
   * @param setup not setup: 0, setup: 1
   * @return error code.
   */
  int getIsSetup(u_int8_t &setup);

  /**
   * @brief checks if the joint is setup from the joint
   * @return error code.
   */
  int getIsSetup(void);

  /**
   * @return the isSetup state variable.
   */
  bool isSetup(void);

  int moveSteps(int32_t steps);
  int checkCom(void);

  /**
   * get driver state flags
   * @return flags.
   */
  u_int8_t getFlags(void);

  std::string name;

protected:
private:
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
    ISSTALLED = 0x21,           ///< R; Size: 1; [(uint8) isStalled]
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
    ISHOMED = 0x2E,             ///< R; Size: 1; [(uint8) isStalled]
    ISSETUP = 0x2F              ///< R; Size: 1; [(uint8) isStalled]
  };

  template <typename T>
  int read(const stp_reg_t reg, T &data, u_int8_t &flags);

  template <typename T>
  int write(const stp_reg_t reg, T data, u_int8_t &flags);

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
   * |reserved|reserved|reserved|reserved|SETUP|HOMED|BUSY|STALL|
   *
   * \b STALL is set if a stall from the stall detection is sensed and the joint is stopped.
   * The flag is cleared when the joint is homed. \n
   * \b BUSY is set if the slave is busy processing a previous command. \n
   * \b HOMED is set if the joint is homed. Movement is only allowed if this flag is clear \n
   * \b SETUP is set if the joint is setup after calling Joint::enable()
   */
  u_int8_t flags = 0x00;

  u_int8_t ishomed = 0; ///< flag if homed
  u_int8_t issetup = 0; ///< flag is setup

  int address;         ///< I2C adress
  float gearRatio = 1; ///< gear ratio from encoder units to joint units
  float offset = 0;      ///< offset in degrees or mm from encoder zero to joint zero.

  int handle = -1; ///< I2C bus handle
};

#include "bioscara_hardware_driver/mJoint.hpp"

#endif