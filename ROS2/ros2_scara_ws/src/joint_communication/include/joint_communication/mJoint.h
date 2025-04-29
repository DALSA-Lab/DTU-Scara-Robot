#ifndef MJOINT_H
#define MJOINT_H

#define JOINT2ENCODERANGLE(jointAngle, gearRatio, offset) (gearRatio * (jointAngle + offset))
#define ENCODER2JOINTANGLE(encoderAngle, gearRatio, offset) (encoderAngle / gearRatio - offset)

class Joint
{
public:
  Joint(const int address, const std::string name, const float gearRatio, const int offset);
  // ~Joint();

  int init(void);
  int deinit(void);
  int printInfo(void);
  int getPosition(float &angle);
  int setPosition(float angle);
  int getVelocity(float &degps);
  int setVelocity(float degps);
  int checkOrientation(float angle = 10.0);

  /**
   * Initialize the driver
   * @param driveCurrent drive current in 0-100 % of 2.5A output (check uStepper doc.)
   * @param holdCurrent hold current in 0-100 % of 2.5A output (check uStepper doc.)
   * @return error code.
   */
  int enable(u_int8_t driveCurrent, u_int8_t holdCurrent);

  /**
   * disenganges the joint motor without closing i2c handle
   * @return error code.
   */
  int disable(void);

  /**
   * make motor move to end stop
   * @param direction  CCW: 0, CW: 1.
   * @param rpm  speed of motor in rpm > 10
   * @param sensitivity Encoder stalldetect sensitivity - From -100 to 10 where lower number is less sensitive and higher is more sensitive
   * @param current homeing current, determines how easy it is to stop the motor and thereby provoke a stall

   * @return error code.
   */
  int home(u_int8_t direction, u_int8_t rpm, int8_t sensitivity, u_int8_t current);

  /**
   * Stops the motor
   * @note When stopping the motor in soft mode, wait sufficiently long until the motor has stopped.
   * Since the stop() function in the motor controller is blocking.
   * Continously checking the busy flag also might interfere with the stop() function on the controller side.
   * @param mode Hard: 0, Soft: 1
   * @return error code.
   */
  int stop(bool mode);
  /**
   * Disables the Closed-Loop PID Controller
   * @return error code.
   */
  int disableCL(void);

  /**
   * Set the Drive Current
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setDriveCurrent(u_int8_t current);

  /**
   * Set the Hold Current
   * @param current 0% - 100% of driver current
   * @return error code.
   */
  int setHoldCurrent(u_int8_t current);

  /**
   * Set Brake Mode
   * @param mode Freewheel: 0, Coolbrake: 1, Hardbrake: 2
   * @return error code.
   */
  int setBrakeMode(u_int8_t mode);

  /**
   * checks if the motor is stalled
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
   * checks if the joint is homed from the joint
   * @param homed not homed: 0, homed: 1
   * @return error code.
   */
  int getIsHomed(u_int8_t &homed);

  /**
   * checks if the joint is homed from the joint
   * @return error code.
   */
  int getIsHomed(void);

  /**
   * @return the isHomed state variable.
   */
  bool isHomed(void);

  /**
   * checks if the joint is setup from the joint
   * @param setup not setup: 0, setup: 1
   * @return error code.
   */
  int getIsSetup(u_int8_t &setup);

  /**
   * checks if the joint is setup from the joint
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
  enum stp_reg_t
  {
    PING = 0x0f,
    SETUP = 0x10,
    SETRPM = 0x11,
    GETDRIVERRPM = 0x12,
    MOVESTEPS = 0x13,
    MOVEANGLE = 0x14,
    MOVETOANGLE = 0x15,
    GETMOTORSTATE = 0x16,
    RUNCOTINOUS = 0x17,
    ANGLEMOVED = 0x18,
    SETCURRENT = 0x19,
    SETHOLDCURRENT = 0x1A,
    SETMAXACCELERATION = 0x1B,
    SETMAXDECELERATION = 0x1C,
    SETMAXVELOCITY = 0x1D,
    ENABLESTALLGUARD = 0x1E,
    DISABLESTALLGUARD = 0x1F,
    CLEARSTALL = 0x20,
    ISSTALLED = 0x21,
    SETBRAKEMODE = 0x22,
    ENABLEPID = 0x23,
    DISABLEPID = 0x24,
    ENABLECLOSEDLOOP = 0x25,
    DISABLECLOSEDLOOP = 0x26,
    SETCONTROLTHRESHOLD = 0x27,
    MOVETOEND = 0x28,
    STOP = 0x29,
    GETPIDERROR = 0x2A,
    CHECKORIENTATION = 0x2B,
    GETENCODERRPM = 0x2C,
    HOME = 0x2D,
    ISHOMED = 0x2E,
    ISSETUP = 0x2F
  };

  template <typename T>
  int read(const stp_reg_t reg, T &data, u_int8_t &flags);

  template <typename T>
  int write(const stp_reg_t reg, T data, u_int8_t &flags);

  u_int8_t flags = 0x00;

  u_int8_t ishomed = 0;
  u_int8_t issetup = 0;

  int address;
  float gearRatio = 1;
  int offset = 0;

  int handle = -1;
};

#include "joint_communication/mJoint.hpp"

#endif