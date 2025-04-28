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
   * @param mode Hard: 0, Soft: 1
   * @return error code.
   */
  int setup(u_int8_t driveCurrent, u_int8_t holdCurrent);

  /**
   * mke motor move to end stop
   * @param direction  CCW: 0, CW: 1.
   * @param rpm  speed of motor in rpm > 10
   * @param sensitivity Encoder stalldetect sensitivity - From -100 to 10 where lower number is less sensitive and higher is more sensitive
   * @param current homeing current, determines how easy it is to stop the motor and thereby provoke a stall

   * @return error code.
   */
  int home(u_int8_t direction, u_int8_t rpm, int8_t sensitivity, u_int8_t current);

  /**
   * Stops the motor
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
   * @brief Enable TMC5130 StallGuard
   * This function enables the builtin stallguard offered from TMC5130 stepper driver.
   * The threshold should be tuned as to trigger stallguard before a step is lost.
   * @param threshold stall sensitivity. A value between -64 and +63
   */
  int enableStallguard(int8_t threshold);

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
    ISHOMED = 0x2E
  };

  template <typename T>
  int read(const stp_reg_t reg, T &data, u_int8_t &flags);

  template <typename T>
  int write(const stp_reg_t reg, T data, u_int8_t &flags);

  u_int8_t flags = 0x00;

  u_int8_t homed = 0;

  int address;
  float gearRatio = 1;
  int offset = 0;

  int handle = -1;
};

#include "joint_communication/mJoint.hpp"

#endif