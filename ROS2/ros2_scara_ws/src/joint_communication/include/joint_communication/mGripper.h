#ifndef MGRIPPER_H
#define MGRIPPER_H
#include "joint_communication/uPWM.h"

class Gripper
{
public:
    Gripper(void);

    int init(void);
    int deinit(void);
    int enable(void);
    int disable(void);
    /**
     * Sets the gripper position/width in mm from the closed position
     * @param width width in mm
     */
    int setPosition(float width);

private:
    RPI_PWM pwm;
};
#endif // MGRIPPER_H