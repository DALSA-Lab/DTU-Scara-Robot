#include "bioscara_hardware_driver/mGripper.h"
#include "bioscara_hardware_driver/uTransmission.h"

Gripper::Gripper(float reduction, float offset, float min, float max) : BaseGripper()
{
    this->reduction = reduction;
    this->offset = offset;
    this->min = min;
    this->max = max;
}

int Gripper::enable()
{
    return this->pwm.start(0, this->freq, 0, 0) < 0 ? -1 : 0;
}

int Gripper::disable(void)
{
    this->pwm.stop();
    return 0;
}

int Gripper::setPosition(float width)
{
    width = width < this->min ? this->min : width;
    width = width > this->max ? this->max : width;

    float angle = JOINT2ACTUATOR(width, this->reduction, this->offset);
    return this->setServoPosition(angle);
}

int Gripper::setServoPosition(float angle)
{
    float ton_us = angle/90.0 * 500.0 + 1500.0;             // Ontime [us]
    float dc = ton_us / (1000*1000) * this->freq * 100;     // dutycycle [%] = ontime [s] /period [s] * 100 %
    for (size_t i = 0; i < 3; i++)
    {
        if (this->pwm.setDutyCycle(dc) >= 0)
        {
            return 0;
        }
    }

    return -1;
}