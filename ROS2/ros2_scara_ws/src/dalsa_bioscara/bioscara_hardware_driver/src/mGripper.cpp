#include "bioscara_hardware_driver/mGripper.h"
#include "bioscara_hardware_driver/uTransmission.h"


Gripper::Gripper(float reduction, float offset, float min, float max) : BaseGripper()
{    
    this->reduction = reduction;
    this->offset = offset;
    this->min = min;
    this->max = max;
}

int Gripper::enable(void){
    return this->pwm.start(0, 50, 0, 0) < 0 ? -1 : 0;
}

int Gripper::disable(void){
    this->pwm.stop();
    return 0;
}

int Gripper::setPosition(float width){
    width = width < this->min ? this->min : width;
    width = width > this->max ? this->max : width;
    
    float dc = JOINT2ACTUATOR(width, this->reduction, this->offset);
    for (size_t i = 0; i < 3; i++)
    {
        if(this->pwm.setDutyCycle(dc) >= 0){
            return 0;
        }
    }
    
    return -1;
}

