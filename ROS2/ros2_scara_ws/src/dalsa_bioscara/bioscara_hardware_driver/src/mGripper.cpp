#include "bioscara_hardware_driver/mGripper.h"

Gripper::Gripper(void)
{
    
}

int Gripper::init(void){
    return 0;
}

int Gripper::deinit(void){
    return 0;
}

int Gripper::enable(void){
    return this->pwm.start(0, 50, 0, 0) < 0 ? -1 : 0;
}

int Gripper::disable(void){
    this->pwm.stop();
    return 0;
}

int Gripper::setPosition(float width){
    width = width < 30 ? 30 : width;
    width = width > 85 ? 85 : width;
    float dc = (10-2.2)*width/85.0+2.2;
    for (size_t i = 0; i < 3; i++)
    {
        if(this->pwm.setDutyCycle(dc) >= 0){
            return 0;
        }
    }
    
    return -1;
}


