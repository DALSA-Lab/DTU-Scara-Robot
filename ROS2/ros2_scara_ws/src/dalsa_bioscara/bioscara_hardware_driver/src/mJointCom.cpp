#include "bioscara_hardware_driver/uI2C.h"
#include "bioscara_hardware_driver/mJointCom.h"

Joint_comms::Joint_comms(void)
{
}

Joint_comms::~Joint_comms()
{
}

int Joint_comms::init(void)
{
    // Init each joint and test connection to each joint by pinging
    for (auto &[name, joint] : this->joints)
    {
        if (joint.init() < 0)
        {
            std::cerr << "Failed to connect to: " << name << std::endl;
            return -1;
        }
    }
    std::cout << "Joint Initialization successfull" << std::endl;

    return 0;
}

int Joint_comms::deinit()
{

    for (auto &[name, joint] : this->joints)
    {
        if (joint.deinit() < 0)
        {
            std::cerr << "Failed to deinit: " << name << std::endl;
            return -1;
        }
    }

    std::cout << "Joint Deinitialization successfull" << std::endl;

    return 0;
}

void Joint_comms::addJoint(const std::string name, const int address, const float reduction, const float offset)
{
    this->joints.insert({name, Joint(name, address, reduction, offset)});
}

void Joint_comms::removeJoint(const std::string name)
{
    this->joints.erase(name);
}

void Joint_comms::removeJoints(void)
{
    this->joints.clear();
}

int Joint_comms::enable(const std::string name, const u_int8_t driveCurrent, const u_int8_t holdCurrent)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }

    if (it->second.enable(driveCurrent, holdCurrent) < 0)
    {
        std::cerr << "Failed to enable: " << name << std::endl;
        return -1;
    }

    std::cout << "Enabled joint " << name << std::endl;

    return 0;
}

int Joint_comms::disables(void)
{
    for (auto &[name, joint] : this->joints)
    {
        int err = joint.disable();
        if (err != 0)
        {
            std::cerr << "Failed to disable: " << name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::home(const std::string name, const u_int8_t direction, const u_int8_t rpm, const u_int8_t sensitivity, const u_int8_t current)
{
    if (checkOrientation(name, 1.0) < 0)
    {
        return -1;
    }

    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }

    if (it->second.home(direction, rpm, sensitivity, current) < 0)
    {
        std::cerr << "Failed to home: " << name << std::endl;
        return -1;
    }

    std::cout << "Homed joint " << name << std::endl;

    return 0;
}

int Joint_comms::getPosition(const std::string name, float &angle)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }

    if (it->second.getPosition(angle) < 0)
    {
        std::cerr << "Failed to get posistion for joint: " << name << std::endl;
        return -1;
    }

    return 0;
}

int Joint_comms::setPosition(const std::string name, const float angle)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }

    if (it->second.setPosition(angle) < 0)
    {
        std::cerr << "Failed to set position for joint: " << name << std::endl;
        return -1;
    }

    return 0;
}

int Joint_comms::getVelocity(const std::string name, float &degps)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }

    if (it->second.getVelocity(degps) < 0)
    {
        std::cerr << "Failed to get velocity for joint: " << name << std::endl;
        return -1;
    }
    return 0;
}


int Joint_comms::setVelocity(const std::string name, float degps)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }

    if (it->second.setVelocity(degps) < 0)
    {
        std::cerr << "Failed to set velocity for joint: " << name << std::endl;
        return -1;
    }
    return 0;
}

int Joint_comms::checkOrientations(float angle)
{
    for (auto &[name, joint] : this->joints)
    {
        int err = joint.checkOrientation(angle);
        if (err != 0)
        {
            std::cerr << "Failed to check orientation of: " << name << " - error: " << err << std::endl;
            return err;
        }
    }
    sleep(1);
    return 0;
}

int Joint_comms::checkOrientation(const std::string name, float angle)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }

    if (it->second.checkOrientation(angle) < 0)
    {
        std::cerr << "Failed to check orientation of joint: " << name << std::endl;
        return -1;
    }
    return 0;
}

int Joint_comms::stops(void)
{
    for (auto &[name, joint] : this->joints)
    {
        int err = joint.stop();
        if (err != 0)
        {
            std::cerr << "Failed to stop motor: " << name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::enableStallguard(const std::string name, const u_int8_t threshold)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }
    if (it->second.enableStallguard(threshold) < 0)
    {
        std::cerr << "Failed to enable stallguard for motor: " << name << std::endl;
        return -1;
    }
    return 0;
}

int Joint_comms::setMaxAcceleration(const std::string name, float maxAccel)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }
    if (it->second.setMaxAcceleration(maxAccel) < 0)
    {
        std::cerr << "Failed to set max acceleration for motor: " << name << std::endl;
        return -1;
    }
    return 0;
}

int Joint_comms::setMaxVelocity(const std::string name, float maxVel)
{
    auto it = this->joints.find(name);
    if (it == this->joints.end())
    {
        std::cerr << "The joint: '" << name << "' does not exist! Add the joint using addJoint()." << std::endl;
        return -2;
    }
    if (it->second.setMaxVelocity(maxVel) < 0)
    {
        std::cerr << "Failed to set max velocity for motor: " << name << std::endl;
        return -1;
    }
    return 0;
}
