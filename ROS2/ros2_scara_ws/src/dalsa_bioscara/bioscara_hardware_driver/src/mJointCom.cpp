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

void Joint_comms::addJoint(const std::string name, const int address, const float gearRatio, const float offset)
{
    this->joints.insert({name,Joint(address, name, gearRatio, offset)});
}

void Joint_comms::removeJoint(const std::string name)
{
    this->joints.erase(name);
}

void Joint_comms::removeJoints(void)
{
    this->joints.clear();
}

// int Joint_comms::enables(std::vector<u_int8_t> driveCurrent_v, std::vector<u_int8_t> holdCurrent_v)
// {
//     if (driveCurrent_v.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }

//     if (holdCurrent_v.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }

//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].enable(driveCurrent_v[i], holdCurrent_v[i]);
//         if (err != 0)
//         {
//             std::cerr << "Failed to enable: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

// int Joint_comms::enables(u_int8_t driveCurrent, u_int8_t holdCurrent)
// {
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].enable(driveCurrent, holdCurrent);
//         if (err != 0)
//         {
//             std::cerr << "Failed to enable: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

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

    it->second.getIsHomed();

    std::cout << "Homed joint " << name << std::endl;

    return 0;
}

// int Joint_comms::getPositions(std::vector<float> &angle_v)
// {
//     if (angle_v.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         float a;
//         if (this->joints[i].getPosition(a) < 0)
//         {
//             std::cerr << "Failed to get angle from: " << this->joints[i].name << std::endl;
//             return -1;
//         }
//         angle_v[i] = a;
//     }
//     return 0;
// }

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

// int Joint_comms::setPositions(std::vector<float> angle_v)
// {
//     if (angle_v.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].setPosition(angle_v[i]);
//         if (err != 0)
//         {
//             std::cerr << "Failed to set angle for: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

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
        std::cerr << "Failed to set posistion for joint: " << name << std::endl;
        return -1;
    }

    return 0;
}

// int Joint_comms::getVelocities(std::vector<float> &degps_v)
// {
//     if (degps_v.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }
//
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         float a;
//         if (this->joints[i].getVelocity(a) < 0)
//         {
//             std::cerr << "Failed to get speed from: " << this->joints[i].name << std::endl;
//             return -1;
//         }
//         degps_v[i] = a;
//     }
//     return 0;
// }

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

// int Joint_comms::setVelocities(std::vector<float> degps_v)
// {
//     if (degps_v.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }
//
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].setVelocity(degps_v[i]);
//         if (err < 0)
//         {
//             std::cerr << "Failed to set speed for: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

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

// int Joint_comms::checkOrientations(std::vector<float> angle_v)
// {
//     if (angle_v.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }

//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].checkOrientation(angle_v[i]);
//         if (err < 0)
//         {
//             std::cerr << "Failed to check orientation for: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     sleep(1);
//     return 0;
// }

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

int Joint_comms::stops(bool mode)
{
    for (auto &[name, joint] : this->joints)
    {
        int err = joint.stop(mode);
        if (err != 0)
        {
            std::cerr << "Failed to stop motor: " << name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

// int Joint_comms::disableCLs(void)
// {
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].disableCL();
//         if (err < 0)
//         {
//             std::cerr << "Failed to disable closed loop for motor: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

// int Joint_comms::setDriveCurrents(std::vector<u_int8_t> current)
// {
//     if (current.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }

//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].setDriveCurrent(current[i]);
//         if (err < 0)
//         {
//             std::cerr << "Failed to set drive current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

// int Joint_comms::setDriveCurrents(u_int8_t current)
// {
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].setDriveCurrent(current);
//         if (err < 0)
//         {
//             std::cerr << "Failed to set drive current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

// int Joint_comms::setHoldCurrents(std::vector<u_int8_t> current)
// {
//     if (current.size() != this->joints.size())
//     {
//         std::cerr << "vector size mismatch" << std::endl;
//         return -2;
//     }

//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].setHoldCurrent(current[i]);
//         if (err < 0)
//         {
//             std::cerr << "Failed to set hold current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

// int Joint_comms::setHoldCurrents(u_int8_t current)
// {
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].setHoldCurrent(current);
//         if (err < 0)
//         {
//             std::cerr << "Failed to set hold current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

// int Joint_comms::setBrakeModes(u_int8_t mode)
// {
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].setBrakeMode(mode);
//         if (err < 0)
//         {
//             std::cerr << "Failed to set brake mode for motor: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

// int Joint_comms::enableStallguards(std::vector<u_int8_t> thresholds)
// {
//     for (size_t i = 0; i < this->joints.size(); i++)
//     {
//         int err = this->joints[i].enableStallguard(thresholds[i]);
//         if (err < 0)
//         {
//             std::cerr << "Failed to enable stallguard for motor: " << this->joints[i].name << " - error: " << err << std::endl;
//             return err;
//         }
//     }
//     return 0;
// }

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