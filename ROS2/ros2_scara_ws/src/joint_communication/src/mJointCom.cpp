#include "joint_communication/uI2C.h"
#include "joint_communication/mJointCom.h"

Joint_comms::Joint_comms(size_t n, const int addresses[], std::string names[])
{
    for (size_t i = 0; i < n; i++)
    {
        this->joints.push_back(Joint(addresses[i], names[i]));
    }
}

Joint_comms::~Joint_comms()
{
}

int Joint_comms::init(void)
{
    // Init each joint and test connection to each joint by pinging
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        if (this->joints[i].init() < 0)
        {
            std::cerr << "Failed to connect to: " << this->joints[i].name << std::endl;
            return -1;
        }
    }

    std::cout << "Joint Initialization successfull" << std::endl;

    return 0;
}

int Joint_comms::deinit()
{

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        if (this->joints[i].deinit() < 0)
        {
            std::cerr << "Failed to deinit: " << this->joints[i].name << std::endl;
            return -1;
        }
    }

    std::cout << "Joint Deinitialization successfull" << std::endl;

    return 0;
}

int Joint_comms::setups(std::vector<u_int8_t> driveCurrent_v, std::vector<u_int8_t> holdCurrent_v)
{
    if (driveCurrent_v.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    if (holdCurrent_v.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setup(driveCurrent_v[i], holdCurrent_v[i]);
        if (err != 0)
        {
            std::cerr << "Failed to setup: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::setups(u_int8_t driveCurrent, u_int8_t holdCurrent)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setup(driveCurrent, holdCurrent);
        if (err <= 0)
        {
            std::cerr << "Failed to setup: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::home(std::string name, u_int8_t direction)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        if(this->joints[i].name == name){
            int err = this->joints[i].home(direction);
            return err;
        }
    }
    std::cerr << "No joint with the name '" << name << "' initialized" << std::endl;
    return -1;
}

int Joint_comms::getPositions(std::vector<float> &angle_v)
{
    if (angle_v.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        float a;
        if (this->joints[i].getPosition(a) < 0)
        {
            std::cerr << "Failed to get angle from: " << this->joints[i].name << std::endl;
            return -1;
        }
        angle_v[i] = a;
    }
    return 0;
}

int Joint_comms::setPositions(std::vector<float> angle_v)
{
    if (angle_v.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setPosition(angle_v[i]);
        if (err != 0)
        {
            std::cerr << "Failed to set angle for: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::getVelocities(std::vector<float> &degps_v)
{
    if (degps_v.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        float a;
        if (this->joints[i].getVelocity(a) < 0)
        {
            std::cerr << "Failed to get speed from: " << this->joints[i].name << std::endl;
            return -1;
        }
        degps_v[i] = a;
    }
    return 0;
}

int Joint_comms::setVelocities(std::vector<float> degps_v)
{
    if (degps_v.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setVelocity(degps_v[i]);
        if (err < 0)
        {
            std::cerr << "Failed to set speed for: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::checkOrientations(std::vector<float> angle_v)
{
    if (angle_v.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].checkOrientation(angle_v[i]);
        if (err < 0)
        {
            std::cerr << "Failed to check orientation for: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::checkOrientations(float angle)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].checkOrientation(angle);
        if (err < 0)
        {
            std::cerr << "Failed to check orientation for: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::stops(bool mode)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].stop(mode);
        if (err < 0)
        {
            std::cerr << "Failed to stop motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::disableCLs(void)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].disableCL();
        if (err < 0)
        {
            std::cerr << "Failed to disable closed loop for motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::setDriveCurrents(std::vector<u_int8_t> current)
{
    if (current.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setDriveCurrent(current[i]);
        if (err < 0)
        {
            std::cerr << "Failed to set drive current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::setDriveCurrents(u_int8_t current)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setDriveCurrent(current);
        if (err < 0)
        {
            std::cerr << "Failed to set drive current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::setHoldCurrents(std::vector<u_int8_t> current)
{
    if (current.size() != this->joints.size())
    {
        std::cerr << "vector size mismatch" << std::endl;
        return -2;
    }

    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setHoldCurrent(current[i]);
        if (err < 0)
        {
            std::cerr << "Failed to set hold current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::setHoldCurrents(u_int8_t current)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setHoldCurrent(current);
        if (err < 0)
        {
            std::cerr << "Failed to set hold current for motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::setBrakeModes(u_int8_t mode)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].setBrakeMode(mode);
        if (err < 0)
        {
            std::cerr << "Failed to set brake mode for motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}

int Joint_comms::enableStallguards(std::vector<int8_t> thresholds)
{
    for (size_t i = 0; i < this->joints.size(); i++)
    {
        int err = this->joints[i].enableStallguard(thresholds[i]);
        if (err < 0)
        {
            std::cerr << "Failed to enable stallguard for motor: " << this->joints[i].name << " - error: " << err << std::endl;
            return err;
        }
    }
    return 0;
}