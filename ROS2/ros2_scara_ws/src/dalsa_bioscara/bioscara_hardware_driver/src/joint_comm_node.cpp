#include <signal.h>
#include <unistd.h>
#include "bioscara_hardware_driver/mJointCom.h"
#include "bioscara_hardware_driver/mGripper.h"

#include <cmath>

using namespace std;

Joint_comms _Joints;
Gripper _Gripper;

void INT_handler(int s)
{
  printf("Caught signal %d\n", s);
  _Joints.disables();
  _Gripper.disable();
  exit(0);
}

int main(int argc, char **argv)
{
  signal(SIGINT, INT_handler);

  (void)argc;
  (void)argv;

  _Joints.addJoint("j4", 0x14, 12, 0); // 345 / 2);

  if (_Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }
  _Joints.enable("j4", 20, 20);
  sleep(1);
  if (!_Joints.joints.at("j4").isHomed())
  {
    std::cout << _Joints.home("j4", 0, 10, 30, 10);
  }

  _Joints.enableStallguard("j4", 20);

  vector<float> q = {0.0};
  vector<float> qd = {0.0};
  vector<float> q_set = {0.0};
  vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 10;
  q_set[0] = 10000;
      if (_Joints.setPosition("j4", q_set[0]) < 0)
    {
      // break;
    }
  while (1)
  {

    // qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    // q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[2] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 10+15;

    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 360;

    // _Joints.setVelocities(qd_set);


    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;

    if (_Joints.getPosition("j4", q[0]) == 0)
    {
      cout << "Positions: ";
      for (float n : q)
      {
        cout << n << '\t';
      }
      cout << endl;
    }
    else
    {
      break;
    }
  }
  _Joints.disables();
  return 0;
}
