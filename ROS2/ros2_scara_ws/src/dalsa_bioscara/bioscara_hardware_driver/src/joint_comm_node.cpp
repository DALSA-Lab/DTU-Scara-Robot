#include <signal.h>
#include <unistd.h>
#include "bioscara_hardware_driver/mJointCom.h"
#include "bioscara_hardware_driver/mGripper.h"

#include <cmath>

using namespace std;

Joint_comms _Joints;

void INT_handler(int s)
{
  printf("Caught signal %d\n", s);
  _Joints.disables();
  exit(0);
}

int main(int argc, char **argv)
{
  signal(SIGINT, INT_handler);

  (void)argc;
  (void)argv;

  _Joints.addJoint("j4", 0x14, 12, 0);//345 / 2);

  if (_Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }
  _Joints.enable("j4", 20, 20);


  sleep(1);

  std::cout << _Joints.joints.at("j4").isHomed() << std::endl;
  if (!_Joints.joints.at("j4").isHomed())
  {
    _Joints.home("j4", 0, 10, 30, 10);
  }

  sleep(1);
  // // return 0;

  _Joints.enableStallguard("j4", 20);
  // usleep(1000 * 1000);

  // vector<float> q = {0.0, 0.0, 0.0, 0.0};
  // vector<float> qd = {0.0, 0.0, 0.0, 0.0};
  // vector<float> q_set = {0.0, 50.0, 0.0, 0.0};
  // vector<float> qd_set = {0.0, 0.0, 0.0, 0.0};
  vector<float> q = {0.0};
  vector<float> qd = {0.0};
  vector<float> q_set = {0.0};
  vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 10;
  while (1)
  {

    q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 10;

    if (_Joints.setPosition("j4", q_set[0]) != 0)
    {
      break;
    }

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
