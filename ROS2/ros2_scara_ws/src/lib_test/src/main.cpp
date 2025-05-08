#include <signal.h>
#include <unistd.h>
#include "joint_communication/mJointCom.h"
#include "joint_communication/mGripper.h"

#include <cmath>

using namespace std;

Joint_comms _Joints;
Gripper _Gripper;

void INT_handler(int s)
{
  printf("Caught signal %d\n", s);
  _Joints.disables();
  _Gripper.disable();
  exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGINT, INT_handler);

  (void)argc;
  (void)argv;

    _Gripper.init();
  if(_Gripper.enable() != 0){
    cerr << "Gripper not enabled" << endl;
    return 0;
  }
  float time = 0;
  int period = 10;
  while (1)
  {

    if (_Gripper.setPosition((float)-cos(0.2 * 2 * M_PI * time) * 85/2+85/2) != 0)
    {
      break;
    }

    usleep(period * 1000);
    time += period * 1.0 / 1000;
  }

  return 0;
}
