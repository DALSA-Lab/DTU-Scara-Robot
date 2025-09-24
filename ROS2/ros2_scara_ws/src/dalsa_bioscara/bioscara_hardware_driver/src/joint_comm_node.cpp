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

  _Gripper.init();
  if (_Gripper.enable() != 0)
  {
    cerr << "Gripper not enabled" << endl;
    return 0;
  }
  // // float time = 0;
  // // int period = 10;
  // // while (1)
  // // {
  // //   int i;
  // //   cin >> i;
  // //   if(_Gripper.setPosition(i*1.0) != 0){
  // //     break;
  // //   }
  // // if (_Gripper.setPosition((float)-cos(0.2 * 2 * M_PI * time) * 85/2+85/2) != 0)
  // // {
  // //   break;
  // // }

  // // usleep(period * 1000);
  // // time += period * 1.0 / 1000;
  // // }

  // // return -1;

  _Joints.addJoint("j1", 0x11, 35, 349.1 / 2);
  _Joints.addJoint("j2", 0x12, -360 / 4, -349.35);
  _Joints.addJoint("j3", 0x13, 24, 301 / 2);
  _Joints.addJoint("j4", 0x14, 12, 345 / 2);

  if (_Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  _Joints.enable("j1", 30, 30);
  _Joints.enable("j2", 40, 40);
  _Joints.enable("j3", 40, 40);
  _Joints.enable("j4", 20, 20);
  sleep(1);

  // if (_Joints.checkOrientations(1))
  // {
  //   cerr << "Could not check orientation of joints" << endl;
  //   return -1;
  // }
  // sleep(1);

  if (!_Joints.joints.at("j1").isHomed())
  {
    _Joints.home("j1", 0, 20, 30, 15);
  }
  // _Joints.joints.at("j1").disable();
  if (!_Joints.joints.at("j2").isHomed())
  {
    _Joints.home("j2", 0, 20, 50, 30);
  }
  // _Joints.joints.at("j2").disable();
  if (!_Joints.joints.at("j3").isHomed())
  {
    _Joints.home("j3", 0, 10, 30, 10);
  }
  // _Joints.joints.at("j3").disable();
  if (!_Joints.joints.at("j4").isHomed())
  {
    _Joints.home("j4", 0, 10, 30, 10);
  }
  // _Joints.joints.at("j4").disable();

  sleep(1);
  // // return 0;

  _Joints.enableStallguard("j1", 20);
  _Joints.enableStallguard("j2", 20);
  _Joints.enableStallguard("j3", 20);
  _Joints.enableStallguard("j4", 20);
  // usleep(1000 * 1000);

  _Joints.disables();
  // // return 0;

  vector<float> q = {0.0, 0.0, 0.0, 0.0};
  vector<float> qd = {0.0, 0.0, 0.0, 0.0};
  vector<float> q_set = {0.0, 50.0, 0.0, 0.0};
  vector<float> qd_set = {0.0, 0.0, 0.0, 0.0};
  // vector<float> q = {0.0};
  // vector<float> qd = {0.0};
  // vector<float> q_set = {0.0};
  // vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 10;
  while (1)
  {

    // qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[2] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 10+15;

    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 360;

    // _Joints.setVelocities(qd_set);
    // if (_Joints.setPositions(q_set) != 0)
    // {
    //   break;
    // }

    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;

    if (_Joints.getPosition("j1", q[0]) == 0 &&
        _Joints.getPosition("j2", q[1]) == 0 &&
        _Joints.getPosition("j3", q[2]) == 0 &&
        _Joints.getPosition("j4", q[3]) == 0)
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
  _Gripper.disable();
  _Joints.disables();
  return 0;
}
