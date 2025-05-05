#include <signal.h>
#include <unistd.h>
#include "joint_communication/mJointCom.h"
#include "joint_communication/mGripper.h"

#include <cmath>
// #include <lgpio.h>

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
  // float time = 0;
  // int period = 10;
  // while (1)
  // {

  //   if (_Gripper.setPosition((float)-cos(0.2 * 2 * M_PI * time) * 85/2+85/2) != 0)
  //   {
  //     break;
  //   }

  //   usleep(period * 1000);
  //   time += period * 1.0 / 1000;
  // }

  // return 0;

  _Joints.addJoint(0x11, "j1", 35, 0);
  _Joints.addJoint(0x12, "j2", -360 / 4, -50);
  _Joints.addJoint(0x13, "j3", 24, +150);

  if (_Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  if (_Joints.enables({30, 40, 40}, {30, 40, 40}))
  {
    cerr << "did not enable joints" << endl;
    return -1;
  }

  sleep(1);

  if (_Joints.checkOrientations(1))
  {
    cerr << "Could not check orientation of joints" << endl;
    return -1;
  }

  sleep(1);

  if (!_Joints.joints[0].isHomed())
  {
    _Joints.home("j1", 0, 20, -3, 15);
  }
  if (!_Joints.joints[1].isHomed())
  {
    _Joints.home("j2", 0, 20, -3, 30);
  }
  if (!_Joints.joints[2].isHomed())
  {
    _Joints.home("j3", 0, 10, -2, 10);
  }

  sleep(1);
  // _Joints.home("j1", 0, 20, -3, 15);
  // sleep(1);
  // _Joints.home("j2", 0, 20, -3, 30);

  if (_Joints.enableStallguards({20, 20, 20}))
  {
    cerr << "Could not enable stallguards of joints" << endl;
    return -1;
  }

  usleep(1000 * 1000);

  // _Joints.disables();
  // return 0;

  vector<float> q = {0.0, 0.0, 0.0};
  vector<float> qd = {0.0, 0.0, 0.0};
  vector<float> q_set = {0.0, 50.0, 0.0};
  vector<float> qd_set = {0.0, 0.0, 0.0};
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
    if (_Joints.setPositions(q_set) != 0)
    {
      break;
    }

    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;

    if (_Joints.getPositions(q) == 0)
    {
      cout << "Positions: ";
      for (float n : q)
      {
        cout << n << ' ';
      }
      cout << endl;
    }
    else
    {
      break;
    }
    if (_Joints.getVelocities(qd) == 0)
    {
      cout << "Velocities: ";
      for (float n : qd)
      {
        cout << n << ' ';
      }
      cout << endl;
    }
    else
    {
      break;
    }
    // if (t > 0.5)
    // {
    //   break;
    // }
  }
  _Gripper.disable();
  _Joints.disables();
  return 0;
}
