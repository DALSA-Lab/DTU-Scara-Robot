#include <signal.h>
#include <unistd.h>
#include "joint_communication/mJointCom.h"
#include <cmath>
#include <lgpio.h>

using namespace std;

void INT_handler(int s)
{
  printf("Caught signal %d\n", s);
  exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGINT, INT_handler);

  (void)argc;
  (void)argv;

  Joint_comms Joints;

  // Joints.addJoint(0x11, "j1", 35, 0);
  // Joints.addJoint(0x12, "j2", -360 / 4, -50);
  Joints.addJoint(0x13, "j3", 24, +150);

  if (Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  if (Joints.setups(20, 20))
  {
    cerr << "Could not setup joints" << endl;
    return -1;
  }

  // if (Joints.setDriveCurrents(70))
  // {
  //   cerr << "Could not set drive currents of joints" << endl;
  //   return -1;
  // }

  // if (Joints.setHoldCurrents(70))
  // {
  //   cerr << "Could not set hold currents of joints" << endl;
  //   return -1;
  // }

  sleep(1);

  if (Joints.checkOrientations(1))
  {
    cerr << "Could not check orientation of joints" << endl;
    return -1;
  }


  Joints.home("j3", 0, 10, -2, 10);
  sleep(1);
  // Joints.home("j1", 0, 20, -3, 15);
  // sleep(1);
  // Joints.home("j2", 0, 20, -3, 30);

  // if (Joints.enableStallguards({20, 30, 10}))
  // {
  //   cerr << "Could not enable stallguards of joints" << endl;
  //   return -1;
  // }

  usleep(1000 * 1000);

  // Joints.deinit();
  // return 0;

  // vector<float> q = {0.0, 0.0, 0.0};
  // vector<float> qd = {0.0, 0.0, 0.0};
  // vector<float> q_set = {0.0, 50.0, 0.0};
  // vector<float> qd_set = {0.0, 0.0, 0.0};
  vector<float> q = {0.0};
  vector<float> qd = {0.0};
  vector<float> q_set = {0.0};
  vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 10;
  while (1)
  {
  
    // qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[2] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 10+10;

    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 360;

    // Joints.setVelocities(qd_set);
    if (Joints.setPositions(q_set) != 0)
    {
      break;
    }

    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;

    if (Joints.getPositions(q) == 0)
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
    if (Joints.getVelocities(qd) == 0)
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
  Joints.deinit();
  return 0;
}
