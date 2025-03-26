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

  int i2chandle = lgI2cOpen(1,0x10,0);

  char txBuf[] = {19,12,23,33};
  // char rxBuf[4] = {0};
  cout << lgI2cReadI2CBlockData(i2chandle,0x18,txBuf,4) << endl;
  float f = 5.5;
  // memcpy(txBuf, &f, sizeof(float));
  memcpy(&f,txBuf, sizeof(float));
  // cout << lguErrorText(lgI2cWriteI2CBlockData(i2chandle,0x11,txBuf,sizeof(float))) << endl;
  cout << txBuf << endl;
  cout << f << endl;

  lgI2cClose(i2chandle);
  return 0; 

  int addresses[] = {0x10}; // Create an array of u_int8_t
  std::string names[] = {"j1"};  // Create an array of std::string

  Joint_comms Joints(1, addresses, names);
  if (Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  if (Joints.checkOrientations())
  {
    cerr << "Could not check orientation of joints" << endl;
    return -1;
  }

  vector<float> q = {0.0};
  vector<float> qd = {0.0};
  vector<float> q_set = {0.0};
  vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 10;
  while (1)
  {
    // qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 360;
    // cout << qd_set[0] << endl;
    // cout << t << endl;

    // Joints.setVelocities(qd_set);
    // Joints.setPositions(q_set);

    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;

    // Joints.joints[0].moveSteps(5000);
    // Joints.setPositions(q_set);
    // q_set[0]+=90;

    // usleep(1000 * 1000);
    if (Joints.getPositions(q) == 0)
    {
      cout << "Positions: ";
      for (float n : q)
      {
        cout << n << ' ' << endl;
      }
    }else{
      break;
    }
    if (Joints.getVelocities(qd) == 0)
    {
      cout << "Velocities: ";
      for (float n : qd)
      {
        cout << n << ' ' << endl;
      }
    }else{
      cout << "Velocities: ";
      break;
    }
    // if (t > 2)
    // {
    //   Joints.deinit();
    //   break;
    // }
  }

  return 0;
}
