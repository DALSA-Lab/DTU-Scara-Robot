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

  // int i2chandle = lgI2cOpen(1,0x10,0);
  // char txBuf[] = {19,12,23,33};
  // // char rxBuf[4] = {0};
  // cout << lgI2cReadI2CBlockData(i2chandle,0x0f,txBuf,1) << endl;
  // // float f = 10.0;
  // // memcpy(txBuf, &f, sizeof(float));
  // // memcpy(&f,txBuf, sizeof(float));
  // // cout << lguErrorText(lgI2cWriteI2CBlockData(i2chandle,0x2b,txBuf,sizeof(float))) << endl;
  // cout << txBuf << endl;
  // // cout << f << endl;

  // lgI2cClose(i2chandle);
  // return 0;

  int addresses[] = {0x10, 0x11};     // Create an array of u_int8_t
  std::string names[] = {"j1", "j2"}; // Create an array of std::string

  Joint_comms Joints(2, addresses, names);
  if (Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  if (Joints.setups(30, 30))
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

  if (Joints.checkOrientations(10))
  {
    cerr << "Could not check orientation of joints" << endl;
    return -1;
  }

  Joints.home("j2", 1);

  if (Joints.enableStallguards({20, 20}))
  {
    cerr << "Could not enable stallguards of joints" << endl;
    return -1;
  }

  usleep(1000 * 1000);

  vector<float> q = {0.0, 0.0};
  vector<float> qd = {0.0, 0.0};
  vector<float> q_set = {0.0, 0.0};
  vector<float> qd_set = {0.0, 0.0};
  float t = 0;
  int period_ms = 10;
  while (1)
  {
    // qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 360;
    q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 360;
    // cout << qd_set[0] << endl;
    // cout << t << endl;

    // Joints.setVelocities(qd_set);
    if (Joints.setPositions(q_set) != 0)
    {
      break;
    }

    // uint8_t stalled;
    // Joints.joints[0].getStall(stalled);
    // if(stalled){
    //   cout << "STALLED" << endl;
    //   break;
    // }

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
