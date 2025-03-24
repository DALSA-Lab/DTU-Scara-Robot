#include <signal.h>
#include <unistd.h>
#include "serial_test/mJointCom.h"
#include <cmath>

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

  u_int8_t addresses[] = {0xa1}; // Create an array of u_int8_t
  std::string names[] = {"j1"};  // Create an array of std::string

  Joint_comms Joints(1, addresses, names);
  if (Joints.init("/dev/ttyS0", 115200))
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  if (Joints.checkOrientations(500))
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
    qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    // cout << qd_set[0] << endl;
    // cout << t << endl;

    Joints.setVelocities(qd_set);

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
    }
    if (Joints.getVelocities(qd) == 0)
    {
      cout << "Velocities: ";
      for (float n : qd)
      {
        cout << n << ' ' << endl;
      }
    }
    if (t > 2)
    {
      Joints.deinit();
      break;
    }
  }

  return 0;
}
