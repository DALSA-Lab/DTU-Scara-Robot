#include <signal.h>
#include <unistd.h>
#include "bioscara_hardware_driver/mJoint.h"
#include <vector>
#include <cmath>

using namespace std;

// Joint J2("j4", 0x14, 1 /*12*/, 0);
Joint J2("j2", 0x12, -2 * M_PI / 0.004, 0.388);

void INT_handler(int s)
{
  printf("Caught signal %d\n", s);
  exit(0);
}

int main(int argc, char **argv)
{
  signal(SIGINT, INT_handler);

  (void)argc;
  (void)argv;

  if (J2.init() < 0)
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  if (J2.enable(30, 30) < 0)
  {
    cerr << "Could not enable joint" << endl;
    return -1;
  }

  sleep(1);
  if (!J2.isHomed())
  {
    cout << "homing" << endl;
    J2.home(0, 20, 5, 30);
    // J2.home(0, 20, 50, 30);
  }

  // J2.disable();
  J2.enableStallguard(6);

  vector<float> q = {0.0};
  vector<float> qd = {0.0};
  vector<float> q_set = {0.0};
  vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 100;


  q_set[0] = 0.350;
  J2.setMaxAcceleration(0.001);
  if (J2.setPosition(q_set[0]) < 0)
  {
    return -1;
  }
  // J2.disable();

  while (1)
  {

    qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    // q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 90;
    // q_set[2] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 10+15;

    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 360;

    // if (J2.setVelocity(DEG2RAD(qd_set[0])) < 0)
    // {
    //   break;
    // }

    if (J2.getPosition(q[0]) == 0)
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

    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;
  }
  return 0;
}
