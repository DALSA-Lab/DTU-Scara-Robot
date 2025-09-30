#include <signal.h>
#include <unistd.h>
#include "bioscara_hardware_driver/mJoint.h"
#include <vector>
#include <cmath>

using namespace std;

Joint J4("j4", 0x14, 1 /*12*/, 0);

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

  if (J4.init() < 0)
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }
  
  if (J4.enable(20, 20) < 0)
  {
    cerr << "Could not enable joint" << endl;
    return -1;
  }
  sleep(1);
  if (!J4.isHomed())
  {
    cout << "homing" << endl;
    std::cout << J4.home(0, 10, 30, 10);
  }

  J4.enableStallguard(6);

  vector<float> q = {0.0};
  vector<float> qd = {0.0};
  vector<float> q_set = {0.0};
  vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 100;
  q_set[0] = 90;

  while (1)
  {

    qd_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 1000;
    // q_set[0] = (float)sin(0.2 * 2 * M_PI * t) * 90;
    // q_set[2] = (float)sin(0.2 * 2 * M_PI * t) * 10;
    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 10+15;

    // q_set[1] = (float)sin(0.2 * 2 * M_PI * t) * 360;

    if (J4.setVelocity(DEG2RAD(qd_set[0])) < 0)
    {
      break;
    }
    // if (J4.setPosition(q_set[0]) < 0)
    // {
    //   break;
    // }

    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;

    if (J4.getPosition(q[0]) == 0)
    {
      // cout << "Positions: ";
      // for (float n : q)
      // {
      //   cout << RAD2DEG(n) << '\t';
      // }
      // cout << endl;
    }
    else
    {
      break;
    }
  }
  return 0;
}
