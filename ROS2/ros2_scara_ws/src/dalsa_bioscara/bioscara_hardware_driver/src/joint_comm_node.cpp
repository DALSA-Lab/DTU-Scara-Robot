#include <signal.h>
#include <unistd.h>
#include "bioscara_hardware_driver/mJoint.h"
#include <vector>
#include <cmath>

using namespace std;

Joint J1("j1", 0x11, 35, -3.04647);
Joint J2("j2", 0x12, -2 * M_PI / 0.004, 0.338);
Joint J3("j3", 0x13, 24, -2.62672);
Joint J4("j4", 0x14, 12, -3.01069);

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

  J1.init();
  J2.init();
  J3.init();
  J4.init();

  if (!J1.isHomed())
  {
    cout << "Homing J1...\n";
    J1.enable(20, 20);
    J1.home(0, 20, 30, 15);
  }
  J1.disable();
  cout << "Press Enter to Continue...";
  cin.ignore();

  if (!J2.isHomed())
  {
    cout << "Homing J2...\n";
    J2.enable(20, 20);
    J2.setMaxAcceleration(0.01);
    J2.home(0, 80, 50, 30);
  }
  J2.disable();
  cout << "Press Enter to Continue...";
  cin.ignore();

  if (!J4.isHomed())
  {
    cout << "Homing J4...\n";
    J4.enable(20, 20);

    J4.home(0, 10, 30, 10);
  }
  J4.disable();
  cout << "Press Enter to Continue...";
  cin.ignore();

  if (!J3.isHomed())
  {
    cout << "Homing J3...\n";
    J3.enable(20, 20);

    J3.home(0, 10, 30, 10);
  }
  J3.disable();

  return 0;

  cout << "Press Enter to Continue...";
  cin.ignore();

  // J2.disable();
  J2.enableStallguard(5);

  vector<float> q = {0.0};
  vector<float> qd = {0.0};
  vector<float> q_set = {0.0};
  vector<float> qd_set = {0.0};
  float t = 0;
  int period_ms = 100;

  q_set[0] = 0.1;
  J2.setMaxAcceleration(0.01);
  J2.setMaxVelocity(0.01);
  // if (J2.setPosition(q_set[0]) < 0)
  // {
  //   return -1;
  // }
  if (J2.setVelocity(DEG2RAD(10)) < 0)
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
        cout << n / 2 << '\t';
      }
      cout << endl;
    }
    else
    {
      break;
    }
    if (J2.isStalled())
    {
      J2.disable();
    }

    usleep(period_ms * 1000);
    t += period_ms * 1.0 / 1000;
  }
  return 0;
}
