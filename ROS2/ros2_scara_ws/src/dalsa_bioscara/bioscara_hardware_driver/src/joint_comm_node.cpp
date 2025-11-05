#include <signal.h>
#include <unistd.h>
#include "bioscara_hardware_driver/mJoint.h"
#include <vector>
#include <cmath>

using namespace std;
using namespace bioscara_hardware_driver;

Joint J1("j1", 0x11, 35, -3.04647, 3.04647);
Joint J2("j2", 0x12, -2 * M_PI / 0.004, 0.338, 0.0);
Joint J3("j3", 0x13, 24, -2.62672, 2.62672);
Joint J4("j4", 0x14, 12, -3.01069, 3.01069);

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

  if (!J2.isHomed())
  {
    cout << "Homing J2...\n";
    J2.enable(20, 20);
    J2.setMaxAcceleration(0.001);
    J2.home(0.003, 50, 30);
  }
  J2.disable();
  cout << "Press Enter to Continue...";
  cin.ignore();

  if (!J1.isHomed())
  {
    cout << "Homing J1...\n";
    J1.enable(20, 20);
    J1.setMaxAcceleration(0.01);
    J1.home(-0.03, 30, 15);
  }
  J1.disable();
  cout << "Press Enter to Continue...";
  cin.ignore();

  if (!J4.isHomed())
  {
    cout << "Homing J4...\n";
    J4.enable(20, 20);
    J4.setMaxAcceleration(0.01);
    J4.home(0.1, 30, 10);
  }
  J4.disable();
  cout << "Press Enter to Continue...";
  cin.ignore();

  if (!J3.isHomed())
  {
    cout << "Homing J3...\n";
    J3.enable(20, 20);
    J3.setMaxAcceleration(0.01);
    J3.home(0.05, 25, 10);
  }
  J3.disable();

  return 0;

  // J1.enable(20, 20);
  // J2.enable(20, 20);
  // J3.enable(20, 20);
  // J4.enable(20, 20);

  // J1.enableStallguard(5);
  // J2.enableStallguard(5);
  // J3.enableStallguard(5);
  // J4.enableStallguard(5);

  // J1.setPosition(0.0);
  // // J2.setPosition(0.0);
  // J3.setPosition(0.0);
  // J4.setPosition(0.0);

  cout << "Press Enter to Continue...";
  cin.ignore();

  vector<float> q = {0.0, 0.0, 0.0, 0.0};
  vector<float> qd = {0.0, 0.0, 0.0, 0.0};
  vector<float> q_set = {0.0, 0.0, 0.0, 0.0};
  vector<float> qd_set = {0.0, 0.0, 0.0, 0.0};
  float t = 0;
  int period_ms = 100;

  J1.disable();
  J2.disable();
  J3.disable();
  J4.disable();

  while (1)
  {

    if (J1.getPosition(q[0]) == err_type_t::OK &&
        J2.getPosition(q[1]) == err_type_t::OK &&
        J3.getPosition(q[2]) == err_type_t::OK &&
        J4.getPosition(q[3]) == err_type_t::OK)
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
