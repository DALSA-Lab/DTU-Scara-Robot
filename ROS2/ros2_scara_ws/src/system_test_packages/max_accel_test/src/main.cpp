#include <signal.h>
#include "bioscara_arm_hardware_driver/mJoint.h"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std;
using namespace bioscara_hardware_drivers;

#define CURRENT 20
#define MAXACCEL 30
#define MAXVEL 10

// Joint J("j1", 0x11, 35, -3.04647, 3.04647);
// Joint J("j2", 0x12, -2 * M_PI / 0.004, 0.338, 0.0);
// Joint J("j3", 0x13, 24, -2.62672, 2.62672);
Joint J("j4", 0x14, 12, -3.01069, 3.01069);

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

  J.init();

  if (!J.isHomed())
  {
    cout << "Not homed, returning...\n";
    return 1;
  }
  J.disable();
  cout << "Move to center. Press Enter to Continue...";
  cin.ignore();
  cout << endl;

 

  RETURN_ON_NEGATIVE((int)J.setMaxAcceleration(MAXACCEL), -1);
  RETURN_ON_NEGATIVE((int)J.setMaxVelocity(MAXVEL), -1);
  RETURN_ON_NEGATIVE((int)J.enable(CURRENT, CURRENT), -1);

  cout << "-----------------------" << endl;

  cout << "joint: " << J.name.c_str() << endl;
  cout << "Vel: " << MAXVEL << endl;
  cout << "max Accel: " << MAXACCEL << endl;
  cout << "current: " << CURRENT << endl;

  cout << "-----------------------" << endl;

  float q = 0.0;
  float qd = 0.0;
  float qdd = 0.0;

  float t = 0;
  int period_ms = 10;

  cout << "t, q, qd, qdd" << endl;

  RETURN_ON_NEGATIVE((int)J.setVelocity(MAXVEL), -1);


  while (t<0.5)
  {
    auto t1 = std::chrono::steady_clock::now();    
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    auto t2 = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = t2 - t1;

    float period = elapsed.count();

    float last_qd = qd;
    J.getPosition(q);
    J.getVelocity(qd);
    qdd = (qd -last_qd)/period;
    t += period;

    cout << t << ", " << q << ", " << qd << ", " << qdd << endl;
  }
  return 0;
}
