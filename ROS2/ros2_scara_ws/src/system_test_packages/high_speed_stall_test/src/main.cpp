#include <signal.h>
#include "bioscara_hardware_driver/mJoint.h"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std;
using namespace bioscara_hardware_driver;

#define CURRENT 20
#define MAXACCEL 30
#define MAXVEL 10

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
    cout << "Not homed\n";
    RETURN_ON_NEGATIVE((int)J.enable(20, 20),-1);
    RETURN_ON_NEGATIVE((int)J.setMaxAcceleration(0.01),-2);
    RETURN_ON_NEGATIVE((int)J.home(0.1, 10, 10),-3);
  }

  RETURN_ON_NEGATIVE((int)J.setMaxAcceleration(MAXACCEL), -1);
  RETURN_ON_NEGATIVE((int)J.setMaxVelocity(MAXVEL), -1);
  RETURN_ON_NEGATIVE((int)J.enable(CURRENT, CURRENT), -1);
  RETURN_ON_NEGATIVE((int)J.enableStallguard(6), -1);


  float q = 0.0;
  float qd = 0.0;

  int period_ms = 10;
  
  while (qd < MAXVEL)
  {
    cout << "velocity: " << qd << endl;
    RETURN_ON_NEGATIVE((int)J.setVelocity(qd), -1);
    RETURN_ON_NEGATIVE((int)J.getVelocity(q), -1);
    qd += 1.0 * period_ms / 1000.0;
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
  while (qd > 0)
  {
    float v;
    cout << "velocity: " << qd << endl;
    RETURN_ON_NEGATIVE((int)J.setVelocity(qd), -1);
    RETURN_ON_NEGATIVE((int)J.getVelocity(v), -1);
    qd -= 1.0 * period_ms / 1000.0;
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
  return 0;
}
