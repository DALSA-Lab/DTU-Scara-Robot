#include <signal.h>
#include <unistd.h>
#include <iostream>
#include "bioscara_hardware_driver/mGripper.h"

using namespace std;

Gripper _Gripper;

void INT_handler(int s)
{
  printf("Caught signal %d\n", s);
  _Gripper.disable();
  _Gripper.deinit();
  exit(1);
}

int main(int argc, char **argv)
{
  signal(SIGINT, INT_handler);

  (void)argc;
  (void)argv;

  _Gripper.init();
  if (_Gripper.enable() != 0)
  {
    cerr << "Gripper not enabled" << endl;
    return 0;
  }

  cout << "Use this program to manually set a gripper position by directly controlling the actuator angle.\n \
  This can be used to position the actuator for mounting or to determine reduction and offset.";
  while (1)
  {
    cout << "enter an angle for the gripper actuator in degrees between -90 and +90: ";
    float i;
    cin >> i;
    if (_Gripper.setServoPosition(i) != 0)
    {
      return -1;
    }
  }

  return 0;
}
