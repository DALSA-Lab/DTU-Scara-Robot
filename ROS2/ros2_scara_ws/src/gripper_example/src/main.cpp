#include <signal.h>
#include <unistd.h>
#include "joint_communication/mGripper.h"

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

  while (1)
  {
    cout << "enter a gripper position between 30 mm and 85 mm: ";
    int i;
    cin >> i;
    if (_Gripper.setPosition(i * 1.0) != 0)
    {
      return -1;
    }
  }

  return 0;
}
