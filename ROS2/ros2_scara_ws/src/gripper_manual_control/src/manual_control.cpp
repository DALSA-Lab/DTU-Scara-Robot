#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <limits>
#include "bioscara_gripper_hardware_driver/mGripper.h"
#include "manual_control.h"

using namespace std;
using namespace bioscara_hardware_drivers;

Gripper _Gripper(1, 0, 0, 100);

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
  if (_Gripper.enable() != err_type_t::OK)
  {
    cerr << "Gripper not enabled" << endl;
    return 0;
  }

  while (1)
  {
    cout << "Control the gripper by width (w) or actuator angle (a) [w/a]: ";
    char mode;
    cin >> mode;
    cin.clear();
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (mode == 'w' || mode == 'W')
    {
      read_reduction();
      read_offset();

      while (1)
      {
        read_width();
      }
    }
    else if (mode == 'a' || mode == 'A')
    {
      while (1)
      {
        read_angle();
      }
    }
    else
    {
      cout << "invalid input: '" << mode << "'" << endl;
    }
  }

  return 0;
}

void read_angle()
{
  cout << "enter the actuator angle in degrees (-180 <= x <= +180): ";
  float i;
  cin >> i;
  if (!cin)
  {
    cout << "invalid input!" << endl;
    cin.clear();
  }
  else
  {
    if (_Gripper.setServoPosition(i) != err_type_t::OK)
    {
      cerr << "Failed to set position!" << endl;
    }
  }
  cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void read_width()
{
  cout << "Enter the gripper width in mm: ";
  float i;
  cin >> i;
  if (!cin)
  {
    cout << "invalid input!" << endl;
    cin.clear();
  }
  else
  {
    if (_Gripper.setPosition(i / 1000.0) != err_type_t::OK)
    {
      cerr << "Failed to set position!" << endl;
    }
  }
  cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void read_offset()
{
  while (1)
  {
    cout << "Enter the offset: ";
    float i;
    cin >> i;
    if (!cin)
    {
      cout << "invalid input!" << endl;
      cin.clear();
    }
    else
    {
      _Gripper.setOffset(i);
      break;
    }
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
}

void read_reduction()
{
  while (1)
  {
    cout << "Enter the reduction: ";
    float i;
    cin >> i;
    if (!cin)
    {
      cout << "invalid input!" << endl;
      cin.clear();
    }
    else
    {
      _Gripper.setReduction(i);
      break;
    }
    cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
}
