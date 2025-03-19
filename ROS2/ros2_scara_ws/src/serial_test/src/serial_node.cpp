#include <unistd.h>
#include "serial_test/mJointCom.h"

using namespace std;

int main(int argc, char **argv)
{
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

  vector<float> q = {0.0};
  vector<float> q_set = {0.0};
  while (1)
  {
    // Joints.joints[0].moveSteps(5000);
    Joints.setAngles(q_set);
    q_set[0]+=90;
    
    usleep(1000 * 1000);
    if (Joints.getAngles(q) == 0)
    {
      for (float n : q)
      {
        cout << n << ' ' << endl;
      }
    }
  }

  return 0;
}
