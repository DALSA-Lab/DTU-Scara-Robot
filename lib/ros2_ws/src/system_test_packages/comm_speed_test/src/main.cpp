#include <signal.h>
#include <unistd.h>
#include "bioscara_arm_hardware_driver/mJoint.h"
#include <unordered_map>
#include <cmath>
#include <chrono>

using namespace std;
using namespace bioscara_hardware_drivers;

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

  std::unordered_map<std::string, Joint> _joints;
  _joints.insert({"j1", Joint("j1", 0x11, 35, -3.04647, 3.04647)});
  _joints.insert({"j2", Joint("j2", 0x12, -2 * M_PI / 0.004, 0.338, 0.0)});
  _joints.insert({"j3", Joint("j3", 0x13, 24, -2.62672, 2.62672)});
  _joints.insert({"j4", Joint("j4", 0x14, 12, -3.01069, 3.01069)});

  for (auto &[name, joint] : _joints)
  {
    RETURN_ON_NEGATIVE((int)joint.init(),-1);
    RETURN_ON_NEGATIVE((int)joint.enable(20, 20),-1);
    RETURN_ON_NEGATIVE((int)joint.enableStallguard(6), -1);
  }

  FILE *fp = std::fopen("/home/scara/bioscara/test/RW_test/temp_100kHz.csv", "w"); // Open file for writing
  fprintf(fp, "time [ms], iteration [-], joint [-], read [us], write[us]\n");

  auto prog_start = std::chrono::high_resolution_clock::now();
  for (size_t i = 1; i <= 1000; i++)
  {

    for (auto &[name, joint] : _joints)
    {

      float v;
      auto start = std::chrono::high_resolution_clock::now();
      /* Insert the function(s) that should be timed below. Currently the getVelocity function is being timed. */
      // joint.getPosition(v);
      joint.getVelocity(v);
      auto now = std::chrono::high_resolution_clock::now();
      const std::chrono::duration<double, std::micro> t_read = now - start;

      start = std::chrono::high_resolution_clock::now();
      // joint.setPosition(v);
      joint.setVelocity(v);
      now = std::chrono::high_resolution_clock::now();
      const std::chrono::duration<double, std::micro> t_write = now - start;

      const std::chrono::duration<double, std::milli> t = now - prog_start;

      fprintf(fp, "%f, %ld, %s, %f, %f\n", t.count(), i, name.c_str(), t_read.count(), t_write.count());
    }
  }

  fclose(fp);
  return 0;
}