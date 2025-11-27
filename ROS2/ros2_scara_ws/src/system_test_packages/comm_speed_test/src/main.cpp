#include <signal.h>
#include <unistd.h>
#include "bioscara_arm_hardware_driver/mJoint.h"
#include <unordered_map>
#include <cmath>
#include <chrono>

using namespace std;

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
  _joints.insert({"j1", Joint("j1", 0x11, 1, 0)});
  _joints.insert({"j2", Joint("j2", 0x12, 1, 0)});
  _joints.insert({"j3", Joint("j3", 0x13, 1, 0)});
  _joints.insert({"j4", Joint("j4", 0x14, 1, 0)});

  for (auto &[name, joint] : _joints)
  {
    int rc = joint.init();
    if(rc < 0){
      std::cerr << "Failed to init:" << rc << std::endl;
      return -1;
    }
    rc = joint.enable(20, 20);
    if(rc < 0){
      std::cerr << "Failed to enable:" << rc << std::endl;
      return -1;
    }
    rc = joint.enableStallguard(5);
    if(rc < 0){
      std::cerr << "Failed to enableStallguard:" << rc << std::endl;
      return -1;
    }
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