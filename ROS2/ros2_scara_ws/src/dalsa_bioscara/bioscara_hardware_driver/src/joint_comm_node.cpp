#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "bioscara_hardware_driver/mJointCom.h"

// #include <cstdio>
// #include <cstdlib>

#include <cmath>

using namespace std;

Joint_comms _Joints;

void INT_handler(int s)
{
  printf("Caught signal %d\n", s);
  _Joints.disables();
  exit(0);
}

int main(int argc, char **argv)
{
  signal(SIGINT, INT_handler);

  (void)argc;
  (void)argv;

  _Joints.addJoint("j4", 0x14, 1 /*12*/, 0 /*345/2*/);

  if (_Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  _Joints.enable("j4", 20, 20);
  sleep(1);

  _Joints.home("j4", 0, 10, 5, 10); // very low threshold -> should trigger immediatly

  sleep(1);

  _Joints.enableStallguard("j4", 20);
  _Joints.setPosition("j4", 0);
  sleep(1);

  vector<float> q = {0.0, 0.0};  // pos_k, pos_(k-1)
  vector<float> qd = {0.0, 0.0}; // vel_k, vel_(k-1)
  vector<float> q_set = {100.0, 0, -100, 0};

  float maxAccel = 5000.0;
  float maxVel = 10000.0;

  _Joints.setMaxAcceleration("j4", maxAccel);
  _Joints.setMaxVelocity("j4", maxVel);
  sleep(1);

  char file[0x100];
  // snprintf(file, sizeof(file), "/home/scara/bioscara/test/accel_test_%.0f_%.0f.csv", maxAccel,maxVel);
  snprintf(file, sizeof(file), "/home/scara/bioscara/test/accel_test_%.0f_%.0f_online_change.csv", maxAccel,maxVel);
  FILE *fp = std::fopen(file, "w"); // Open file for writing
  fprintf(fp,"maxAccel: %f\n", maxAccel);
  fprintf(fp,"maxVel: %f\n", maxVel);
  fprintf(fp,"time, position, velocity, accleration\n");
  float angle = 360*100;
  _Joints.setPosition("j4", angle);

  auto last_measurement = std::chrono::high_resolution_clock::now();
  float t = -1.0;
  do
  {
    _Joints.getPosition("j4", q[0]);
    _Joints.getVelocity("j4", qd[0]);
    auto now = std::chrono::high_resolution_clock::now();
    float acc;
    if (t >= 0)
    {
      const std::chrono::duration<float> elapsed = now - last_measurement;
      last_measurement = now;
      float dt = elapsed.count();
      acc = (qd[0] - qd[1]) / dt;
      t += dt;
    }
    else
    {
      acc = 0.0;
      t = 0.0;
    }
    q[1] = q[0];
    qd[1] = qd[0];
    fprintf(fp,"%f, %f, %f, %f\n", t, q[0], qd[0], acc);

    std::this_thread::sleep_for(10ms);

    if (t>1 && maxAccel != -1)
    {
      _Joints.setMaxAcceleration("j4", 2000);
      maxAccel = -1;
    }
    if (t>2 && maxVel != -1)
    {
      _Joints.setMaxVelocity("j4", 5000);
      maxVel = -1;
    }
    

  } while (abs(q[0] - angle) > 1);

  // std::cout << "Press Enter to Continue...";
  // std::cin.ignore();

  _Joints.disables();
  fclose(fp);
  return 0;
}
