#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "bioscara_hardware_driver/mJointCom.h"

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

  _Joints.addJoint("j4", 0x14, 12, 345 / 2);

  if (_Joints.init())
  {
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  _Joints.enable("j4", 20, 20);
  sleep(1);

  if (!_Joints.joints.at("j4").isHomed())
  {
    _Joints.home("j4", 0, 10, 5, 10); // very low threshold -> should trigger immediatly
  }

  sleep(1);

  _Joints.enableStallguard("j4", 20);

  _Joints.setPosition("j4", 0);
  sleep(1);

  vector<float> q = {0.0, 0.0};  // pos_k, pos_(k-1)
  vector<float> qd = {0.0, 0.0}; // vel_k, vel_(k-1)
  vector<float> q_set = {100.0, 0, -100, 0};

  vector<float> maxAccel = {10.0, 40.0, 10.0, 40.0};
  vector<float> maxVel = {20.0, 20.0, 100, 100};


  for (size_t i = 0; i < 4; i++)
  {
    _Joints.setMaxAcceleration("j4", maxAccel[i % 4]);
    _Joints.setMaxVelocity("j4", maxVel[i % 4]);
    sleep(1);
    if (_Joints.setPosition("j4", q_set[i % 4]) != 0)
    {
      break;
    }

    auto last_measurement = std::chrono::high_resolution_clock::now();
    float t = -1.0;
    while (abs(q[0] - q_set[i % 4] > 1.0))
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
        acc = (q[0] - q[1])/dt;
        t += dt;
      }else{
        acc = 0.0;
        t = 0.0;
      }
      q[1] = q[0];
      qd[1] = qd[0];
      printf("%f, %f, %f, %f",t,q[0],qd[0],acc);

      std::this_thread::sleep_for(10ms);

    }

    std::cout << "Press Enter to Continue...";
    std::cin.ignore();
  }

  _Joints.disables();
  return 0;
}
