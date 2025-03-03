#include <unistd.h>
#include "serial_test/mJointCom.h"


using namespace std;

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  u_int8_t addresses[] = {0xa1}; // Create an array of u_int8_t
  std::string names[] = {"j1"};  // Create an array of std::string

  Joint_comms Joints(1, addresses, names); // Pass the arrays to the constructor
  if(Joints.init("/dev/ttyS0",115200)){
    cerr << "Could not establish connection to joints" << endl;
    return -1;
  }

  float angle = 0;
  while (1)
  {
    Joints.joints[0].getAngle(angle);
    cout << angle << endl;
    usleep(500*1000);
  }

  // Replace with your serial port name
  // const char *portname = "/dev/ttyS0";
  // int fd = openSerialPort(portname);
  // if (fd < 0)
  //   return 1;

  // if (!configureSerialPort(fd, B115200))
  // {
  //   closeSerialPort(fd);
  //   return 1;
  // }

  //   char txBuf[256];
  //   while (true) {
  //     cout << "Enter message to send: ";
  //     cin.getline(txBuf, sizeof(txBuf) - 1); // Read input from cin
  //     strcat(txBuf, "\n"); // Append newline character

  //     if (writeToSerialPort(fd, txBuf, strlen(txBuf)) < 0) {
  //         cerr << "Error writing to serial port: "
  //              << strerror(errno) << endl;
  //     } else {
  //         cout << "Sent: " << txBuf;
  //     }

  //     usleep(1000 * 1000); // Sleep for 1 second
  // }

  // char txBuf[3] = {0xa1, 1, ANGLEMOVED};
  // while (1)
  // {
  //   tcflush(fd, TCIOFLUSH);
  //   if (writeToSerialPort(fd, txBuf, sizeof(txBuf)) < 0)
  //   {
  //     cerr << "Error writing to serial port: "
  //          << strerror(errno) << endl;
  //   }
  //   usleep(100 * 1000);

  //   char buffer[100];
  //   auto beg = chrono::high_resolution_clock::now();
  //   int n = readFromSerialPort(fd, buffer, 1, 100);
  //   if (n < 0)
  //   {
  //     cerr << "Error reading from serial port: "
  //          << strerror(errno) << endl;
  //   }
  //   else
  //   {
  //     cout << "Read from serial port: "
  //          << std::string(buffer, n) << endl;

  //     buffer[100] = {};
  //     n = readFromSerialPort(fd, buffer, 4, 100);
  //     if (n < 0)
  //     {
  //       cerr << "Error reading from serial port: "
  //            << strerror(errno) << endl;
  //     }
  //     else
  //     {
  //       int32_t angle;
  //       memcpy(&angle, buffer, 4);
  //       cout << "Read from serial port: "
  //            << angle << endl;
  //     }
  //   }
  //   auto end = chrono::high_resolution_clock::now();
  //   cout << chrono::duration_cast<chrono::microseconds>(end - beg).count() << endl;
  // }

  // closeSerialPort(fd);
  return 0;
}
