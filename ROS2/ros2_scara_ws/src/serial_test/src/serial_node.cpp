#include "serial_test/uSerialJoint.h"

using namespace std;

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  // Replace with your serial port name
  const char *portname = "/dev/ttyS1";
  int fd = openSerialPort(portname);
  if (fd < 0)
    return 1;

  if (!configureSerialPort(fd, B9600))
  {
    closeSerialPort(fd);
    return 1;
  }

  const char *message = "Hello, Serial Port!";
  if (writeToSerialPort(fd, message, strlen(message)) < 0)
  {
    cerr << "Error writing to serial port: "
         << strerror(errno) << endl;
  }

  char buffer[100];
  int n = readFromSerialPort(fd, buffer, sizeof(buffer));
  if (n < 0)
  {
    cerr << "Error reading from serial port: "
         << strerror(errno) << endl;
  }
  else
  {
    cout << "Read from serial port: "
         << std::string(buffer, n) << endl;
  }

  closeSerialPort(fd);
  return 0;
}
