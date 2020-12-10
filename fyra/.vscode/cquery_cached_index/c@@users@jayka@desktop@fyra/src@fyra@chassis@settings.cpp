#include <main.h>
using namespace std;

//dist = ticks/tpr * wheelDiam * pi;

class Chassis

{
  public:
  double wheel_diam;
  double wheel_track;
  double wheel_base;
  double tpr;
  int leftA_port, leftB_port;
  int rightA_port, rightB_port;
  int imu_port;

  void setChassis(
    double wheelDiam, double wheelTrack, double wheelBase, double ticksPerRotation,
    int leftAPort, int leftBPort, int rightAPort, int rightBPort,
    int imuAPort
    ){
      wheel_diam = wheelDiam;
      wheel_track = wheelTrack;
      wheel_base = wheelBase;
      tpr = ticksPerRotation;
      leftA_port = leftAPort;
      leftB_port = leftBPort;
      rightA_port = rightAPort;
      rightB_port = rightBPort;
      imu_port = imuAPort;
  }
};
