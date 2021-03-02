#include "ChassisSystems/chassisGlobals.h"
#include "Util/vex.h"
#include "ChassisSystems/odometry.h"


void FourMotorDrive::moveToPointAndAngle(double xTarget, double yTarget, double angleTarget, double timeAllocatted) {
  double time = 0;
  while(time < timeAllocatted) {

  double poseError = hypot(yTarget - positionArray[ODOM_Y], xTarget - positionArray[ODOM_X]);
 
  posPID driveBase(5, 0);

  posPID angleBase(5,0);
  
  
  double XYpower = driveBase.calculatePower(poseError);
 
  double AnglePower = angleBase.calculatePower(angleTarget, positionArray[ODOM_THETA]);

  double direction = atan2(-yTarget + positionArray[ODOM_Y], -xTarget + positionArray[ODOM_X]) - positionArray[ODOM_THETA] - M_PI/2;
  //Tune global_a on in the odometry function so it reports radians.  Which will allow me to move in the same direction while turning

    //Front Right and Back Left
  double p1 = -cos(direction + M_PI/4);
    //Front Left and Back Right
  double p2 = sin(direction + M_PI/4);
 
    double s = fmax(std::abs(p1), std::abs(p2));

    setDrive( XYpower * (p2/s)+ AnglePower, XYpower * (p1/s) + AnglePower, -1*XYpower * (p1/s) - AnglePower, -1*XYpower * (p2/s) - AnglePower);
    time += 10;
    task::sleep(10);

  }
 
    // leftFront.spin(forward, XYpower * (p2/s)+ AnglePower, volt);
    // rightFront.spin(reverse, XYpower * (p1/s) - AnglePower, volt);
    // leftBack.spin(forward, XYpower * (p1/s) + AnglePower, volt);
    // rightBack.spin(reverse, XYpower * (p2/s) - AnglePower, volt);
}