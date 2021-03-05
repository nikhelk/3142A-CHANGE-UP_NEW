#include "ChassisSystems/chassisGlobals.h"
#include "Util/vex.h"
#include "ChassisSystems/odometry.h"
#include "Util/mathAndConstants.h"

void FourMotorDrive::moveToPointAndAngle(double xTarget, double yTarget, double angleTarget) {

  bool atTarget = false;

  double prevPoseError;

  math3142a::TimeoutTimer NotMovedTimer(10,250);
  math3142a::TimeoutTimer CloseTimer(10,250);
  
  while(!atTarget) {

  double poseError = hypot(yTarget - positionArray[ODOM_Y], xTarget - positionArray[ODOM_X]);
 
  posPID driveBase(3, 0);

  posPID angleBase(5,0);
  
  
  double XYpower = driveBase.calculatePower(poseError);
 
  double AnglePower = 0;

  double direction = atan2(-yTarget + positionArray[ODOM_Y], -xTarget + positionArray[ODOM_X]) - positionArray[ODOM_THETA]-M_PI/2;
  std::cout << direction << '\n';
  //Tune global_a on in the odometry function so it reports radians.  Which will allow me to move in the same direction while turning

    //Front Right and Back Left
  double p1 = -cos(direction + M_PI/4);
    //Front Left and Back Right
  double p2 = sin(direction + M_PI/4);
 
  double s = fmax(std::abs(p1), std::abs(p2));

  setDrive( XYpower * (p2/s)+ AnglePower, XYpower * (p1/s) + AnglePower, (XYpower * (p1/s) - AnglePower), (XYpower * (p2/s) - AnglePower));
    
  if(poseError < .5) {
    CloseTimer.m_currentTime += 10;
  }

  if(std::abs(poseError-prevPoseError) < .1) {
    NotMovedTimer.m_currentTime += 10;
  }

  if(CloseTimer.m_currentTime > CloseTimer.m_timeout ) {
    atTarget = true;
  }

  prevPoseError = poseError;
  task::sleep(10);

  }

  setDrive(0, 0, 0, 0);
 
    // leftFront.spin(forward, XYpower * (p2/s)+ AnglePower, volt);
    // rightFront.spin(reverse, XYpower * (p1/s) - AnglePower, volt);
    // leftBack.spin(forward, XYpower * (p1/s) + AnglePower, volt);
    // rightBack.spin(reverse, XYpower * (p2/s) - AnglePower, volt);
}