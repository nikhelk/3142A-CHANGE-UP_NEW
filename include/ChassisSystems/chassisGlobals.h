#pragma once
#include "ChassisSystems/posPID.h"
#include "Util/premacros.h"
#include "Util/vex.h"
#include "chassisConstraints.h"
#include <vector>

using namespace vex;

/*
* Header file with all the global classes (motors, sensors) and other
applications such as PID and Motion Profiling
*
* @author Nikhel Krishna, 3142A

*/

/// FourMotorDrive class that we use for our builder, and chassis functions
class FourMotorDrive {
private:

  /**
   * Omtimizes the output of a PD controller for turning
   * if the difference between the output and targer is less than 180, -1*angleOutput otherwise
   * @param targetAngle the target angle we have to reach
   * @param angleOutput that needs to be optimized
   */

  inline void adjustOutput(double targetAngle, double &angleOutput);

  /**
   * Changes the voltage of our feedforward/feedback controller depending if we are going backwards
   * Will multiply by -1 if we are going backwards
   * @param lVoltage left voltage output
   * @param rVoltage tright voltage output
   * @param backwards if we are going backwards or not
   */

  inline void checkBackwards(double &lVoltage, double &rVoltage, bool backwards);
public:
  Dimensions m_chassisDimensions;
  Limits m_chassisLinearLimits;
  Limits m_chassisAngularLimits;

  posPID distancePID;
  posPID anglePID;
  posPID turnPID;

  double gearRatio;
  gearSetting setting;

  enum backOrFront {
    LEFT,
    RIGHT,
  };

  class FourMotorDriveBuilder;

  motor leftFront;
  motor rightFront;
  motor leftBack;
  motor rightBack;

  /**
   * Initializes 4 motor drive
   * @param leftGroup vector of left motor ports (front,back)
   * @param rightGroup vector of right motor ports (front,back)
   * @param setting gear cartridge type (36:1.18:1,6:1)
   * @param gearRatio gear ratio
   * @param chassisDimensions chassis dimensions (trackWidth and wheel size)
   * @param chassisLimits chassis limits (max velocity and acceleration)
   * @param PDGains Controller chassis parameters
   */

  FourMotorDrive(const std::array<int32_t, 2> &leftGroup,
                 const std::array<int32_t, 2> &rightGroup,
                 const gearSetting setting, const double gearRatio,
                 const Dimensions chassisDimensions, const Limits linLimits,
                 const Limits angLimits,
                 const std::initializer_list<PDcontroller> PDGains);

  /**
   * Handles the reversal of motors.
   * @param LeftReverseVals boolean array of leftFront and leftBack desired
   * reversal states
   * @param RightReverseVals boolean array of rightFront and rightBack desired
   * reversal states
   */

  void setReverseSettings(const std::array<bool, 2> &LeftReverseVals, const std::array<bool, 2> &RightReverseVals);

  /// resets the chassis encoders to 0
  void resetPosition();

  /// resets the chassis encoders to 0
  void resetRotation();

  void setDrive(double leftFrontVoltage,double leftBackVoltage, double rightFrontVoltage, double rightBackVoltage);

  void moveToPointAndAngle(double xTarget, double yTarget, double angleTarget,double timeAllocatted);
};





