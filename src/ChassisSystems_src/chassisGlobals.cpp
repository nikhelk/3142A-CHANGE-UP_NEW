#include "ChassisSystems/chassisGlobals.h"
#include "Util/vex.h"

FourMotorDrive::FourMotorDrive( const std::array<int32_t, 2> &frontGroup,
                 const std::array<int32_t, 2> &backGroup,
                 const gearSetting setting, const double gearRatio,
                 const Dimensions chassisDimensions, const Limits linLimits,
                 const Limits angLimits,
                 const std::initializer_list<PDcontroller> PDGains)

    : m_chassisDimensions(chassisDimensions), m_chassisLinearLimits(linLimits),
    m_chassisAngularLimits(angLimits),
      leftFront(frontGroup[LEFT], setting),
      rightFront(frontGroup[RIGHT], setting),
      leftBack(backGroup[LEFT], setting), rightBack(backGroup[RIGHT], setting) {

  enum posPIDType { DISTANCEPID, ANGLEPID, TURNPID };
  int count = 0;
  for (auto &element : PDGains) {

    switch (count) {

    case DISTANCEPID:
      distancePID.setPD(element.kP, element.kD);
      break;
    case ANGLEPID:
      anglePID.setPD(element.kP, element.kD);
      break;
    case TURNPID:
      turnPID.setPD(element.kP, element.kD);
      break;
    }

    count++;
  }
  this->gearRatio = gearRatio;
  this->setting = setting;
}

void FourMotorDrive::setDrive(double leftFrontVoltage, double rightFrontVoltage , double leftBackVoltage, double rightBackVoltage) {
  leftFront.spin(fwd,leftFrontVoltage,volt);
  rightFront.spin(fwd,rightFrontVoltage,volt);
  
  leftBack.spin(fwd,leftBackVoltage,volt);
  rightBack.spin(fwd,rightBackVoltage,volt);
}


void FourMotorDrive::setReverseSettings(
    const std::array<bool, 2> &LeftReverseVals,
    const std::array<bool, 2> &RightReverseVals) {
  leftFront.setReversed(LeftReverseVals[0]);
  leftBack.setReversed(LeftReverseVals[1]);
  rightFront.setReversed(RightReverseVals[0]);
  rightBack.setReversed(RightReverseVals[1]);
}

void FourMotorDrive::resetPosition() {
  this->leftFront.resetPosition();
  this->leftBack.resetPosition();
  this->rightFront.resetPosition();
  this->rightBack.resetPosition();
}

void FourMotorDrive::resetRotation() {
  this->leftFront.resetRotation();
  this->leftBack.resetRotation();
  this->rightFront.resetRotation();
  this->rightBack.resetRotation();
}







Dimensions::Dimensions( const long double trackWidth,  const long double wheelRadius, const long double ticksToDegrees)
    : m_trackWidth(trackWidth), m_wheelRadius(wheelRadius) {}

Limits::Limits(  long double maxVelocity,   long double maxAcceleration)
    : m_maxVelocity(maxVelocity), m_maxAcceleration(maxAcceleration) {}

