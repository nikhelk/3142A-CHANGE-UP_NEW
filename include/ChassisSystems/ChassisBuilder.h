#pragma once
#include "ChassisSystems/chassisGlobals.h"

//This class let's us have a chassis "builder"
//we can set specific attributes like gearRatio, kinematical limits 
// See example in src/config/chassis_config.cpp
class FourMotorDrive::FourMotorDriveBuilder {

  private:
  std::array<int32_t, 2> b_frontGroup;
  std::array<int32_t, 2> b_backGroup;
  gearSetting b_gearbox;
  double b_gearRatio;
  Dimensions b_chassisDimensions;
  Limits b_chassisLinearLimits;
  Limits b_chassisAngularLimits;
  std::initializer_list<PDcontroller> b_PDGains;


  public:
    FourMotorDriveBuilder& withMotors(const std::array<int32_t, 2> &frontGroup,const std::array<int32_t, 2> &backGroup) {
      b_frontGroup = frontGroup;
      b_backGroup = backGroup;
      return *this;
    }
    FourMotorDriveBuilder& withGearSetting(const gearSetting gears) {
      b_gearbox = gears;
      return *this;
    }
    FourMotorDriveBuilder& withGearRatio(const double ratio) {
      b_gearRatio = ratio;
      return *this;
    }
    FourMotorDriveBuilder& withDimensions(const Dimensions chassisDimensions) {
      b_chassisDimensions = chassisDimensions;
      return *this;
    }
    FourMotorDriveBuilder& withLinearLimits(Limits linearChassisLimits) {
      b_chassisLinearLimits = linearChassisLimits;
      return *this;
    }
    FourMotorDriveBuilder& withAngularLimits(Limits angularChassisLimits) {
      b_chassisAngularLimits = angularChassisLimits;
      return *this;
    }
    FourMotorDriveBuilder& withPDGains(std::initializer_list<PDcontroller> PDGains) {
      b_PDGains = PDGains;
      return *this;
    }

    ///builder that returns a new FourMotorDrive
    FourMotorDrive buildChassis() const
    {
      return FourMotorDrive{b_frontGroup, b_backGroup, b_gearbox, b_gearRatio,b_chassisDimensions,b_chassisLinearLimits,b_chassisAngularLimits,b_PDGains};
    }

};


