#include "Impl/usercontrol.h"
#include "Impl/api.h"
void usercontrol() {
  while(true) {
    double throttle = BigBrother.Axis3.value();
    double strafe = BigBrother.Axis4.value();
    double turn = BigBrother.Axis1.value();

    if(std::abs((throttle+strafe+turn)) <5) {
      chassis.leftFront.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.leftFront.spin(directionType::fwd, (throttle+strafe+turn), velocityUnits::pct);
    }

    if(std::abs((throttle-strafe+turn)) <5) {
      chassis.leftBack.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.leftBack.spin(directionType::fwd, (throttle-strafe+turn), velocityUnits::pct);
    }

    if(std::abs((throttle+strafe-turn)) <5) {
      chassis.rightBack.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.rightBack.spin(directionType::fwd, (throttle+strafe-turn), velocityUnits::pct);
    }

    if(std::abs((throttle-strafe-turn)) <5) {
      chassis.rightFront.spin(directionType::fwd, 0, velocityUnits::pct);
    }
    else {
      chassis.rightFront.spin(directionType::fwd, (throttle-strafe-turn), velocityUnits::pct);
    }


    task::sleep(20);
  }
}