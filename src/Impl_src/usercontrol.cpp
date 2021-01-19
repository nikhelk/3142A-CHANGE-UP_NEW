#include "Impl/usercontrol.h"
#include "Impl/api.h"

void unlockRobot() {
  Flywheel.spin(forward,6,volt);
  task::sleep(300);
  Flywheel.stop();
}

double cubic(double input) { 

  //Used to ramp the turning speed, for finer control at lower speeds.

  if (input != 0) {return (pow(input,3))/(100*std::abs(input));}
  else {return 0;}
}

void usercontrol() {

 // unlockRobot();
  while(true) {

  //DRIVING
    double throttle = BigBrother.Axis3.value();
    double strafe = BigBrother.Axis4.value();
    double turn = cubic(BigBrother.Axis1.value());

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

  //INTAKES
    if(BigBrother.ButtonR1.pressing()){
      IntakeL.spin(fwd,12,volt);
      IntakeR.spin(fwd,12,volt); 
      Indexer.spin(fwd,12,volt);                                                                 
    }
    else if(BigBrother.ButtonR2.pressing()){
      IntakeL.spin(reverse,12,volt);
      IntakeR.spin(reverse,12,volt);
      Indexer.spin(reverse,12,volt);
    }
    else{
      IntakeL.stop();
      IntakeR.stop();
      Indexer.stop();
    }
  
  //INDEXER
    if(BigBrother.ButtonR1.pressing()==false) {
    if(BigBrother.ButtonL1.pressing()){
      Indexer.spin(fwd,12,volt);
    }
    else if(BigBrother.ButtonL2.pressing()){
      Indexer.spin(reverse,12,volt);
    }
    else {
      Indexer.stop();
    }
    }


  //FLYWHEEL
    if(BigBrother.ButtonA.pressing()) {
      Flywheel.spin(forward,12,volt);
    }
    else if(BigBrother.ButtonB.pressing()) {
      Flywheel.spin(reverse,12,volt);
    }
    else{
      Flywheel.stop();
    }

  task::sleep(20);
  
  }
}