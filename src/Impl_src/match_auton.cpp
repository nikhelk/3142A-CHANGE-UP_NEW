#include "Impl/match_auton.h"
#include "Impl/api.h"


void unlockRobotAuton() {
  Flywheel.spin(forward,6,volt);
  task::sleep(1000);
  Flywheel.stop();
}

void redAuton() {
  LOG("Running Red Auton!");
  if(!selector3142a::skills) {
    LOG("dumping in middle goal!");
    unlockRobotAuton();
    task::sleep(400);

    Flywheel.spin(fwd,12,volt);
    Indexer.spin(fwd,10,volt);
    task::sleep(4000);
;
    chassis.setDrive(-6, -6, -6 , -6 );
    task::sleep(3000);
    chassis.setDrive(6, 6, 6 , 6 );
    task::sleep(2000);
    chassis.setDrive(0, 0, 0, 0);
  }

}


void blueAuton() {

  if(!selector3142a::skills) {

    unlockRobotAuton();
    task::sleep(400);

    Flywheel.spin(fwd,12,volt);
    Indexer.spin(fwd,10,volt);
    task::sleep(4000);
;
    chassis.setDrive(-6, -6, -6 , -6 );
    task::sleep(3000);
    chassis.setDrive(6, 6, 6 , 6 );
    task::sleep(2000);
    chassis.setDrive(0, 0, 0, 0);

  }
}