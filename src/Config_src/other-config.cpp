#include "Config/other-config.h"

using namespace vex;

using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

controller BigBrother = controller(primary);

motor Flywheel = motor(PORT10, ratio6_1, true);
motor IntakeL = motor(PORT7, ratio6_1, false);
motor IntakeR = motor(PORT6, ratio6_1, true);
motor Indexer = motor(PORT11, ratio6_1, false);


triport Expander19 = triport(PORT19);
line outyLine = line(Expander19.A);
line bottomLine = line(Expander19.B);
line middleLine = line(Expander19.C);
line topLine = line(Expander19.D);


