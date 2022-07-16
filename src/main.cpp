#include "main.h"

void initialize() { calibrateIMU(); }
void autonomous() { skills(); }

void opcontrol() {
  setCoast();
  while (true) {
    double ly = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double ry = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // test

    ly = (ly * ly * ly) / (127 * 127);
    ry = (ry * ry * ry) / (127 * 127);
    setDrive(ly, ry);

    if (master.get_digital(DIGITAL_R1))
      intake = 127;
    if (master.get_digital(DIGITAL_R2))
      intake = -127;
    else
      intake = 0;

    if (master.get_digital(DIGITAL_L1))
      shoot();
    if (master.get_digital(DIGITAL_L2))
      turnToGoal();

    flywheel = hypot(100 - posX, 100 - posY) * 2;

    pros::delay(10);
  }
}
