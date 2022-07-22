#include "main.h"

void initialize() { selectAuton(); }
void autonomous() { runAuton(autonNum); }

void opcontrol() {
    setCoast();
    while (true) {
        double leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        leftY = (leftY * leftY * leftY) / (127 * 127);
        rightY = (rightY * rightY * rightY) / (127 * 127);
        setDrive(leftY, rightY);

        if (master.get_digital_new_press(DIGITAL_L1)) shoot();
        if (master.get_digital_new_press(DIGITAL_L2)) turnToGoal();
        
        if (master.get_digital(DIGITAL_R1)) intake = 127;
        if (master.get_digital(DIGITAL_R2)) intake = -127;
        else intake = 0;

        flywheel = hypot(100 - posX, 100 - posY) * 2;

        pros::delay(10);
    }
}
