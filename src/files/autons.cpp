#include "main.h"

void skills() {
    posX = 36;
    posY = 9;
    intake = 127;
    followPath({{36, 12}, {31, 47}, {50, 75}}, 10);
    shoot();
}

void soloAWP() {
    posX = 36;
    posY = 9;
    flywheel = 127;
    turnToGoal();
    shoot();
}

void leftAWP() {
    posX = 36;
    posY = 9;
    intake = 127;
    flywheel = 127;
    turnToGoal();
    shoot();
}
