#include "main.h"

double posX = 0, posY = 0, degrees = 0, radians = 0;
bool odom = true;

void odometry() {
    double dR = 0, dL = 0, dB = 0, dX = 0, dY = 0, dH = 0;
    double oldR = 0, oldL = 0, oldB = 0, trackWidth = 10, backDistance = 5;
    while (odom) {
        dR = right.get_value() - oldR;
        dL = left.get_value() - oldL;
        dB = back.get_value() - oldB;

        dX = wheelConstant * (dB - backDistance * ((dR + dL) / trackWidth));
        dY = wheelConstant * (dR + dL) / 2;
        dH = wheelConstant * (dR - dL) / trackWidth;

        radians += dH / 2;
        degrees = radians * 180 / pi;

        posX += dX * cos(radians) - dY * sin(radians);
        posY += dX * sin(radians) + dY * cos(radians);
        radians += dH / 2;

        oldR = right.get_value();
        oldL = left.get_value();
        oldB = back.get_value();

        wait(10);
    }
}