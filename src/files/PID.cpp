#include "main.h"

void turnPID(double t) {
    double kP = 5, kD = 35, kI = 0, error = 0, prevError = 0, totalError = 0, TMP = 0;
    double direction = 1;
    while (true) {
        error = fabs(t - imu.get_heading());
        if (error > 180) error = -(error - 360);

        if (fabs(error) < 5) break;
        totalError += error;
        if (totalError > 300) totalError = 300;

        TMP = kP * error + kD * (error - prevError) + kI * totalError;
        if (TMP > 100) TMP = 100;
        if (TMP < -100) TMP = -100;

        setDrive(direction * TMP, direction * -TMP);

        prevError = error;
        wait(10);
    }
}

void drivePID(double fwdL) {
    double kP = 0.45, kD = 5, kI = 0, error = 0, prevError = 0, totalError = 0, LMP = 0;
    double r = right.get_value(), ih = imu.get_heading();
    while (true) {
        double d = right.get_value(), dh = imu.get_heading();
        if (dh > 180) dh -= 360;
        dh -= ih;

        error = fwdL - (d - r);
        if (fabs(error) < 10) break;
        totalError += error;
        if (totalError > 300) totalError = 300;

        LMP = kP * error + kD * (error - prevError) + kI * totalError;
        if (LMP > 110) LMP = 110;
        if (LMP < -110) LMP = -110;

        setDrive(LMP + dh, LMP - dh);

        prevError = error;
        wait(10);
    }
}

void moveTo(double x, double y, double h) {
    double changeX = x - posX;
    double changeY = y - posY;

    double newHeading = atan2(changeX, changeY) * 180 / pi;
    double distance = sqrt(changeX * changeX + changeY * changeY);

    turnPID(newHeading);
    drivePID(distance);
    turnPID(h);
}
