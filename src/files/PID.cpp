#include "main.h"

double kP = 0.45, kD = 5, kI = 0, error = 0, prevError = 0, totalError = 0;
double tkP = 5, tkD = 35, tkI = 0, turnError = 0, turnPrevError = 0,
       totalTurnError = 0;
double LMP = 0, TMP = 0;

void turnPID(double t) {
  while (true) {
    turnError = t - imu.get_heading();
    if (turnError > 180)
      turnError = -1 * (turnError - 360);
    if (turnError < -180)
      turnError = -1 * (turnError + 360);
    if (fabs(turnError) < 1)
      break;
    totalTurnError += turnError;

    TMP = tkP * turnError + tkD * (turnError - turnPrevError) +
          tkI * totalTurnError;
    if (TMP > 100)
      TMP = 100;
    if (TMP < -100)
      TMP = -100;
    setDrive(TMP, -1 * TMP);

    turnPrevError = turnError;
    wait(10);
  }
}

void drivePID(double fwdL) {
  double r = right.get_value();
  double ih = imu.get_heading();
  while (true) {
    double d = right.get_value();
    double dh = imu.get_heading();
    if (dh > 180)
      dh -= 360;
    dh -= ih;

    error = fwdL - (d - r);
    if (error < 2)
      break;
    totalError += error;

    LMP = kP * error + kD * (error - prevError) + kI * totalError;
    if (LMP > 110)
      LMP = 110;
    if (LMP < -110)
      LMP = -110;
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