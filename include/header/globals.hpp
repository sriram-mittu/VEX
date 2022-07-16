#include "main.h"

extern pros::Motor l1,l2,l3,r1,r2,r3;
extern pros::Motor flywheel;
extern pros::Motor intake;
extern pros::Controller master;
extern pros::ADIEncoder right, left, back;
extern pros::ADIDigitalOut indexer, expand;
extern pros::Imu imu;
extern double pi, wc;

void calibrateIMU();
void setDrive(double l, double r);
void h(pros::Motor m);
void c(pros::Motor m);
void setHold();
void setCoast();
void wait(double time);
void turnPID(double t);
void drivePID(double fwdL);
void moveTo(double x, double y, double h);
void turnToGoal();
void shoot();
void driverControl();