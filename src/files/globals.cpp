#include "main.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor l1(1, pros::E_MOTOR_GEARSET_06, false,
               pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor l2(2, pros::E_MOTOR_GEARSET_06, false,
               pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor l3(3, pros::E_MOTOR_GEARSET_06, false,
               pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor r1(4, pros::E_MOTOR_GEARSET_06, false,
               pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor r2(5, pros::E_MOTOR_GEARSET_06, false,
               pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor r3(6, pros::E_MOTOR_GEARSET_06, false,
               pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor flywheel(7, pros::E_MOTOR_GEARSET_06, false,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake(8, pros::E_MOTOR_GEARSET_06, false,
                   pros::E_MOTOR_ENCODER_DEGREES);

pros::Rotation rightRot(10);
pros::Rotation leftRot(11);

pros::ADIEncoder right('A', 'B'), left('C', 'D'), back('E', 'F');
pros::ADIDigitalOut indexer('G');
pros::ADIAnalogIn detector('H');
pros::Imu imu(9);

double pi = 3.14159265358979323846;
double wc = pi * 2.75 / 360;

void calibrateIMU() {
  imu.reset();
  while (imu.is_calibrating())
    pros::delay(10);
  imu.tare_heading();
}

void setDrive(double l, double r) {
  l1 = l, l2 = l, l3 = l, r1 = r, r2 = r, r3 = r;
}

void h(pros::Motor m) { m.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); }
void c(pros::Motor m) { m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); }

void setHold() { h(l1), h(l2), h(l3), h(r1), h(r2), h(r3); }
void setCoast() { c(l1), c(l2), c(l3), c(r1), c(r2), c(r3); }

void wait(double time) { pros::delay(time); }

void turnToGoal() {
  double disX = 100 - posX;
  double disY = 100 - posY;

  double newhead = atan2(disY, disX);
  if (newhead < 0)
    newhead += 360;
  turnPID(newhead);
}

void shoot() {
  indexer.set_value(true);
  wait(100);
  indexer.set_value(false);
  wait(100);
  indexer.set_value(true);
  wait(100);
  indexer.set_value(false);
  wait(100);
  indexer.set_value(true);
  wait(100);
  indexer.set_value(false);
}