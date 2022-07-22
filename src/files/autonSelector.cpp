#include "main.h"
#include "pros/llemu.hpp"
using namespace pros;

int autonNum = 1;
int numOfAutons = 3;
bool selectingAuton = true;

void onLeftButton() {
    if (autonNum > 1) autonNum--;
    if (autonNum == 1) autonNum = numOfAutons;    
}

void onRightButton() {
    if (autonNum < numOfAutons) autonNum++;
    if (autonNum == numOfAutons) autonNum = 0;
}

void centerButton() {
    selectingAuton = false;
    lcd::clear();
    runAuton(autonNum);    
}

void setText(int number) {
    lcd::clear();
    if (number == 1) lcd::set_text(1, "1. Skills Auton");    
    if (number == 2) lcd::set_text(1, "2. Solo Autonomous Win Point");    
    if (number == 3) lcd::set_text(1, "3. Left Side Autonomous Win Point");
}

void selectAuton() {
    Task calibrate(calibrateIMU);

    lcd::initialize();
    lcd::set_background_color(0, 0, 0);

    lcd::register_btn0_cb(onLeftButton);    
    lcd::register_btn1_cb(centerButton);
    lcd::register_btn2_cb(onRightButton);

    while(selectingAuton) {
        setText(autonNum);
        delay(10);
    }
    
    setText(autonNum);
}

void runAuton(int anum) {
    if (anum == 1) skills();
    if (anum == 2) soloAWP();
    if (anum == 3) leftAWP();
}