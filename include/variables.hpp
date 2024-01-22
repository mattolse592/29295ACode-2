#ifndef VARIABLES_H
#define VARIABLES_H

#include "main.h"

//static pros::Motor intake(7, pros::E_MOTOR_GEARSET_06); //new
static pros::Motor intake(1, pros::E_MOTOR_GEARSET_06); //old
//static pros::Motor intake(5000, pros::E_MOTOR_GEARSET_06);

//static pros::Motor cataleft(12, pros::E_MOTOR_GEARSET_36);
//static pros::Motor cataright(-19, pros::E_MOTOR_GEARSET_36); //comment out with prog
//static pros::Motor cataleft(8999, pros::E_MOTOR_GEARSET_36);
//static pros::Motor cataright(9000, pros::E_MOTOR_GEARSET_36); // comment out with real bort

//static pros::Motor_Group cata({cataleft, cataright});

static pros::Motor cata(3, pros::E_MOTOR_GEARSET_36);

//pros::Rotation rot(2); //old
pros::Rotation rot(22);
//pros::ADIDigitalIn limitSwitch('H');

pros::ADIDigitalOut blocker('H');
pros::ADIDigitalOut Wings('G');
//pros::ADIDigitalOut rightWing('F');

#endif