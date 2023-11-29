#include "main.h"
// #include "variables.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants()
{
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.35, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.35, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants()
{
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants()
{
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults()
{
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition()
{
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

///
// Drive Example
///
void drive_example()
{
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
}

///
// Turn Example
///
void turn_example()
{
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed()
{
  // wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}

///
// Swing Example
///
void swing_example()
{
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive

  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}

///
// Interference example
///
void tug(int attempts)
{
  for (int i = 0; i < attempts - 1; i++)
  {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered)
    {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else
    {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example()
{
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  if (chassis.interfered)
  {
    tug(3);
    return;
  }

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
}

///
///
///
///

/*
 * Autonomous Routines designed in EZ Template for 29295A's Team Bot
 * Do Not Reveal This Code to Outside Sources
 * LemLib is Currently Being Tested And Could Replace Current Code
 * First Autonomous Function is For the In-Game 15 Second Autonmous
 * Second is For the Skills Run
 * Remember, We Ball
 */

/*
  Psuedo-Code (In-Game)


*/

// runs 15 autonomous routine for the game.
void gameAuton()
{
  pros::Motor intake(8, pros::E_MOTOR_GEARSET_06);
  pros::ADIDigitalOut Wings('G');

  double rC = -1; // conversion between prog bot and team bot
  int intakeTime = 180;

  while (pros::millis() < 15000)
  { // caps code at 15s for testing purposes (comment out upon actual game) /// remove code before commenting out
    // first drive forward

    chassis.set_drive_pid(90 * rC, 127);
    chassis.wait_until(38 * rC);
    // chassis.reset_pid_targets();

    intake.move(127);
    pros::delay(intakeTime);
    intake.move(0);

    chassis.set_drive_pid(-200 * rC, 127);
    chassis.wait_until(-121 * rC);

    // chassis.set_turn_pid(40, TURN_SPEED);
    // chassis.wait_drive();

    chassis.set_swing_pid(ez::RIGHT_SWING, -40, 127);
    chassis.wait_drive();

    // left arm extends

    Wings.set_value(true);
        pros::delay(200);

        //360

    //chassis.set_turn_pid(-400, 127);
    //chassis.wait_drive();

    chassis.set_drive_pid(-100 * rC, 127);
    chassis.wait_until(-79 * rC);

    chassis.set_swing_pid(ez::RIGHT_SWING, -90, 127);
    chassis.wait_drive();

        Wings.set_value(false);

    chassis.set_drive_pid(-90 * rC, 127);
    chassis.wait_until(-35 * rC);

    chassis.set_drive_pid(80 * rC, 127);
    chassis.wait_until(44 * rC);

    chassis.set_turn_pid(90, 127);
    chassis.wait_drive();

    // intake output
    intake.move(-127);
    pros::delay(intakeTime);
    intake.move(0);

    chassis.set_drive_pid(80 * rC, 127);
    chassis.wait_until(47 * rC);
/*
    chassis.set_drive_pid(-50, 127);
    chassis.wait_until(-20);

    chassis.set_drive_pid(50, 127);
    chassis.wait_until(25);
*/
    chassis.set_drive_pid(-80 * rC, 127);
    chassis.wait_until(-35 * rC);

    chassis.set_turn_pid(26, 127);
    chassis.wait_drive();

    chassis.set_drive_pid(240 * rC, 127);
    chassis.wait_until(170 * rC);

    intake.move(127);
    pros::delay(intakeTime);
    intake.move(0);

    chassis.set_turn_pid(150, 127);
    chassis.wait_drive();

    intake.move(-127);
    pros::delay(intakeTime);
    intake.move(0);

    chassis.set_turn_pid(70, 127);
    chassis.wait_drive();

    chassis.set_drive_pid(120 * rC, 127);
    chassis.wait_until(74 * rC);

    intake.move(127);
    pros::delay(intakeTime);
    intake.move(0);

    chassis.set_turn_pid(0, 127);
    chassis.wait_drive();

    // extend both arms
    Wings.set_value(true);

    chassis.set_drive_pid(-150 * rC, 127);
    chassis.wait_until(-120 * rC);

    chassis.set_drive_pid(60 * rC, 127);
    chassis.wait_until(40 * rC);

    Wings.set_value(false);

        chassis.set_turn_pid(180, 127);
    chassis.wait_drive();

    intake.move(-127);
    pros::delay(intakeTime);
    intake.move(0);

    chassis.set_drive_pid(90 * rC, 127);
    chassis.wait_until(45 * rC);

    while (1)
    {
      pros::delay(100);
    }
  }
      for (int i = 0; i < 2; i++)
    {
      chassis.left_motors[i].move(0);
      chassis.right_motors[i].move(0);
    }
  while (1)
  {
    // intake.move(0);
    // cata.move(0);
    pros::delay(50);
  } // while closer (comment here if necessary)
}
/*
  Psuedo-Code (Skills)
  - Allign With Location For Match Loads
  - Match Load Certain Ammount Using a Delay (Or Millis if Necessary) With Cata Going
  - Allign and Drive Forward on an Arc To Allign Perp With Middle Bar
  - Extend Pnuematic Arm and Grab Corner Ball
  - Close Arm and Ram in On Arc
  - Back Out and Arc To Far Side
  - Both Arms Extend And Push On A Slight Angle
  - Back Up and Arc to Final Position
  - Push Other Side In To Finish
*/

// skills autonomous routine //no while loop is needed as it is preset to 1 minute
void skillsAuton()
{
}