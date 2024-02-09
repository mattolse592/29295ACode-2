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
  chassis.set_pid_constants(&chassis.headingPID, 15, 0, 46, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
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

  chassis.set_drive_pid(24, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();

  while (1)
  {
    pros::delay(100);
  }
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

  // chassis.set_turn_pid(45, TURN_SPEED);
  // chassis.wait_drive();

  // chassis.set_turn_pid(0, TURN_SPEED);
  // chassis.wait_drive();
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

/*
 * Autonomous Routines designed in EZ Template for 29295A's Team Bot.
 * LemLib is Currently Being Tested And Could Replace Current Code (false information)
 * Autos Are in the Following Order:
 * Defensive
 * Offensive
 * Skills
 * Remember, We Ball.
 */

const int defDriveSpeed = 110;
const int defTurnSpeed = 110;

void defAuton()
{
  // variables for intake, blocker, and wings
  pros::Motor intake(1, pros::E_MOTOR_GEARSET_06);
  pros::ADIDigitalOut blocker('A');
  pros::ADIDigitalOut rWing('B');
  pros::ADIDigitalOut lWing('C');

  int intakeTime = 320;

  // outtake preload
  intake = -70;

  pros::delay(300);
  intake = -127;
  pros::delay(intakeTime);
  intake = 0;

  // turn onto first triball]
  chassis.set_turn_pid(-70, defTurnSpeed);
  chassis.wait_drive();

  // drive to and intake it, while staying on line
  intake = 127;

  chassis.set_drive_pid(-114, defDriveSpeed);
  chassis.wait_drive();

  intake = 30;

  // turn to other ball to hit over with wing
  chassis.set_turn_pid(-179, defTurnSpeed);
  chassis.wait_drive();

  // drop left wing
  lWing.set_value(true);

  // drive at and hit over triball
  chassis.set_drive_pid(45, defDriveSpeed);
  chassis.wait_drive();

  // retrack wing
  lWing.set_value(false);

  // back up for turn
  chassis.set_drive_pid(-20, defDriveSpeed);
  chassis.wait_drive();

  // turn to point where triball will be dropped off
  chassis.set_turn_pid(-240, defTurnSpeed);
  chassis.wait_drive();

  // drive to outtake point there
  chassis.set_drive_pid(-55, defDriveSpeed);
  chassis.wait_drive();

  // turn to outtake
  chassis.set_turn_pid(-313, defTurnSpeed);
  chassis.wait_drive();

  // outtake middle ball
  intake = -127;
  pros::delay(intakeTime);
  intake = 0;

  // turn back to line up with back wall
  chassis.set_turn_pid(-10, 127);
  chassis.wait_drive();

  // drive there
  chassis.set_drive_pid(14, defDriveSpeed);
  chassis.wait_drive();

  // turn to get lined up with matchload zone
  chassis.set_turn_pid(-90, 127);
  chassis.wait_drive();

  // swing to grab corner ball
  chassis.set_swing_pid(RIGHT_SWING, -135, defTurnSpeed);
  chassis.wait_drive();

  // drop right wing to clear
  rWing.set_value(true);

  // clear
  chassis.set_drive_pid(14, defDriveSpeed);
  chassis.wait_drive();

  // close wing
  rWing.set_value(false);

  // swing to line up and clear
  chassis.set_swing_pid(RIGHT_SWING, -180, defTurnSpeed);
  chassis.wait_drive();

  // drive forward slightly
  chassis.set_drive_pid(27, defDriveSpeed);
  chassis.wait_drive();

  // left wing drops
  lWing.set_value(true);

  // clear side
  chassis.set_drive_pid(50, defDriveSpeed);
  chassis.wait_drive();
}

// runs 15 offensive autonomous routine for the game.
void offAuton()
{
  pros::Motor intake(1, pros::E_MOTOR_GEARSET_06);
  pros::ADIDigitalOut blocker('A');
  pros::ADIDigitalOut rWing('B');
  pros::ADIDigitalOut lWing('C');

  int intakeTime = 320;

  // intake and move to first triball
  intake.move(127);
  chassis.set_drive_pid(-163, 127);
  chassis.wait_drive();
  intake.move(30);

  // swing to outake
  chassis.set_turn_pid(-53, 127);
  chassis.wait_drive();

  lWing.set_value(true);

  // slam
  chassis.set_drive_pid(90, 127);
  chassis.wait_drive();

  // back
  chassis.set_drive_pid(-35, 127);
  chassis.wait_drive();

  lWing.set_value(false);

  chassis.set_turn_pid(-233, 127);
  chassis.wait_drive();

  intake.move(0);

  chassis.set_drive_pid(-54, 127);
  chassis.wait_drive();

  // back up slightly
  chassis.set_drive_pid(30, 127);
  chassis.wait_drive();

  // turn to snatch
  chassis.set_turn_pid(-79, 127);
  chassis.wait_drive();

  // grab 3rd ball
  intake.move(127);
  chassis.set_drive_pid(-68, 127);
  chassis.wait_drive();
  intake.move(30);

  // swing to outtake into corner
  chassis.set_turn_pid(-198, 127);
  chassis.wait_drive();

  // drive to drop off point
  chassis.set_drive_pid(-64, 127);
  chassis.wait_drive();

  // turn to outtake to corner
  chassis.set_turn_pid(-250, 127);
  chassis.wait_drive();

  // outtake
  intake.move(-127);
  pros::delay(250);
  intake.move(0);

  // turn to grab preload and corner ball
  chassis.set_turn_pid(-330, 127);
  chassis.wait_drive();

  // drive up
  chassis.set_drive_pid(40, 127);
  chassis.wait_drive();

/*
  // turn
  chassis.set_turn_pid(-420, 127);
  chassis.wait_drive();
*/
  // swing to grab corner ball
  chassis.set_swing_pid(RIGHT_SWING, -466, 127);
  chassis.wait_drive();


  // right wing extend
  rWing.set_value(true);
  // extend left wing to grab pre load
  lWing.set_value(true);
  // drive and clear
  chassis.set_drive_pid(25, 127);
  chassis.wait_drive();

  // close corner wing
  rWing.set_value(false);

    chassis.set_drive_pid(32, 127);
  chassis.wait_drive();

  // swing to slam in the three triballs
  chassis.set_swing_pid(RIGHT_SWING, -510, 127);
  chassis.wait_drive();

  // slam in triballs
  chassis.set_drive_pid(32, 127);
  chassis.wait_drive();

  // backup
  chassis.set_drive_pid(-15, 127);
  chassis.wait_drive();

  // double slam
  chassis.set_drive_pid(20, 127);
  chassis.wait_drive();

  // close left wing
  lWing.set_value(false);

  while (1)
  {
    pros::delay(100);
  }

  chassis.set_drive_pid(-10, 127);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, -239, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(50, 127);
  chassis.wait_drive();

  lWing.set_value(true);

  chassis.set_drive_pid(40, 127);
  chassis.wait_drive();
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
  // static pros::Motor cataleft(12, pros::E_MOTOR_GEARSET_36);
  // static pros::Motor cataright(-19, pros::E_MOTOR_GEARSET_36); // comment out with prog
  //  static pros::Motor cataright(9000, pros::E_MOTOR_GEARSET_36); // comment out with real bort

  static pros::Motor cata(3, pros::E_MOTOR_GEARSET_36);

  // static pros::Motor_Group cata({cataleft, cataright});
  // pros::Rotation rot(2);

  pros::ADIDigitalOut blocker('A');
  pros::ADIDigitalOut rWing('B');
  pros::ADIDigitalOut lWing('C');

  // rot.set_position(0);

  int turnChange = 23;

  cata.move(127);
  pros::delay(41300);

  // pros::delay(2600);
  cata.move(0);

  chassis.set_turn_pid(-110, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(65, 127);
  chassis.wait_until(65);

  chassis.set_turn_pid(-80, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(130, 127);
  chassis.wait_until(45);

  chassis.set_drive_pid(-130, 127);
  chassis.wait_until(-25);

  chassis.set_turn_pid(60, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(180, 127);
  chassis.wait_until(140);

  chassis.set_turn_pid(23, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(330, 127);
  chassis.wait_until(290);

  // Wings.set_value(false);

  chassis.set_turn_pid(turnChange + -50, 90);
  // chassis.set_drive_pid(120, 127);
  chassis.wait_drive();

  // Wings.set_value(true);

  chassis.set_drive_pid(220, 127);
  chassis.wait_until(150);

  // Wings.set_value(false);

  chassis.set_drive_pid(-220, 127);
  chassis.wait_until(-20);

  chassis.set_turn_pid(turnChange + -160, 90);
  // chassis.set_drive_pid(120, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(220, 127);
  chassis.wait_until(120);

  chassis.set_turn_pid(turnChange + -40, 127);
  chassis.wait_drive();

  lWing.set_value(true);
  rWing.set_value(true);

  chassis.set_drive_pid(200, 127);
  chassis.wait_until(123);

  lWing.set_value(false);
  rWing.set_value(false);

  chassis.set_turn_pid(turnChange + 0, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(-150, 127);
  chassis.wait_until(-100);

  chassis.set_turn_pid(turnChange + 90, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(-150, 127);
  chassis.wait_until(-100);

  chassis.set_turn_pid(turnChange + 20, 127);
  chassis.wait_drive();

  lWing.set_value(true);
  rWing.set_value(true);

  chassis.set_drive_pid(160, 127);
  chassis.wait_until(120);

  lWing.set_value(false);
  rWing.set_value(false);

  chassis.set_turn_pid(turnChange + -20, 127);
  chassis.wait_drive();

  chassis.set_drive_pid(-160, 127);
  chassis.wait_until(-100);

  chassis.set_turn_pid(turnChange + 0, 127);
  chassis.wait_drive();

  lWing.set_value(true);
  rWing.set_value(true);

  chassis.set_drive_pid(160, 127);
  chassis.wait_until(120);

  while (1)
  {
    pros::delay(100);
  }
}