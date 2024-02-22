#include "main.h"
#include "variables.hpp"
#include <vector>

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {-7, 9, -8} // real
    //{-9, -20} // old

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {2, -5, 6} // real
    //{9, 10} // old

    // IMU Port
    ,
    19

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    4.125

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
    // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
    ,
    0.57143

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  // Print our branding over your terminal :D
  ez::print_ez_template();

  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1);                     // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0);                   // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  default_constants();                               // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults();                         // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({

      //Auton("Example Drive\n\nDrive forward and come back.", drive_example),
      //Auton("Example Turn\n\nTurn 3 times.", turn_example),
      // Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
      // Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      // Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
      // Auton("Combine all 3 movements", combining_movements),
      // Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
      //Auton("Runs defensive In-Game Autonomous.", defAuton),
      Auton("Runs offensive In-Game Autonomous.", offAuton),
      //Auton("Runs Skills Autonomous.", skillsAuton),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
  chassis.reset_pid_targets();               // Resets PID targets to 0
  chassis.reset_gyro();                      // Reset gyro position to 0
  chassis.reset_drive_sensor();              // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
  // old cata variables
  /*
  bool cataOff = true;
  long launchTrack = pros::millis();
  bool launchFlip = true;
  int_least8_t oldRot;
  rot.set_position(0);
  */

  // pneumatic switch booleans
  bool blockerSwitch = false;
  bool lWingSwitch = false;
  bool rWingSwitch = false;

  // cata variables
  int cataSpeed = 127;
  const int cataAdjust = 6;

  // drive variables
  const float dBand = 5;

  // stick variables to calulate speeds with curve
  double power;
  double powerC;
  double turn;
  double turnC;

  // curve out of 10
  // modelled after https://www.desmos.com/calculator/toufp2r8qb
  int pCurve = 0;   // curve for fwd/back
  int tCurve = 0.3; // curve for turn

  double e = exp(1); // Euler's constant

  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  while (true)
  {
    // chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

    // driver code
    //  variable for # of motors per side
    const int sideMotors = 3;
    // get stick values
    if (master.get_analog(ANALOG_LEFT_Y) > dBand || master.get_analog(ANALOG_LEFT_Y) < -dBand)
    {
      // calculates power curve for joystick
      power = master.get_analog(ANALOG_LEFT_Y);
      powerC = power * (pow(e, -(pCurve / 10)) + pow(e, (abs(power) - 127) / 10) * (1 - pow(e, -(pCurve / 10))));
    }
    else
    {
      powerC = 0;
    }

    // gets turn value and calculates curve
    if (master.get_analog(ANALOG_RIGHT_X) > dBand || master.get_analog(ANALOG_RIGHT_X) < -dBand)
    {
      turn = -master.get_analog(ANALOG_RIGHT_X);
      turnC = turn * (pow(e, -(tCurve / 10)) + pow(e, (abs(turn) - 127) / 10) * (1 - pow(e, -(tCurve / 10))));
    }
    else
    {
      turnC = 0;
    }

    for (int i = 0; i < sideMotors; i++)
    {
      chassis.left_motors[i].move(powerC - turnC);
      chassis.right_motors[i].move(powerC + turnC);
    }

    // intake code
    if (master.get_digital(DIGITAL_R2)) // outtake
    {
      intake.move(-127);
    }
    else if (master.get_digital(DIGITAL_L2)) // intake
    {
      intake.move(127);
    }
    else
    {
      intake.move(0);
    }

    // cata speed adjustment code
    if (master.get_digital_new_press(DIGITAL_UP))
    {
      cataSpeed += cataAdjust;
    }
    if (master.get_digital_new_press(DIGITAL_DOWN))
    {
      cataSpeed -= cataAdjust;
    }

    //caps catapult speed at a certain range
    if (cataSpeed > 127)
    {
      cataSpeed = 127;
    }
    if (cataSpeed < 70)
    {
      cataSpeed = 70;
    }

    // new cata code
    if (master.get_digital(DIGITAL_B))
    {
      cata.move(cataSpeed);
    }
    else
    {
      cata.move(0);
    }

    // toggleable blocker

    // wings
    

    //wings control only 1 solenoid
    if (master.get_digital_new_press(DIGITAL_A))
    {
      if (lWingSwitch == false)
      {
        lWingSwitch = true;
        lWing.set_value(true);
      }
      else
      {
        lWingSwitch = false;
        lWing.set_value(false);
      }
    }

    

    // master.rumble(".");

    // ez::print_to_screen("Rotation Angle: " + std::to_string(rotDeg), 3);
    // master.set_text(1, 1, "Rot: " + std::to_string(rotDeg));
    ez::print_to_screen("CataSpeed = " + std::to_string(cataSpeed), 3);
    ez::print_to_screen("Linear Speed: " + std::to_string(power), 4);
    ez::print_to_screen("Drive Motor Temp: " + std::to_string(static_cast<int>(chassis.left_motors[0].get_temperature())), 2);
    //  master.set_text(1, 1, std::to_string(static_cast<int>(chassis.left_motors[0].get_temperature())) + "power = " + std::to_string(power));


    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
