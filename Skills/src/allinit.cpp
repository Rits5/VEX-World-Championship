#include "main.h"
#include "allinit.h"
#include "intake.h"
#include "flywheel.h"
#include "flipper.h"
#include "lift.h"

using namespace pros::literals;

//motors

 pros::Motor drive_left_b (DRIVE_LEFT_B_PORT, pros::E_MOTOR_GEARSET_18, NOT_REVERSE, pros::E_MOTOR_ENCODER_COUNTS);
 pros::Motor drive_left_f (DRIVE_LEFT_F_PORT, pros::E_MOTOR_GEARSET_18, NOT_REVERSE, pros::E_MOTOR_ENCODER_COUNTS);
 pros::Motor drive_right_b (DRIVE_RIGHT_B_PORT, pros::E_MOTOR_GEARSET_18, REVERSE, pros::E_MOTOR_ENCODER_COUNTS);
 pros::Motor drive_right_f (DRIVE_RIGHT_F_PORT, pros::E_MOTOR_GEARSET_18, REVERSE, pros::E_MOTOR_ENCODER_COUNTS);

 pros::Motor flywheel_f (FLYWHEEL_FRONT_PORT, pros::E_MOTOR_GEARSET_06, NOT_REVERSE, pros::E_MOTOR_ENCODER_COUNTS);

 pros::Motor intake_motor (INTAKE_PORT, pros::E_MOTOR_GEARSET_18, NOT_REVERSE, pros::E_MOTOR_ENCODER_COUNTS);
 pros::Motor lift_motor (LIFT_PORT, pros::E_MOTOR_GEARSET_18, REVERSE, pros::E_MOTOR_ENCODER_COUNTS);
 pros::Motor flipper_motor (FLIPPER_PORT, pros::E_MOTOR_GEARSET_18, NOT_REVERSE, pros::E_MOTOR_ENCODER_COUNTS);

 pros::Motor test_motor (TEST_PORT_MOTOR, pros::E_MOTOR_GEARSET_18, REVERSE, pros::E_MOTOR_ENCODER_COUNTS);



//sensors

  pros::ADIDigitalIn limit_switch (LIMIT_SWITCH_PORT);
  pros::ADIAnalogIn light (LIGHT_SENSOR_PORT);
  pros::ADIAnalogIn light_flywheel (LIGHT_SENSOR_FLYWHEEL_PORT);
  pros::ADIAnalogIn pot (POT_PORT);

  pros::ADIEncoder encoder_left(LEFT_ENC_PORT_TOP, LEFT_ENC_PORT_BOT, NOT_REVERSE);
  pros::ADIEncoder encoder_right(RIGHT_ENC_PORT_TOP, RIGHT_ENC_PORT_BOT, REVERSE);


//controller

  pros::Controller master (pros::E_CONTROLLER_MASTER);
