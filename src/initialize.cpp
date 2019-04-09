#include "main.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"
#include "drive.h"
#include "slew_rate.h"
#include "intake.h"
#include "flywheel.h"
#include "lift.h"
#include "allinit.h"
#include "flipper.h"
#include "lcd.h"


using namespace pros::literals;


void initialize() {

  pros::lcd::initialize();

	//pros::lcd::print(7, "initialized");

  pros::ADIGyro gyro (GYRO_PORT, 0.9510); //0.9456 //0.9704 //0.97 //0/96 //0.9550

//WAS NOT COMMENTED FOR BCIT
  // pros::ADIDigitalIn limit_switch (LIMIT_SWITCH_PORT);
  // pros::ADIAnalogIn light (LIGHT_SENSOR_PORT);
  // pros::ADIAnalogIn light_flywheel (LIGHT_SENSOR_FLYWHEEL_PORT);
  // pros::ADIAnalogIn pot (POT_PORT);


  pros::task_t intake_task_init = pros::c::task_create(intake_task, (void*)NULL, TASK_PRIORITY_DEFAULT,
                              TASK_STACK_DEPTH_DEFAULT, "INTAKE TASK");

  pros::task_t flywheel_task_init = pros::c::task_create(flywheel_task, (void*)NULL, TASK_PRIORITY_DEFAULT,
                              TASK_STACK_DEPTH_DEFAULT, "FLYWHEEL TASK");

  pros::task_t lift_task_init = pros::c::task_create(lift_task, (void*)NULL, TASK_PRIORITY_DEFAULT,
                              TASK_STACK_DEPTH_DEFAULT, "LIFT TASK");

	pros::task_t flipper_task_init = pros::c::task_create(flipper_task, (void*)NULL, TASK_PRIORITY_DEFAULT,
                              TASK_STACK_DEPTH_DEFAULT, "FLIPPER TASK");

  //flipper_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //flywheel_f.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  auton = 0;
  red = 0;
  blue = 0;
  max_auton_limit = 6;


}


void disabled() {}


void competition_initialize() { //testing this thing

  // while(pros::competition::is_disabled()){
  //   pros::lcd::register_btn2_cb(on_right_button);
  //   pros::lcd::register_btn0_cb(on_left_button);
	//
  //   pros::delay(5);
  // }
}
