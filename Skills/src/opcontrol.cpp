#include "main.h"
#include "motor.h"
#include "drive.h"
#include "flywheel.h"
#include "intake.h"
#include "slew_rate.h"
#include "lift.h"
#include "allinit.h"
#include "flipper.h"
#include "lcd.h"

using namespace pros::literals;

void opcontrol() {

	pros::lcd::initialize();

	flipper(true, REST);

	while (true) {

		if (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < 10){
			left_drive_set(0);
		}
		else{
			left_drive_set(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*0.95);
		}

		if (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) < 10){
			right_drive_set(0);
		}
		else{
			right_drive_set(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)*0.95);
		}


				if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) == 1){
					lift_switch_mode = 0;
					target_height = 1450;
				}
				else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) == 1){
					lift_switch_mode = 0;
					target_height = 900;
				}

				else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 1 &&
					master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == 0){

					lift_switch_mode = 1;
					lift_motor.move(120);
				}
				else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == 1 &&
					master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 0){

					lift_switch_mode = 1;
					lift_motor.move(-120);
				}
				else if(lift_switch_mode == 1 &&
					master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 0 &&
					master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == 0){

					lift_motor.move(0);
				}


	//double shot with light sensor
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == 1 || master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1
					|| master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == 1 || master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 1){

			run_flywheel = false;
		}

		pros::lcd::print(0, "pot value: %d", pot.get_value());

	}
}
