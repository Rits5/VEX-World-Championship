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

	Yf = 1.1;
	switch_mode_flywheel_top = 0;
	switch_mode_flywheel_bottom = 0;

	pros::ADIGyro gyro (GYRO_PORT, 0.967742);
	//gyro.reset();
//  pros::ADIUltrasonic sonar (PORT_ECHO, PORT_PING);

	//pros::ADIEncoder encoder_left(LEFT_ENC_PORT_TOP, LEFT_ENC_PORT_BOT, REVERSE);
	//pros::ADIEncoder encoder_right(RIGHT_ENC_PORT_TOP, RIGHT_ENC_PORT_BOT, REVERSE);


	//flipper_motor.move_absolute(50, 200); //move flipper initially
	flipper(true, REST);

	  pros::lcd::register_btn2_cb(on_right_button);
	  pros::lcd::register_btn0_cb(on_left_button);

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
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == 1){
			double_shot_setup(570, 406, 128); //front double shot //146
			ball_shot = false; //setting to false to reset sensor into thinking ball has not been shot yet
			sensor_double_shot = true; //turns on sensor based double shot mode
			double_shot = 1;
			brake_timer = 0;
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1){
			double_shot_setup(520, 427, 81); //back double shot //was 86 //428 power
			ball_shot = false; //setting to false to reset sensor into thinking ball has not been shot yet
			sensor_double_shot = true; //turns on sensor based double shot mode
			double_shot = 1;
			brake_timer = 0;
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == 1){
			double_shot_setup(525, 470, 0); //back double shot //was 86 //467 power
			ball_shot = false; //setting to false to reset sensor into thinking ball has not been shot yet
			sensor_double_shot = true; //turns on sensor based double shot mode
			double_shot = 1;
			brake_timer = 0;
		}

	//flipper controls
		//else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) == 1){
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1){
			button_pressed = true;
			double_shot = 0; //disable double shot if its on
			ball_shot = false; //setting to false to reset sensor into thinking ball has not been shot yet
			sensor_double_shot = false; //turns off sensor based double shot mode

			//testing new flipper code
			// flipper_counter++;
			// if(flipper_counter > 1){
			// 	flipper_counter = 0;
			// }
			flipper(true);

		}



	//	dx = sonar.get_value();

		pros::lcd::print(6, "limit switch %d", limit_switch.get_value());
		pros::lcd::print(5, "light sen %d", light.get_value());

		printf("gyro %f\n", gyro.get_value());
		pros::lcd::print(3, "gyro: %f", gyro.get_value());
		pros::lcd::print(4, "flywheel light sensor %d", light_flywheel.get_value());

		//printf("left encoder: %d\n", encoder_left.get_value());
		//printf("right encoder: %d\n", encoder_right.get_value());
		//printf("pot value %d\n", pot.get_value());

		pros::lcd::print(7, "left encoder %d", encoder_left.get_value());

	//	printf("light_sen_flywheel: %d\n", light_flywheel.get_value());

		pros::delay(5);

		//printf("flipper position: %f\n", flipper_motor.get_position());
		//printf("controller button pressed: %d\n", master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN));

	}
}
