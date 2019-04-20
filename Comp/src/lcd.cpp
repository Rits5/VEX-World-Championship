#include "main.h"
#include "lcd.h"


int auton;
int red;
int blue;
int max_auton_limit;


void auton_limiter(int counter, bool colour){

	if (colour == true){ //for red
		if(counter > max_auton_limit){
			red = 1;
		}
	}

	if (colour == false){ //for blue
		if(counter > max_auton_limit){
			blue = 1;
		}
	}

}


void red_selector(int counter){
	switch (counter) {

		case 1:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "red front park");
		break;

		case 2:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "red front cap");
		break;

		case 3:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "red back park quick");
		break;

		case 4:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "red back counter cap");
		break;

    case 5:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "red back counter park");
		break;

    case 6:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "skills");
		break;
	}

}


void blue_selector(int counter){
	switch (counter) {

		case 1:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "blue front park");
		break;

		case 2:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "blue front cap");
		break;

		case 3:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "blue back park quick");
		break;

		case 4:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "blue back counter cap");
		break;

    case 5:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "blue back counter park");
		break;

    case 6:
		pros::lcd::clear_line(0);
		pros::lcd::print(0, "Testing");
		break;
	}

}


void on_right_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		blue = 0;
		red++;
		auton_limiter(red, RED);
		red_selector(red);
	//	pros::lcd::print(2, "red value: %d", red);
	}
	else {
		blue = 0;
		red++;
		auton_limiter(red, RED);
		red_selector(red);
	//	pros::lcd::print(2, "red value: %d", red);
	}
}

void on_left_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		red = 0;
		blue++;
		auton_limiter(blue, BLUE);
		blue_selector(blue);
	//	pros::lcd::print(2, "blue value: %d", blue);
	}
	else {
		red = 0;
		blue++;
		auton_limiter(blue, BLUE);
		blue_selector(blue);
	//	pros::lcd::print(2, "blue value: %d", blue);
	}
}
