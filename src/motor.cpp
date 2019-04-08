#include "main.h"
#include "allinit.h"

using namespace pros::literals;

void drive_set(int speed){
  drive_left_f.move(speed);
  drive_left_b.move(speed);
  drive_right_f.move(speed);
  drive_right_b.move(speed);
}

void turn_set(int speed){
  drive_left_f.move(speed);
  drive_left_b.move(speed);
  drive_right_f.move(-speed);
  drive_right_b.move(-speed);
}

void left_drive_set(int speed){
  drive_left_f.move(speed);
  drive_left_b.move(speed);
}

void right_drive_set(int speed){
  drive_right_f.move(speed);
  drive_right_b.move(speed);
}

void flywheel_set(int rpm){
  flywheel_f.move_velocity(rpm);
}
