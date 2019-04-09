#ifndef FLIPPER_H_
#define FLIPPER_H_

#include "main.h"
#include "drive.h"

//use positive voltage for flipper to go down

void flipper_task(void*ingore); //using task 2 now
extern pros::task_t flipper_task_init;

extern bool flipper_down;
extern bool button_pressed;

#define UP 0
#define DOWN 1

//void flipper(float give_dist_before_flipper, int give_flipper_position, int give_flipper_speed);
void flipper_backup(float give_dist_before_flipper, bool give_flipper_down, int give_millis_counter_flipper = 200);


//new flipper code
//variables explained in .cpp file
  extern bool flipper_move_to_position;
  extern int flipper_position;
  extern bool flipper_timed;
  extern int resting_position;
  extern int extended_position;
  extern bool flipper_position_move; //only for auton

  extern int millis_counter_flipper;
  extern float dist_before_flipper;
  extern int flipper_speed;
  extern int flipper_position_auton;

  #define REST 1050 //1000
  #define EXTEND 2850
  #define BRAKE 360

  extern pid_terms flipper_pid;

  void flipper(bool give_flipper_move_to_position, int give_flipper_position = BRAKE, float give_dist_before_flipper = 0, int give_flipper_speed = 0, int give_resting_position = REST, int give_extended_position = EXTEND);
  void flipper_task2(void* ignore);
  void auto_flipper(float give_dist_before_flipper, int give_flipper_position_auton = REST, bool give_flipper_position_move = true, int give_flipper_speed = 100, int give_millis_counter_flipper = 200);


#endif
