#ifndef FLYWHEEL_H_
#define FLYWHEEL_H_

#include "main.h"

void flywheel_task(void* ignore);
extern int rpm_task;
extern int timer_flywheel_task;
extern int flywheel_slew;
extern int flywheel_commanded;

extern pros::task_t flywheel_task_init;
//void flywheel(int give_rpm_task, int give_timer_flywheel_task);
void flywheel_flag(int rpm, int timer, float distance);

extern unsigned int millis_counter_flywheel;

extern int target_rpm;
extern double Yf;
extern double dx;
extern int flywheel_avg;
extern double dx_global;

extern int double_shot;
extern bool speed_ramp_up;
extern bool change_flywheel_speed;
extern bool fast_to_slow;
extern bool slow_to_fast_close;
extern bool slow_to_fast_far;
extern int delay_until_set_speed;
extern bool delay_done;
extern bool failsafe_flywheel;

double calc(void);

extern float distance_flag;

extern int switch_mode_flywheel_bottom;
extern int switch_mode_flywheel_top;


//sensor based double shot code
  #define LIGHT_FLYWHEEL_THRESHOLD 1750 //1800
  extern bool ball_shot;
  extern bool sensor_double_shot;
  extern int brake_timer;

  extern bool timed_flywheel;
  extern bool run_flywheel;

  void double_shot_setup(int give_top_flag_speed, int give_middle_flag_speed, int give_flywheel_brake);

  void auto_double_shot(int give_top_flag_speed, int give_middle_flag_speed, int give_flywheel_brake, bool give_timed_flywheel = false);
  void flywheel(int give_rpm_task, int give_timer_flywheel_task, bool give_timed_flywheel = true);

#endif
