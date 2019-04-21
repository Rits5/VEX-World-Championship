#include "main.h"
#include "flywheel.h"
#include "motor.h"
#include "allinit.h"
#include "flipper.h"

using namespace pros::literals;

//globals init
  int rpm_task;
  int timer_flywheel_task;
  int flywheel_slew;
  int flywheel_commanded;

  unsigned int millis_counter_flywheel;

  int target_rpm;
  double Yf;
  double dx;
  int flywheel_avg;
  double dx_global;

  int double_shot;
  bool speed_ramp_up;
  bool change_flywheel_speed;
  bool fast_to_slow;
  bool slow_to_fast_close;
  bool slow_to_fast_far;
  int delay_until_set_speed;
  bool delay_done;

  float distance_flag;

  int switch_mode_flywheel_bottom;
  int switch_mode_flywheel_top;

  pros::task_t flywheel_task_init;

//sensor based double shot code
  bool ball_shot;
  bool sensor_double_shot;
  int brake_timer;
  int top_flag_speed;
  int middle_flag_speed;
  int flywheel_brake;

  bool timed_flywheel;
  bool run_flywheel;



void flywheel_task(void*ignore){
  while(true){

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1){
      run_flywheel = true;
      flywheel_set(0);
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 0 && run_flywheel == true){

      flywheel_slew += 30;
      if(flywheel_slew > flywheel_commanded){flywheel_slew = flywheel_commanded;}

      flywheel_set(flywheel_slew);
    }

    else if(run_flywheel == false){
      flywheel_set(0);
    }

  pros::delay(5);
}
}





void flywheel(int give_rpm_task, int give_timer_flywheel_task, bool give_timed_flywheel){

  rpm_task = 0;
  timer_flywheel_task = 0;

  rpm_task = give_rpm_task;
  timer_flywheel_task = give_timer_flywheel_task;
  millis_counter_flywheel = pros::millis() + give_timer_flywheel_task;

  timed_flywheel = give_timed_flywheel;
}

void auto_double_shot(int give_top_flag_speed, int give_middle_flag_speed, int give_flywheel_brake, bool give_timed_flywheel){

  top_flag_speed = give_top_flag_speed;
  middle_flag_speed = give_middle_flag_speed;
  flywheel_brake = give_flywheel_brake;

  timed_flywheel = give_timed_flywheel;
  ball_shot = false;
  brake_timer = 0;
  timed_flywheel = false;
}

void double_shot_setup(int give_top_flag_speed, int give_middle_flag_speed, int give_flywheel_brake){ //for driver control

  top_flag_speed = give_top_flag_speed;
  middle_flag_speed = give_middle_flag_speed;
  flywheel_brake = give_flywheel_brake;
}


double calc(void){

const double Hbot = 0.33;
double Vt;  //calculated speed

// x part of code
const double Vix = 1/sqrt(2); //cos(45)
int t;
const int ax = 0;

// y part of code
const double Yi = Hbot;
const double Viy = 1/sqrt(2);  //sin(45)
const float ay = -9.81;   //gravity in m/s2

dx_global = (dx/1000) - 0.3 + 0.0625;
t = dx/(Vix); //didn't use this term to make calculation easier to read for direct calc

Vt = (dx_global)/ (Vix*(sqrt((2/ay)*(Yf - Yi - Viy*((dx_global)/Vix)))));


return Vt;
}
