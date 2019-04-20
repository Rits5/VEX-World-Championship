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



void flywheel_task(void*ignore){
  while(true){

//autonomous part
    while(pros::competition::is_autonomous()){
      while((pros::millis() < millis_counter_flywheel) && pros::competition::is_autonomous() && timed_flywheel == true){
        flywheel_set(rpm_task);
        pros::delay(5);
      }
  //  flywheel_set(rpm_task); was not commented for provincials


  //double shot with light sensor ----- top to middle flag
    while(ball_shot == false && timed_flywheel == false && pros::competition::is_autonomous()){ //run flywheel while the sensor is not detecting a ball

        //auto_flipper(true, REST);
        flipper(true, REST, 0, 0, false);
        flywheel_set(top_flag_speed); //540

          if(light_flywheel.get_value() < LIGHT_FLYWHEEL_THRESHOLD){
            ball_shot = true;
          }
          pros::delay(5);
      }

        if(ball_shot == true){
          flywheel_set(middle_flag_speed); //430
          if(brake_timer <= (flywheel_brake/2)){   //100
            brake_timer++;
            flipper(false, 0, 0, -80, false);
            //auto_flipper(0, 0, false, -80);
          }

        else {
          //flipper_motor.move_absolute(50, 200);
          flipper(true, REST);
        }
      }

    pros::delay(2);
  }

//driver control part
    while(!pros::competition::is_autonomous()){

      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1){

          double_shot = 0; //disable double shot

          //sensor based double shot variables
          ball_shot = false; //setting to false to reset sensor into thinking ball has not been shot yet
          sensor_double_shot = false; //turns off sensor based double shot mode

          if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1){

            flywheel_commanded = 570; //190
            double_shot = 0;
          }

            while((master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 1) && !pros::competition::is_autonomous()){

              flywheel_slew += 30;
              if(flywheel_slew > flywheel_commanded){flywheel_slew = flywheel_commanded;}

              flywheel_set(flywheel_slew);
              pros::delay(5);
            }
          }



  //double shot - driver control
    else if (double_shot == 1 && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 0 && !pros::competition::is_autonomous()){
      //flipper_motor.move_absolute(50, 200);
      flipper(true, REST);

      //double shot with light sensor ----- top to middle flag
      while (sensor_double_shot == true && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 0 && !pros::competition::is_autonomous()){

        while(ball_shot == false && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 0 && double_shot == 1 && !pros::competition::is_autonomous()){ //run flywheel while the sensor is not detecting a ball
            flywheel_set(top_flag_speed); //540

            if(light_flywheel.get_value() < LIGHT_FLYWHEEL_THRESHOLD){
              ball_shot = true;
            }
            pros::delay(5);
          }

          if(ball_shot == true){
            flywheel_set(middle_flag_speed); //430

            if(brake_timer <= (flywheel_brake/2)){   //100
              brake_timer++;
              //flipper_motor.move(-80);
              flipper(false, 0, 0, -80);
            }

            else{
              //flipper_motor.move_absolute(50, 200);
              flipper(true, REST);
            }
          }

        pros::delay(2);
      }
    }


      //else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 0 && switch_mode_flywheel_top == 0 && switch_mode_flywheel_bottom == 0){
      else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) == 0){
          flywheel_set(570);
          // flywheel_slew -= 50;
          // flywheel_commanded = 0;
          // if(flywheel_slew < 0){flywheel_slew = 0;}
      }
      pros::delay(5);
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
