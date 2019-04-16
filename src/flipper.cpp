#include "main.h"
#include "flipper.h"
#include "allinit.h"
#include "motor.h"
#include "drive.h"
#include "flywheel.h"

using namespace pros::literals;

float dist_before_flipper;
int flipper_speed;
pros::task_t flipper_task_init;

bool flipper_down = true;
bool button_pressed;
int millis_counter_flipper;

//new flipper code
bool flipper_move_to_position; //drive control only, move flipper using ticks/pot values
int flipper_position; //current flipper position in ticks or pot value
bool flipper_timed; //true if timed, false if not timed
int resting_position; //up position
int extended_position; //down position
bool flipper_position_move; //only for auton, for movements based on encoder ticks or pot values
int flipper_position_auton;

pid_terms flipper_pid;


void flipper_task(void* ignore){
  float Kp = 0.13; //0.15
  float Kd = 0.15; //0.15
  float Ki = 0.005;
  float final_power;

  // float Kp = 0.085; //0.085 for BCIT
  // float Kd = 0.60; //0.8 for BCIT
  // float Ki = 0.005; //0.005 for BCIT

  //pid_terms flipper;
  //USED FOR BCIT //pid_init(&flipper_pid, Kp, Ki, Kd, 10, 3000);  //0.06kp and 0.2kd
  pid_init(&flipper_pid, Kp, Ki, Kd, 10, 500); //30 and 500

  while(true){

    while(!pros::competition::is_disabled()){
    //while(pros::competition::is_autonomous()){
      int millis_flipper = millis_counter_flipper; //stores value of millis_counter_flipper given by give_millis_counter_flipper from the auto_flipper function
      millis_counter_flipper = pros::millis() + millis_flipper;

        while(fabs((dist_before_flipper/(4*pi) * RPM200_GEARSET)) >=
           fabs((drive_left_f.get_position() + drive_right_f.get_position() + drive_left_b.get_position() +
           drive_right_b.get_position())/4) && fabs(dist_before_flipper) > 0.1 && !pros::competition::is_disabled()){

             pros::delay(10);
             millis_counter_flipper = pros::millis() + millis_flipper;
        }

          if(flipper_move_to_position == true){

              if(pot.get_value() < REST){
                flipper_pid.integral = 0;
              }

              // if(pot.get_value() > 2000){
              //   pid_init(&flipper_pid, 0.09, Ki, Kd, 20, 300);
              // }
              //
              // if(pot.get_value() < 2000){
              //   pid_init(&flipper_pid, Kp, Ki, Kd, 20, 300);
              // }

            float calculated_power = pid_cal(&flipper_pid, flipper_position, pot.get_value());

            // if(pros::competition::is_autonomous()){final_power = power_limit(80, calculated_power);}
            // if(!pros::competition::is_autonomous()){final_power = power_limit(100, calculated_power);}
            float final_power = power_limit(80, calculated_power);

            flipper_motor.move(final_power);
          }

          else if(flipper_move_to_position == false){

            flipper_motor.move(flipper_speed); // voltage not rpm
            pros::delay(10);
          }

        pros::delay(10);
      }

      pros::delay(10);
    }

}


void flipper(bool give_flipper_move_to_position, int give_flipper_position, float give_dist_before_flipper, int give_flipper_speed, bool give_timed_flywheel, int give_resting_position, int give_extended_position){

  timed_flywheel = give_timed_flywheel;

  flipper_position = give_flipper_position;
  flipper_move_to_position = give_flipper_move_to_position;
  dist_before_flipper = give_dist_before_flipper;
  //flipper_timed = give_flipper_timed;
  flipper_speed = give_flipper_speed;
  resting_position = give_resting_position;
  extended_position = give_extended_position;

  //timed_flywheel = true; ///set flywheel variable to true so it doesn't interfere with the flipper function

  int mid_pos = 1500;

  if(give_flipper_position == BRAKE){

    if(pot.get_value() < mid_pos && flipper_move_to_position == true){
      flipper_position = extended_position;
      //flipper_move_to_position = false;
      //flipper_timed = false;
    }

    else if(pot.get_value() > mid_pos && flipper_move_to_position == true){
      flipper_position = resting_position;
      //flipper_timed = false;
    }
  }
}


void auto_flipper(float give_dist_before_flipper, int give_flipper_position_auton, bool give_flipper_position_move, int give_flipper_speed, int give_millis_counter_flipper){

  dist_before_flipper = give_dist_before_flipper;
  flipper_position_move = give_flipper_position_move;
  millis_counter_flipper = give_millis_counter_flipper;
  flipper_speed = give_flipper_speed;
  flipper_position_auton = give_flipper_position_auton;
  //brake_timer = 0;
}
