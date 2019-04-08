#include "main.h"
#include "intake.h"
#include "allinit.h"

using namespace pros::literals;

//globals init
int time_for_intake;
int speed_for_intake;

bool intake_flag;
float dist_before_intake;

bool outtake;
bool light_sen_flag;

unsigned int millis_counter_intake;

pros::task_t intake_task_init;


void intake_task(void*ignore){
speed_for_intake = 0;
time_for_intake = 0;
millis_counter_intake = 0;
outtake = false;

  while(true){

    //timed intake
      while(pros::competition::is_autonomous()  && intake_flag == false && outtake == false){

            while((pros::millis() < millis_counter_intake)  &&  pros::competition::is_autonomous()){
                intake_motor.move(speed_for_intake);
                pros::delay(5);
            }
            if(pros::millis() > millis_counter_intake){
              intake_motor.move(0);
            }
          }

    //start intaking after reaching a certain distance
      while(pros::competition::is_autonomous() && intake_flag == true && outtake == false){

           while(fabs((dist_before_intake/(4*pi) * RPM200_GEARSET)) >=
              fabs((drive_left_f.get_position() + drive_right_f.get_position() + drive_left_b.get_position() +
              drive_right_b.get_position())/4)  && pros::competition::is_autonomous()){

                pros::delay(5);
            }

          while (intake_flag == true  &&  pros::competition::is_autonomous()){
            intake_motor.move(120); //75
            //printf("inside loop");
            if(limit_switch.get_value() == 1){
              intake_flag = false;
            }
            pros::delay(5);
          }
      }

    //outtake
      while(pros::competition::is_autonomous() && outtake == true){

            while(fabs((dist_before_intake/(4*pi) * RPM200_GEARSET)) >=
            fabs((drive_left_f.get_position() + drive_right_f.get_position() + drive_left_b.get_position() +
                  drive_right_b.get_position())/4)  && pros::competition::is_autonomous()){

               pros::delay(5);
               millis_counter_intake = 0;
               millis_counter_intake = pros::millis() + time_for_intake;
             }

            while(((pros::millis() < millis_counter_intake) &&  pros::competition::is_autonomous())){
                intake_motor.move(speed_for_intake);
                pros::delay(5);

            if(light_sen_flag == true){
              if((pros::millis() > millis_counter_intake) || light.get_value() < LIGHT_SEN_THRESHOLD){
                intake_motor.move(0);
                outtake = false;
              }
            }

          }
          if((pros::millis() > millis_counter_intake)){
            intake_motor.move(0);
            outtake = false;
          }
        }


  //driver control
    while(!pros::competition::is_autonomous()){

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 1 && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1){
          intake_motor.move(-120);
        }

        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 1 && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 0){
          intake_motor.move(120);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 0 && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1){
          intake_motor.move(-60);
        }

        else{
          intake_motor.move(0);
        }
        pros::delay(20);
      }
    pros::delay(5);
  }
}


//timed intake and outtake with preset speed
void intake (int give_speed_for_intake, int give_time_for_intake){

  outtake = false;

  speed_for_intake = 0;
  time_for_intake = 0;

  speed_for_intake = give_speed_for_intake;
  time_for_intake = give_time_for_intake;
  millis_counter_intake = pros::millis() + give_time_for_intake;
}


//used to limit ball position close to flywheel
void intake_limit(bool give_intake_flag, float give_dist_before_intake){

  dist_before_intake = give_dist_before_intake;
  outtake = false;

  if(give_intake_flag == true){
    intake_flag = true;
  }
  else{
    intake_flag = false;
  }

}


//used for outtaking or flipping cap using light sensor or travelling a dist before flipping
void intake_flip(bool give_outtake, int give_speed_for_intake, int give_time_for_intake, float give_dist_before_intake, float give_light_sen_flag){

  speed_for_intake = 0;
  time_for_intake = 0;
  dist_before_intake = give_dist_before_intake;

  if(give_outtake == true){
    outtake = true;
  }

  else{
    outtake = false;
  }

  if (give_light_sen_flag == true){
    light_sen_flag = true;
  }

  else{
    light_sen_flag = false;
  }

  speed_for_intake = give_speed_for_intake;
  time_for_intake = give_time_for_intake;
  millis_counter_intake = pros::millis() + give_time_for_intake;

}


void wait_for_intake(void){
int exit_fail_safe = 0;

  while(intake_flag == true){
    pros::delay(20);
    exit_fail_safe++;

    if(exit_fail_safe > (3000/20)){  //if 3 seconds pass by, exit the limit switch loop
      intake_flag = false;
      break;
    }
  }
}
