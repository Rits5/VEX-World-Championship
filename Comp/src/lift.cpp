#include "main.h"
#include "lift.h"
#include "allinit.h"

using namespace pros::literals;

//globals init
  pros::task_t lift_task_init;

  int target_height;
  int lift_delay;
  int lift_speed_up;
  int lift_speed_down;
  int max_lift_speed;
  float Kp_lift;
  int lift_switch_mode;


void lift_task(void* ignore){

    while(true){

      float error;
      Kp_lift = 0.5;
      float final_power;
      max_lift_speed = 120;
      target_height = 0;
      lift_delay = 0;
      int rest_height = -30;

      int count = 0;

      while(!pros::competition::is_autonomous() &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == 0 && master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 0 &&
        lift_switch_mode == 0){

          if(target_height < 1200){lift_delay = 800;}
          else{lift_delay = 600;}

          error = target_height - lift_motor.get_position();
          final_power = error*Kp_lift;

          if(final_power > max_lift_speed){final_power = max_lift_speed;}
          else if(final_power < -max_lift_speed){final_power = -max_lift_speed;}

          lift_motor.move(final_power);

          //NOT COMMENTED FOR BCIT
          //if (fabs(final_power) < 5){final_power = 0;} //was < 30 for BCIT

            if( (((lift_motor.get_position() - 30) < target_height < (lift_motor.get_position() + 30)) )
             && target_height != rest_height){

              count++;
            }
            if(count > (lift_delay/20)){
              target_height = rest_height;
              count = 0;
            }

          pros::delay(20);

        }

      pros::delay(20);
    }

}
