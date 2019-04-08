#include "main.h"
#include "slew_rate.h"
#include "allinit.h"

//globals init
  int islew;


int motor_error[21] = {0};
int motor_target[21] = {0};
int motor_power[21] = {0};
int motor_port[21] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};


int slewcap(int value, int cap){
  if (value > cap){
    return cap;}
  else if(value < -cap){
    return -cap;}
  else {
    return value;}
  }


void slew_rate_task (void*ignore){

    islew = 0;

    while(true){
        pros::delay(20);

      if (pros::competition::is_disabled()){break;}

      else if (!pros::competition::is_disabled()){

          while(islew <= 9){

            motor_error[islew] = motor_target[islew] - motor_power[islew];
            motor_power[islew] = motor_power[islew] + slewcap(motor_error[islew], DEFAULT_SLEW);

            //motor_move(motor_port[islew], motor_power[islew]);

          //  motor_port[islew].move(motor_power[islew]);

            pros::Motor *motors[MAX_NUM_MOTORS];
            motors[motor_port[islew]-1] = new pros::Motor(motor_port[islew]);
            motors[motor_port[islew]-1]->move(motor_power[islew]);
          //  delete motors[motorNumber-1];

            islew++;
          }

      }
  }
  //task_delete(slewtask);
}

/*

void driver_chassis_left(int left_power){

    motor_target[DRIVE_LEFT_F - 1] = left_power*0.95;
    motor_target[DRIVE_LEFT_B - 1] = left_power*0.95;
}

void driver_chassis_right(int right_power){

  motor_target[DRIVE_RIGHT_F - 1] = right_power*0.95;
  motor_target[DRIVE_RIGHT_B - 1] = right_power*0.95;
}
*/
