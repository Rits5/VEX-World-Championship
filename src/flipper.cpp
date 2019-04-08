#include "main.h"
#include "flipper.h"
#include "allinit.h"
#include "motor.h"
#include "drive.h"

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


// void flipper_task(void*ignore){
//   while(true){
//
//   //autonomous
//     while(pros::competition::is_autonomous()){ //autonomous part of code
//
//       while(fabs((dist_before_flipper/(4*pi) * RPM200_GEARSET)) >=
//          fabs((drive_left_f.get_position() + drive_right_f.get_position() + drive_left_b.get_position() +
//          drive_right_b.get_position())/4)  && pros::competition::is_autonomous()){
//
//            pros::delay(10);;
//        }
//       //runs after distance in completed
//          while (pros::competition::is_autonomous()){
//
//              if(flipper_motor.get_position() < 100 && flipper_down == true){ //for making flipper go down
//
//                while(pros::millis() < millis_counter_flipper && pros::competition::is_autonomous()){
//                  flipper_motor.move(60); // voltage not rpm
//                  pros::delay(10);;
//                }
//                 flipper_motor.move(0);
//              }
//
//              if(flipper_motor.get_position() > 100 && flipper_down == false){ //for making flipper go back up
//
//                while(pros::millis() < millis_counter_flipper && pros::competition::is_autonomous()){
//                  flipper_motor.move_absolute(50, 200); // rpm
//                  pros::delay(10);;
//                }
//              }
//             pros::delay(10);;
//            }
//          }
//
//
//
// //Drvier Control
//     while(!pros::competition::is_autonomous()){ //driver control part of code
//
//       millis_counter_flipper = pros::millis() + 200;
//
//         if(flipper_motor.get_position() < 100 && flipper_down == true && button_pressed == true){ //for making flipper go down
//           while(pros::millis() < millis_counter_flipper){
//             flipper_motor.move(60); // voltage not rpm
//             pros::delay(10);;
//           }
//             flipper_motor.move(0);
//             flipper_down = false;
//             button_pressed = false; //to prevent flipper from automatically going up and dow uncontrollably
//         }
//
//         if(flipper_motor.get_position() > 100 && flipper_down == false && button_pressed == true){ //for making flipper go back up
//
//           while(pros::millis() < millis_counter_flipper){
//             flipper_motor.move_absolute(50, 200); // voltage not rpm
//             pros::delay(10);;
//           }
//             flipper_down = true;
//             button_pressed = false; //to prevent flipper from automatically going up and dow uncontrollably
//         }
//         pros::delay(10);;
//       }
//
//       pros::delay(10);;
//     }
//     pros::delay(10);;
//   }



// void flipper(float give_dist_before_flipper, bool give_flipper_down, int give_millis_counter_flipper){
//
//   dist_before_flipper = give_dist_before_flipper;
//   flipper_down = give_flipper_down;
//   millis_counter_flipper = give_millis_counter_flipper;
// }




// wednesday pre BCIT flipper manual control
// if(flipper_motor.get_position() < 100 && flipper_down == true && button_pressed == true){ //for making flipper go down
//   flipper_motor.move(75); //100 voltage not rpm
//   pros::delay(200); //variable time
//   flipper_motor.move(0);
//   flipper_down = false;
//   button_pressed = false; //to prevent flipper from automatically going up and dow uncontrollably
// }
// if(flipper_motor.get_position() > 100 && flipper_down == false && button_pressed == true){ //for making flipper go back up
//   flipper_motor.move_absolute(50, 200);
//   pros::delay(200); //give time for flipper to go back up
//   flipper_down = true;
//   button_pressed = false;
// }




void flipper_task(void* ignore){
  pid_terms flipper;
  pid_init(&flipper, 0.085, 0.005, 0.8, 10, 3000);  //0.06 and 0.2

    while(true){

  //autonomous part
    while(pros::competition::is_autonomous()){ //autonomous part of code
      int millis_flipper = millis_counter_flipper; //stores value of millis_counter_flipper given by give_millis_counter_flipper from the auto_flipper function
      millis_counter_flipper = pros::millis() + millis_flipper;

        while(fabs((dist_before_flipper/(4*pi) * RPM200_GEARSET)) >=
           fabs((drive_left_f.get_position() + drive_right_f.get_position() + drive_left_b.get_position() +
           drive_right_b.get_position())/4)  && pros::competition::is_autonomous() && dist_before_flipper != 0){

             pros::delay(10);
             millis_counter_flipper = pros::millis() + millis_flipper;
        }

         while(pros::competition::is_autonomous() && flipper_position_move == true){
         //if(flipper_position_move == true){
           //float error = flipper_position - flipper_motor.get_position();
           float calculated_power = pid_cal(&flipper, flipper_position_auton, pot.get_value());
           float final_power = power_limit(80, calculated_power);

           flipper_motor.move(final_power);

           pros::delay(10);;
           millis_counter_flipper = pros::millis() + millis_flipper;

          // printf("inside flipper loop\n");
         }

         while(pros::competition::is_autonomous() && flipper_position_move == false && pros::millis() < millis_counter_flipper){
             flipper_motor.move(flipper_speed); // voltage not rpm
             pros::delay(10);;
         //  }
         }
       pros::delay(10);;
      }


  //driver control part

      while(!pros::competition::is_autonomous()){
//printf("pot value %d\n", pot.get_value());
        //while(!pros::competition::is_autonomous() && flipper_move_to_position == true){
        if(flipper_move_to_position == true){
          //float error = flipper_position - flipper_motor.get_position();
          if(pot.get_value() > 1000){
            //pid_init(&flipper, 0.7, 2, 0, 0, 0);
            flipper.integral = 0;
          }
          // else{
          //   pid_init(&flipper, 0.05, 2, 0.2, 0, 0);
          // }
          float calculated_power = pid_cal(&flipper, flipper_position, pot.get_value());
          float final_power = power_limit(80, calculated_power);

          flipper_motor.move(final_power);

          //pros::delay(10);;
        }

        //while(!pros::competition::is_autonomous() && flipper_move_to_position == false){
        else if(flipper_move_to_position == false){
        //  while(pros::millis() < millis_counter_flipper){
            flipper_motor.move(flipper_speed); // voltage not rpm
            pros::delay(10);;
        //  }
          }

        pros::delay(10);;
      }

      pros::delay(10);;
    }

}


void flipper(bool give_flipper_move_to_position, int give_flipper_position, int give_flipper_speed, int give_resting_position, int give_extended_position){

  flipper_position = give_flipper_position;
  flipper_move_to_position = give_flipper_move_to_position;
  //flipper_timed = give_flipper_timed;
  flipper_speed = give_flipper_speed;
  resting_position = give_resting_position;
  extended_position = give_extended_position;

  int mid_pos = 1500;

  if(give_flipper_position == 350){

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



//was auton part of flipper task 2
// while(fabs((dist_before_flipper/(4*pi) * RPM200_GEARSET)) >=
//    fabs((drive_left_f.get_position() + drive_right_f.get_position() + drive_left_b.get_position() +
//    drive_right_b.get_position())/4)  && pros::competition::is_autonomous()){
//
//      pros::delay(10);;
//      millis_counter_flipper = pros::millis() + millis_flipper;
// }
//
//  //while(pros::competition::is_autonomous() && flipper_position_move == true){
//  if(flipper_position_move == true){
//    //float error = flipper_position - flipper_motor.get_position();
//    float calculated_power = pid_cal(&flipper, flipper_position, flipper_motor.get_position());
//    float final_power = power_limit(100, calculated_power);
//
//    flipper_motor.move(final_power);
//
//    pros::delay(10);;
//    millis_counter_flipper = pros::millis() + millis_flipper;
//  }
//
//  //while(pros::competition::is_autonomous() && flipper_position_move == false && pros::millis() < millis_counter_flipper){
//  if(flipper_position_move == false && pros::millis() < millis_counter_flipper){
//  //  while(pros::millis() < millis_counter_flipper){
//      flipper_motor.move(flipper_speed); // voltage not rpm
//      //pros::delay(10);;
//  //  }
//  }
// pros::delay(10);;
// }
