#include "main.h"
#include "drive.h"
#include "allinit.h"
#include "motor.h"

using namespace pros::literals;

//globals init
  float correction_turn;  //stored value that makes turns more accurate
  float degrees_flag;  //targeted turn commandes given in PID loops
  float prev_correction_turn; //stored value in a timespace just before the correction_turn

  float correction_drive; //stored value that keeps robot driving straight overtime
  float prev_correction_drive;  //stored value in a timespace just before the correction_drive
  float drive_distance_correction; //stores a value of actual distance travelled and it is compared to the given target to accumulate
                      //and record encoder ticks overtime in order to minimize uncertainty
  float ticks_to_deg = 2.69739; //2.7395 //2.67 //2.72 & 2.717 was good //2.69739

  pid_terms drive;
  pid_terms turn_gyro;
  pid_terms swing;
  pid_terms turn_encoder;
  pid_terms drive_encoder;
  pid_terms swing_encoder;


  float power_limit(float allowed_speed, float actual_speed){

       if (actual_speed > allowed_speed){
           actual_speed = allowed_speed;
       }

       if (actual_speed < -allowed_speed){
           actual_speed = -allowed_speed;
       }

       return actual_speed;
   }

   void pid_init(pid_terms *pid, float Kp, float Ki, float Kd, float integral_limit, float integral_active_zone){

       pid->Kp = Kp;
       pid->Ki = Ki;
       pid->Kd = Kd;

       pid->proportional = 0;
       pid->integral = 0;
       pid->derivative = 0;

       pid->integral_limit = integral_limit;
       pid->error = 0;
       pid->last_error = 0;
       pid->power = 0;
       pid->integral_active_zone = integral_active_zone;

   }

   float pid_cal(pid_terms *pid, float target, float sensor){

       pid->error = target - sensor;
       pid->derivative = (pid->error - pid->last_error);
       pid->last_error = pid->error;
       pid->proportional = pid->error;
       pid->integral = pid->error + pid->integral;

           if ((pid->integral)*(pid->Ki) > pid->integral_limit){
             pid->integral = pid->integral_limit;
           }
           if ((pid->integral)*(pid->Ki) < -pid->integral_limit){
             pid->integral = -pid->integral_limit;
           }

           if(fabs(pid->error) > pid->integral_active_zone){
             pid->integral = 0;
           }

       pid->power = (pid->proportional)*(pid->Kp) + (pid->integral)*(pid->Ki) + (pid->derivative)*(pid->Kd);

       return pid->power;

   }


void reset_drive_encoders(void){
  drive_left_f.tare_position();
  drive_left_b.tare_position();
  drive_right_f.tare_position();
  drive_right_b.tare_position();
  encoder_left.reset();
  encoder_right.reset();
}


void reset_error_globals(void){
  correction_drive = 0;
  correction_turn = 0;
  degrees_flag = 0;
  prev_correction_turn = 0;
  prev_correction_drive = 0;
  drive_distance_correction = 0;
}


void drive_pid(float target, unsigned int timeout, int max_speed, float Kp_C){

  pros::ADIGyro gyro (GYRO_PORT, 0.967742);
  gyro.reset();
  reset_drive_encoders();			//reset encoder values

    if (pros::competition::is_autonomous()){

      //pid_terms drive;

      float Kp = 0.20;
      float Ki = 0;
      float Kd = 0.25;

      int failsafe = 2500;    //varible value
      int initial_millis = pros::millis();

      float final_power;
      float calculated_power;

      unsigned int net_timer;
      float error_c;

      float encoder_avg;

      bool timer_drive = true;			//used to exit out of piD loop after robot reaches error

      net_timer = pros::millis() + timeout; //just to initialize net_timer at first

      pid_init(&drive, Kp, Ki, Kd, 30, (12/(2.75*pi)));

        	while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){

            // encoder_avg = (drive_left_f.get_position() + drive_right_f.get_position() +
            //                   drive_left_b.get_position() + drive_right_b.get_position())/4;

            encoder_avg = (encoder_left.get_value() + encoder_right.get_value())/2;

            float error = ((target - drive_distance_correction)/(2.75*pi) * 360) - encoder_avg;

            calculated_power = pid_cal(&drive, ((target - drive_distance_correction)/(2.75*pi) * 360), encoder_avg);

            final_power = power_limit(max_speed, calculated_power);

              error_c = correction_drive + gyro.get_value();

              left_drive_set(final_power - error_c*Kp_C);
              right_drive_set(final_power + error_c*Kp_C);


        		if (fabs(error) < (1/(2.75*pi) * 360)){	//less than 1 inches
        			timer_drive = false;		//start timer to to exit pid loop
              drive.integral = 0;
        		}

        		else if (timer_drive){
        			net_timer = pros::millis() + timeout;
        		}

      		pros::delay(20);

      }

    }

    	drive_set(0);		//set drive to 0 power
      // printf("encoder avg: %f\n",(drive_left_f.get_position() + drive_right_f.get_position() +
      //                   drive_left_b.get_position() + drive_right_b.get_position())/286.5);
      // pros::lcd::print(2, "encoder avg: %f\n",(drive_left_f.get_position() + drive_right_f.get_position() +
      //                   drive_left_b.get_position() + drive_right_b.get_position())/286.5);

      printf("encoder avg: %f\n", (encoder_left.get_value() + encoder_right.get_value())/83.34);
      pros::lcd::print(2, "encoder avg: %f\n", (encoder_left.get_value() + encoder_right.get_value())/83.34);

      //printf("encoder left F %f\n", (motor_get_position(DRIVE_LEFT_F)/71.62));

      correction_turn = gyro.get_value() + correction_drive;
      prev_correction_drive = correction_drive;   //
      correction_drive = gyro.get_value() + prev_correction_drive;

      // drive_distance_correction = ((drive_left_f.get_position() + drive_right_f.get_position() +
      //                 drive_left_b.get_position() + drive_right_b.get_position())/286.5)
      //                 - target + drive_distance_correction;

      drive_distance_correction = ((encoder_left.get_value() + encoder_right.get_value())/83.34) - target + drive_distance_correction;

      printf("correction drive %f\n", correction_drive);
      printf("correction_turn = %1f\n", correction_turn);
      pros::lcd::print(0, "correction_turn = %1f\n", correction_turn);
}




void turn_pid(float degs, unsigned int timeout, float Ki){

  drive_distance_correction = 0;
  pros::ADIGyro gyro (GYRO_PORT, 0.967742);

  gyro.reset();
  //pid_terms turn_gyro;

  int failsafe = 1200; //still needs testing
  int initial_millis = pros::millis();
  degrees_flag = degs*10;

  float Kp = 0.24; //0.23 on friday //0.24 at BCIT
  float Kd = 1.54; //2.7 on friday //2.65 at BCIT
  Ki = 0.23;

  int max_speed = 110;			//caping minimum and maximum speeds for robot

  float calculated_power;
  float final_power;
  unsigned int net_timer;
  int direction;

  int encoder_average;

  bool timer_turn = true;

  net_timer = pros::millis() + timeout; //just to initialize net_timer at first

  pid_init(&turn_gyro, Kp, Ki, Kd, 50, 250); //40 was limit at BCIT

      while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){

        float error = (degs*10 - correction_turn) - gyro.get_value();

        calculated_power = pid_cal(&turn_gyro, degs*10 - correction_turn, gyro.get_value());

        final_power = power_limit(max_speed, calculated_power);

        turn_set(final_power);

        if (fabs(error) < 10){
          turn_gyro.integral = 0;
          timer_turn = false;
        }

        if (timer_turn){
          net_timer = pros::millis() + timeout;
        }

        pros::delay(20);
      }

    turn_set(0);
    printf("gyro value of turn: %1f\n", gyro.get_value());
    pros::lcd::print(2, "gyro turn value: %f", gyro.get_value());

    correction_drive = (correction_turn + gyro.get_value() - degrees_flag);
    prev_correction_turn = correction_turn;
    correction_turn = (prev_correction_turn + gyro.get_value() - degrees_flag);
    gyro.reset();

    pros::lcd::print(0, "correction_turn: %f", correction_turn);

  }



  //-------------------------------------------------------------------------------------------------------------


  void swing_fast_pid(float dist, float degs, unsigned int timeout, float Kp_turn, int failsafe_turn){

    int failsafe_drive = 2000;
    int initial_millis;
    initial_millis = pros::millis();

    degrees_flag = degs*10;

    pros::ADIGyro gyro (GYRO_PORT, 0.967742);
    gyro.reset();
    reset_drive_encoders();

  //pid_terms swing;

    float Kp = Kp_turn; //0.26
    float Kd = 1.24;
    float Ki = 0.07;

    float Kp_dist;

    float calculated_power;
    float final_power;
    unsigned int net_timer;
    int direction;

    float Kp_C = 0.35; //0.3
    float error_c;

    float encoder_average;
    int average_left;
    int average_right;
    int new_error_left;
    int new_error_right;

    int max_speed = 110;

    bool timer_swing = true;			//used to exit out of piD loop after robot reaches error

    net_timer = pros::millis() + timeout; //just to initialize net_timer at first

    if(dist > 0){direction = 1;}
    else if(dist < 0){direction = -1;}

    float error_dist = 100;

  pid_init(&swing, Kp, Ki, Kd, 30, 250);

      while((fabs(error_dist) > (1/(2.75*pi) * 360)) && pros::competition::is_autonomous() && ((initial_millis + failsafe_drive) > pros::millis())){   //less than 1in to the target

          float encoder_average = (encoder_left.get_value() + encoder_right.get_value())/2;

          error_dist = ((dist - drive_distance_correction)/(2.75*pi) * 360) - encoder_average;		//wheel size is 4.17 inches

          if (direction == 1){final_power = 100;}
          else if (direction == -1){final_power = -100;}

            error_c = correction_drive + gyro.get_value();

            left_drive_set(final_power - error_c*Kp_C);
            right_drive_set(final_power + error_c*Kp_C);

          pros::delay(20);

      }

      float avg = (drive_left_f.get_position() + drive_right_f.get_position() +
                  drive_left_b.get_position() + drive_right_b.get_position())/4;

     printf("encoder avg: %f\n",(drive_left_f.get_position() + drive_right_f.get_position() +
                                drive_left_b.get_position() + drive_right_b.get_position())/286.5);

     correction_turn = gyro.get_value() + correction_drive;
     prev_correction_drive = correction_drive;
     correction_drive = gyro.get_value() + prev_correction_drive;

     drive_distance_correction = 0;
     // drive_distance_correction = ((drive_left_f.get_position() + drive_right_f.get_position() +
     //                 drive_left_b.get_position() + drive_right_b.get_position())/286.5)
     //                 - dist + drive_distance_correction;

    //------------------------------------------------TURN PART OF LOOP- --------------------------------------------
      drive_set(0);
      reset_drive_encoders();			//reset encoder values
      gyro.reset();

      net_timer = pros::millis() + timeout;
      initial_millis = pros::millis();

    while ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe_turn) > pros::millis())){

      Kp_dist = 1.0;

        float error_turn = (degs*10 - correction_turn) - gyro.get_value();   //maybe do not need to multiply by 10 as degrees in PROS could be 360 already

        calculated_power = pid_cal(&swing, degs*10 - correction_turn, gyro.get_value());

        final_power = power_limit(max_speed, calculated_power);

          new_error_left = 0 - (drive_left_f.get_position() + drive_left_b.get_position())/2;
          new_error_right = 0 - (drive_right_f.get_position() + drive_right_b.get_position())/2;

              if (degs > 0 && dist > 0){  //when bot is commanded to go right while going forward
                left_drive_set(final_power);
                right_drive_set(new_error_right * Kp_dist);
              }

              if (degs < 0 && dist > 0){  //when bot is commanded to go left while forward
                right_drive_set(fabs(final_power * direction));
                left_drive_set(new_error_left * Kp_dist);
              }

              if (degs > 0 && dist < 0){  //when bot is commanded to go right while going backward
                right_drive_set(fabs(final_power) * direction);
                left_drive_set(new_error_left * Kp_dist);
              }

              if (degs < 0 && dist < 0){  //when bot is commanded to go left while going backward
                left_drive_set(fabs(final_power) * direction);
                right_drive_set(new_error_right * Kp_dist);
              }


        if (fabs(error_turn) < 10){ //15 //start timer when less than 1 degree
          swing.integral = 0; //variable
          timer_swing = false;
        }

          if (timer_swing){
            net_timer = pros::millis() + timeout;
          }

          pros::delay(20);
      }

      printf("gyro value of turn: %f\n", gyro.get_value());
      pros::lcd::print(2, "gyro turn after: %f", gyro.get_value());
      //turn_set(0);
      drive_set(0);
      correction_drive = (correction_turn + gyro.get_value() - degrees_flag);
      prev_correction_turn = correction_turn;
      correction_turn = (prev_correction_turn + gyro.get_value() - degrees_flag);

      gyro.reset();

  }




void drive_time(int speed, int timer){
  drive_set(speed);
  pros::delay(timer);
  drive_set(0);

}


void turn_time_flag(int speed, int timer){
  degrees_flag = -900;

  pros::ADIGyro gyro (GYRO_PORT, 0.967742);
  gyro.reset();

  drive_set(speed);
  pros::delay(timer);
  drive_set(0);

  correction_drive = (correction_turn + gyro.get_value() - degrees_flag);
}






  void swing_pid(float dist, float degs, unsigned int timeout, float Kp_turn, int failsafe_turn){

    int failsafe_drive = 2000;
  //  int failsafe_turn = 1500;
    int initial_millis;
    initial_millis = pros::millis();

    degrees_flag = degs*10;

    pros::ADIGyro gyro (GYRO_PORT, 0.967742);
    gyro.reset();

    float Kp_dist;

    float Kp;
    float Kd;		//1.6			//ONLY test values
    float Ki;
    //float Ki_turn;
    float proportional, integral, derivative;

    float error_dist;
    int error_turn;

    int last_error;

    int integral_limit = 30;
    float final_power;
    unsigned int net_timer;
    int direction;

    float Kp_C = 0.4; //0.3
    float error_c;

    int encoder_average;
    int average_left;
    int average_right;
    int new_error_left;
    int new_error_right;
    //int timeout = 200;

    int max_positive_speed = 120;			//caping minimum and maximum speeds for robot
    int max_negative_speed = -120;		//to prevent stalling

  //  int max_turn_speed = 95;

    bool timer_swing = true;			//used to exit out of piD loop after robot reaches error

    reset_drive_encoders();		//reset encoder values
  	gyro.reset();

    net_timer = pros::millis() + timeout; //just to initialize net_timer at first

    if(dist > 0){direction = 1;}
    else if(dist < 0){direction = -1;}

    error_dist = ((dist - drive_distance_correction)/(4*pi) * RPM200_GEARSET) - encoder_average;

      while((fabs(error_dist) > (1/(4*pi) * RPM200_GEARSET)) && pros::competition::is_autonomous() && ((initial_millis + failsafe_drive) > pros::millis())){   //less than 1in to the target

          Kp = 0.2;
          Kd = 0.25;
          Ki = 0;
          integral_limit = 30;

          int encoder_average = (drive_left_f.get_position() + drive_right_f.get_position() +
                                drive_left_b.get_position() + drive_right_b.get_position())/4;

          error_dist = (dist/(4*pi) * RPM200_GEARSET) - encoder_average;		//wheel size is 4.17 inches

          proportional = error_dist*Kp;
          integral = (error_dist + integral)*Ki;
          derivative = (error_dist - last_error)*Kd;
          last_error = error_dist;

          if (error_dist > (12/(4*pi) * RPM200_GEARSET)){
            integral = 0;
          }

          if (integral > integral_limit){
            integral = integral_limit;
          }

          if (-integral < -integral_limit){
            integral = -integral_limit;
          }

          final_power = proportional + derivative + integral;

            if (final_power > max_positive_speed){
              final_power = max_positive_speed;
            }

            if (final_power < max_negative_speed){
              final_power = max_negative_speed;
            }

            if(fabs(error_dist) > (0.1/(4*pi) * RPM200_GEARSET)){

              if (direction > 0){   //while going forward
                error_c = correction_drive + gyro.get_value();

              //  printf("inside loop forwards\n");
                left_drive_set(final_power - error_c*Kp_C);
                right_drive_set(final_power + error_c*Kp_C);
              }

              else if (direction < 0){  //while going backwards
                error_c = correction_drive + gyro.get_value();

              //  printf("inside loop backwards\n");
                left_drive_set(final_power - error_c*Kp_C);
                right_drive_set(final_power + error_c*Kp_C);
              }
            }

              else {
              left_drive_set(final_power);
              right_drive_set(final_power);
              }

          pros::delay(10);

      }

      float avg = (drive_left_f.get_position() + drive_right_f.get_position() +
                  drive_left_b.get_position() + drive_right_b.get_position())/4;

     printf("encoder avg: %f\n",(drive_left_f.get_position() + drive_right_f.get_position() +
                                drive_left_b.get_position() + drive_right_b.get_position())/286.5);

    drive_distance_correction = 0;
    // drive_distance_correction = ((drive_left_f.get_position() + drive_right_f.get_position() +
    //                 drive_left_b.get_position() + drive_right_b.get_position())/286.5)
    //                 - dist + drive_distance_correction;

    //------------------------------------------------TURN PART OF LOOP- --------------------------------------------
      drive_set(0);
      reset_drive_encoders();			//reset encoder values
      gyro.reset();

      net_timer = pros::millis() + timeout;
      initial_millis = pros::millis();

    while ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe_turn) > pros::millis())){

    //  Kp = 0.25;    //0.25
      Kd = 1.24;		//1.2
      Ki = 0.07;    //0.07
      Kp_dist = 1.0;
      integral_limit = 50;

        error_turn = (degs*10 - correction_turn) - gyro.get_value();   //maybe do not need to multiply by 10 as degrees in PROS could be 360 already

          derivative = (error_turn - last_error)*Kd;
          last_error = error_turn;
          proportional = error_turn*Kp_turn;
          integral = (error_turn + integral)*Ki;

            if (integral > integral_limit){
                integral = integral_limit;
            }

            if (integral < -integral_limit){
                integral = -integral_limit;
            }

            final_power = proportional + derivative + integral;

            if (final_power > max_positive_speed){
              final_power = max_positive_speed;
            }

            if (final_power < max_negative_speed){
              final_power = max_negative_speed;
            }

            new_error_left = 0 - (drive_left_f.get_position() + drive_left_b.get_position())/2;
            new_error_right = 0 - (drive_right_f.get_position() + drive_right_b.get_position())/2;

                if (degs > 0 && dist > 0){  //when bot is commanded to go right while going forward
                  left_drive_set(final_power);
                  right_drive_set(new_error_right * Kp_dist);
                }

                if (degs < 0 && dist > 0){  //when bot is commanded to go left while forward
                  right_drive_set(fabs(final_power * direction));
                  left_drive_set(new_error_left * Kp_dist);
                }

                if (degs > 0 && dist < 0){  //when bot is commanded to go right while going backward
                  right_drive_set(fabs(final_power) * direction);
                  left_drive_set(new_error_left * Kp_dist);
                }

                if (degs < 0 && dist < 0){  //when bot is commanded to go left while going backward
                  left_drive_set(fabs(final_power) * direction);
                  right_drive_set(new_error_right * Kp_dist);
                }


          if (abs(error_turn) < 10){ //15 //start timer when less than 1 degree
            integral = 0; //variable
            timer_swing = false;
          }

          if (timer_swing){
            net_timer = pros::millis() + timeout;
          }

          pros::delay(10);
        //  correction_drive = (correction_turn + gyro.get_value()) - degs;

        }

      printf("gyro value of turn: %f\n", gyro.get_value());
      //pros::lcd::print(5, "gyro turn after: %f", gyro.get_value());
      //turn_set(0);
      drive_set(0);
      correction_drive = (correction_turn + gyro.get_value() - degrees_flag);
      prev_correction_turn = correction_turn;
      correction_turn = (prev_correction_turn + gyro.get_value() - degrees_flag);

      gyro.reset();

  }







//---------------------------------------------------------------------------------------------------------------------------------------------------------

void turn_pid_encoder(double target, unsigned int timeout){ //makn encoder pid

  //pid_terms turn_encoder;

  drive_distance_correction = 0;
  reset_drive_encoders();

  degrees_flag = target*ticks_to_deg;

  float Kp = 0.7;  //0.42
  float Kd = 5;		//0.1.2
  float Ki = 0.25;    //0.25

  float calculated_power;
  float final_power;
  float encoder_avg;

  int max_speed = 110; //90

  int failsafe = 1500;    //2000
  int initial_millis = pros::millis();

  unsigned int net_timer;

  bool timer_turn = true;

  pid_init(&turn_encoder, Kp, Ki, Kd, 50, 25*ticks_to_deg);

  net_timer = pros::millis() + timeout; //just to initialize net_timer at first

    while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){

      encoder_avg = (encoder_left.get_value() - encoder_right.get_value())/2;
      float error = (target*ticks_to_deg - correction_turn) - encoder_avg;

      calculated_power = pid_cal(&turn_encoder, (target*ticks_to_deg - correction_turn), encoder_avg);
      final_power = power_limit(max_speed, calculated_power);

      turn_set(final_power);

        if (timer_turn == true){
          net_timer = pros::millis() + timeout;
        }

        if (fabs(error) < 2*ticks_to_deg){   //if less than 2 degrees
          turn_encoder.integral = 0;
          timer_turn = false;
        }

      pros::delay(10);
      pros::lcd::print(2, "left encoder %d", encoder_left.get_value());
      pros::lcd::print(1, "right encoder %d", encoder_right.get_value());

      printf("left encoder: %d\n", encoder_left.get_value());
      printf("right encoder: %d\n", encoder_right.get_value());

  }
    turn_set(0);

    correction_drive = (correction_turn + (encoder_left.get_value() - encoder_right.get_value())/2 - degrees_flag)*2;
    prev_correction_turn = correction_turn;
    correction_turn = (prev_correction_turn + (encoder_left.get_value() - encoder_right.get_value())/2 - degrees_flag);

    pros::lcd::print(0, "correction_turn: %f", correction_turn);

    printf("correction turn %f\n", correction_turn);
    printf("encoder avg %d\n", (encoder_left.get_value() + encoder_right.get_value())/2);


}







//--------------------------------------------------------------------------------------------------------------------------------------------


void drive_pid_encoder(float target, unsigned int timeout, int max_speed, float Kp_C){//, float Kp){

  //pid_terms drive_encoder;
  reset_drive_encoders();			//reset encoder values

if (pros::competition::is_autonomous()){

  int failsafe = 2500;    //varible value
  int initial_millis = pros::millis();

  float Kp = 0.5; //0.2
  float Kd = 0.8;		//0.27
  float Ki = 0; //0

  float calculated_power;
  float final_power;

  int integral_limit = 50;
  unsigned int net_timer;
  //float Kp_C = 0.4; //0.4
  float error_c;

  float encoder_average;

  bool timer_drive = true;			//used to exit out of piD loop after robot reaches error

  net_timer = pros::millis() + timeout; //just to initialize net_timer at first

  pid_init(&drive_encoder, Kp, Ki, Kd, 30, (12/(2.75*pi)));

  	while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){

        encoder_average = (encoder_right.get_value() + encoder_left.get_value())/2;

    		float error = ((target)/(2.75*pi) * 360)  - drive_distance_correction - encoder_average;		//wheel size is 4.17 inches

    	  calculated_power = pid_cal(&drive_encoder, ((target)/(2.75*pi) * 360)  - drive_distance_correction, encoder_average);

        final_power = power_limit(max_speed, calculated_power);

        error_c = (encoder_left.get_value() - encoder_right.get_value()) + correction_drive;

        left_drive_set(final_power - error_c*Kp_C);
        right_drive_set(final_power + error_c*Kp_C);

        		if (fabs(error) < (1/(2.75*pi) * 360)){	//less than 1 inches
        			timer_drive = false;		//start timer to to exit piD loop
              drive_encoder.integral = 0;
        		}

        		else if (timer_drive){
        			net_timer = pros::millis() + timeout;
        		}

  		pros::delay(10);

    }

  }

  	drive_set(0);		//set drive to 0 power

    correction_turn = ((encoder_left.get_value() - encoder_right.get_value())/2 + correction_drive);
    prev_correction_drive = correction_drive;
    correction_drive = (encoder_left.get_value() - encoder_right.get_value()) + prev_correction_drive;

    drive_distance_correction = ((encoder_left.get_value() + encoder_right.get_value())/2)
                    - ((target)/(2.75*pi) * 360) + drive_distance_correction;

    printf("correction drive %f\n", correction_drive);
    printf("correction_turn = %1f\n", correction_turn);

    pros::lcd::print(0, "correction drive %f", correction_drive);
}





void swing_pid_encoder(float dist, float degs, unsigned int timeout, float Kp_turn, int failsafe_turn){

  int failsafe_drive = 2000;
  int initial_millis;
  initial_millis = pros::millis();

  degrees_flag = degs*10;

  pros::ADIGyro gyro (GYRO_PORT, 0.967742);
  gyro.reset();
  reset_drive_encoders();

//pid_terms swing_encoder;

  float Kp = Kp_turn;
  float Kd = 1.24;
  float Ki = 0.07;

  float Kp_dist;

  float calculated_power;
  float final_power;
  unsigned int net_timer;
  int direction;

  float Kp_C = 0.4; //0.3
  float error_c;

  float encoder_average;
  int average_left;
  int average_right;
  int new_error_left;
  int new_error_right;

  int max_speed = 110;

  bool timer_swing = true;			//used to exit out of piD loop after robot reaches error

  net_timer = pros::millis() + timeout; //just to initialize net_timer at first

  if(dist > 0){direction = 1;}
  else if(dist < 0){direction = -1;}

  float error_dist;

pid_init(&swing_encoder, Kp, Ki, Kd, 50, 250);

    while((fabs(error_dist) > (1/(2.75*pi) * 360)) && pros::competition::is_autonomous() && ((initial_millis + failsafe_drive) > pros::millis())){   //less than 1in to the target

        float encoder_average = ((drive_left_f.get_position() + drive_right_f.get_position() +
                              drive_left_b.get_position() + drive_right_b.get_position()))/4;

        error_dist = ((dist - drive_distance_correction)/(4*pi) * 360) - encoder_average;		//wheel size is 4.17 inches

        if (direction == 1){final_power = 100;}
        else if (direction == -1){final_power = -100;}

          error_c = correction_drive + gyro.get_value();

          left_drive_set(final_power - error_c*Kp_C);
          right_drive_set(final_power + error_c*Kp_C);

        pros::delay(10);

    }

   correction_turn = ((encoder_left.get_value() - encoder_right.get_value())/2 + correction_drive)/ticks_to_deg; //get correction in degrees
   correction_drive = 0;
   drive_distance_correction = 0;

  //------------------------------------------------TURN PART OF LOOP- --------------------------------------------
    drive_set(0);
    reset_drive_encoders();			//reset encoder values
    gyro.reset();

    net_timer = pros::millis() + timeout;
    initial_millis = pros::millis();

  while ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe_turn) > pros::millis())){

    Kp_dist = 1.0;

      float error_turn = (degs*10 - correction_turn*10) - gyro.get_value();   //maybe do not need to multiply by 10 as degrees in PROS could be 360 already

      calculated_power = pid_cal(&swing_encoder, degs*10 - correction_turn*10, gyro.get_value());

      final_power = power_limit(max_speed, calculated_power);

        new_error_left = 0 - (drive_left_f.get_position() + drive_left_b.get_position())/2;
        new_error_right = 0 - (drive_right_f.get_position() + drive_right_b.get_position())/2;

            if (degs > 0 && dist > 0){  //when bot is commanded to go right while going forward
              left_drive_set(final_power);
              right_drive_set(new_error_right * Kp_dist);
            }

            if (degs < 0 && dist > 0){  //when bot is commanded to go left while forward
              right_drive_set(fabs(final_power * direction));
              left_drive_set(new_error_left * Kp_dist);
            }

            if (degs > 0 && dist < 0){  //when bot is commanded to go right while going backward
              right_drive_set(fabs(final_power) * direction);
              left_drive_set(new_error_left * Kp_dist);
            }

            if (degs < 0 && dist < 0){  //when bot is commanded to go left while going backward
              left_drive_set(fabs(final_power) * direction);
              right_drive_set(new_error_right * Kp_dist);
            }


      if (fabs(error_turn) < 10){ //15 //start timer when less than 1 degree
        swing_encoder.integral = 0; //variable
        timer_swing = false;
      }

        if (timer_swing){
          net_timer = pros::millis() + timeout;
        }

        pros::delay(10);
    }

    drive_set(0);
    correction_drive = ((correction_turn*10 + gyro.get_value() - degrees_flag)/10) *2*ticks_to_deg;
    prev_correction_turn = correction_turn;
    correction_turn = ((prev_correction_turn*10 + gyro.get_value() - degrees_flag)/10) * ticks_to_deg; //convert to encoder degrees

}



//
//   //-------------------------------------------------------------------------------------------------------------
//   //NOT GOOD, DON'T USE
//
//   void swing_pid_encoder(float dist, float degs, unsigned int timeout, float Kp_turn, int failsafe_turn){
//
//     int failsafe_drive = 2000;
//   //  int failsafe_turn = 1500;
//     int initial_millis;
//     initial_millis = pros::millis();
//
//     degrees_flag = degs*ticks_to_deg;
//
//     float Kp_dist;
//
//     float Kp;
//     float Kd;		//1.6			//ONLY test values
//     float Ki;
//     //float Ki_turn;
//     float proportional, integral, derivative;
//
//     float error_dist;
//     int error_turn;
//
//     int last_error;
//
//     int integral_limit = 30;
//     float final_power;
//     unsigned int net_timer;
//     int direction;
//
//     float Kp_C = 0.4; //0.3
//     float error_c;
//
//     int encoder_average;
//     int avg_left;
//     int avg_right;
//     int new_error_left;
//     int new_error_right;
//     float encoder_avg;
//
//     int max_speed = 90; //100
//
//     bool timer_swing = true;			//used to exit out of piD loop after robot reaches error
//
//     reset_drive_encoders();		//reset encoder values
//   //	gyro.reset();
//
//     net_timer = pros::millis() + timeout; //just to initialize net_timer at first
//
//     if(dist > 0){direction = 1;}
//     else if(dist < 0){direction = -1;}
//
//     float target = dist/(4*pi) * 360;
//     error_dist = 500; //initialize error
//
//       while((fabs(error_dist) > (0.5/(4*pi) * 360)) && pros::competition::is_autonomous() && ((initial_millis + failsafe_drive) > pros::millis())){   //less than 1in to the target
// // (2/(4*pi) * 360)
//           encoder_avg = (encoder_left.get_value() + encoder_right.get_value())/2;
//
//           error_dist = target - drive_distance_correction - (encoder_left.get_value() + encoder_right.get_value())/2;		//wheel size is 4.17 inches
//
//           if (direction == 1){final_power = max_speed;}
//           else if (direction == -1){final_power = -max_speed;}
//
//
//           error_c = (encoder_left.get_value() - encoder_right.get_value()) + correction_drive;
//
//           left_drive_set(final_power - error_c*Kp_C);
//           right_drive_set(final_power + error_c*Kp_C);
//
//           pros::delay(10);
//
//           pros::lcd::print(5, "error dist value %f", error_dist);
//           pros::lcd::print(6, "encoder avg %f", encoder_avg);
//           printf("error dist value %f\n", error_dist);
//           printf("distance correction %f\n", drive_distance_correction);
//       }
//
//      correction_turn = ((encoder_left.get_value() - encoder_right.get_value())/2 + correction_drive);///ticks_to_deg;
//      prev_correction_drive = correction_drive;
//      correction_drive = (encoder_left.get_value() - encoder_right.get_value()) + prev_correction_drive;
//
//      // drive_distance_correction = ((encoder_left.get_value() + encoder_right.get_value())/2)
//      //                 - ((dist)/(4*pi) * 360) + drive_distance_correction;
//      drive_distance_correction = 0; //0 to eliminate inconcistency in next movement
//
//     //------------------------------------------------TURN PART OF LOOP- --------------------------------------------
//       drive_set(0);
//       reset_drive_encoders();			//reset encoder values
//
//       net_timer = pros::millis() + timeout;
//       initial_millis = pros::millis();
//
//     while ((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe_turn) > pros::millis())){
//
//     //  Kp = 0.25;    //0.25
//       Kd = 1.24;		//1.2
//       Ki = 0.07;    //0.07
//       Kp_dist = 1.0;
//       integral_limit = 50;
//
//         if(degs > 0){encoder_avg = encoder_left.get_value();} //turning clockwise / to the right
//         if(degs < 0){encoder_avg = encoder_right.get_value();} //turning counter clockwise / to the left
//
//         if(dist > 0){error_turn = degs*ticks_to_deg - correction_turn - encoder_avg;}
//         if(dist < 0){error_turn = degs*ticks_to_deg - correction_turn + encoder_avg;}
//
//         new_error_left = 0 - encoder_left.get_value();
//         new_error_right = 0 - encoder_right.get_value();
//         // new_error_left = 0 - (drive_left_f.get_position() + drive_left_b.get_position())/2;
//         // new_error_right = 0 - (drive_right_f.get_position() + drive_right_b.get_position())/2;
//
//           derivative = (error_turn - last_error)*Kd;
//           last_error = error_turn;
//           proportional = error_turn*Kp_turn;
//           integral = (error_turn + integral)*Ki;
//
//             if (integral > integral_limit){
//                 integral = integral_limit;
//             }
//
//             if (integral < -integral_limit){
//                 integral = -integral_limit;
//             }
//
//             final_power = proportional + derivative + integral;
//
//             if (final_power > max_speed){
//               final_power = max_speed;
//             }
//
//             if (final_power < -max_speed){
//               final_power = -max_speed;
//             }
//
//
//                 if (degs > 0 && dist > 0){  //when bot is commanded to face right while going forward
//                   left_drive_set(final_power * direction);
//                   right_drive_set(new_error_right * Kp_dist);
//                 }
//
//                 if (degs < 0 && dist > 0){  //when bot is commanded to face left while forward
//                   right_drive_set(final_power * direction);
//                   left_drive_set(new_error_left * Kp_dist);
//                 }
//
//                 if (degs > 0 && dist < 0){  //when bot is commanded to face right while going backward
//                   right_drive_set(final_power * direction);
//                   left_drive_set(new_error_left * Kp_dist);
//                 }
//
//                 if (degs < 0 && dist < 0){  //when bot is commanded to face left while going backward
//                   left_drive_set(final_power * direction);
//                   right_drive_set(new_error_right * Kp_dist);
//                 }
//
//
//           if (abs(error_turn) < 10){ //15 //start timer when less than 1 degree
//             integral = 0; //variable
//             timer_swing = false;
//           }
//
//           if (timer_swing){
//             net_timer = pros::millis() + timeout;
//           }
//
//           pros::delay(10);
//         //  correction_drive = (correction_turn + gyro.get_value()) - degs;
//
//         }
//
//     //  printf("gyro value of turn: %f\n", gyro.get_value());
//       //pros::lcd::print(5, "gyro turn after: %f", gyro.get_value());
//       //turn_set(0);
//       drive_set(0);
//       // correction_drive = (correction_turn + gyro.get_value() - degrees_flag);
//       // prev_correction_turn = correction_turn;
//       // correction_turn = (prev_correction_turn + gyro.get_value() - degrees_flag);
//
//
//   }
