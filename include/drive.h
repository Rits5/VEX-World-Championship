#ifndef DRIVE_H_
#define DRIVE_H_

#include "main.h"

extern float correction_turn;  //stored value that makes turns more accurate
extern float degrees_flag;  //targeted turn commandes given in PID loops
extern float prev_correction_turn; //stored value in a timespace just before the correction_turn

extern float prev_gyro_value;

extern float correction_drive; //stored value that keeps robot driving straight overtime
extern float prev_correction_drive;  //stored value in a timespace just before the correction_drive
extern float drive_distance_correction; //stores a value of actual distance travelled and it is compared to the given target to accumulate
                    //and record encoder ticks overtime in order to minimize uncertainty

extern float ticks_to_deg;

//gyro containing PID
void drive_pid(float target, unsigned int timeout = 150, int max_speed = 120, float Kp_C = 0.5);
void turn_pid(float degs, unsigned int timeout = 150, float Ki = 0.23); //0.13
void swing_pid(float dist, float degs, unsigned int timeout = 150, float Kp_turn = 0.293, int failsafe_turn = 1500);

void swing_fast_pid(float dist, float degs, unsigned int timeout = 150, float Kp_turn = 0.35, int failsafe_turn = 1500);

//encoder containing PID
void turn_pid_encoder(double target, unsigned int timeout = 150);
void drive_pid_encoder(float target, unsigned int timeout = 100, int max_speed = 110, float Kp_C = 0.5);
void swing_pid_encoder(float dist, float degs, unsigned int timeout = 150, float Kp_turn = 1.0, int failsafe_turn = 2000);

//Utility stuff
void reset_drive_encoders(void);
void reset_error_globals(void);
void drive_time(int speed, int timer);
void turn_time_flag(int speed, int timer);

void turn_pid2(float degs, unsigned int timeout = 150, float Ki = 0.23);
void drive_pid2(float target, unsigned int timeout = 150, int max_speed = 120, float Kp_C = 0.4);
void swing_fast_pid2(float dist, float degs, unsigned int timeout = 150, float Kp_turn = 0.25, int failsafe_turn = 1500);


typedef struct pid_terms{
    float Kp;
    float Kd;
    float Ki;

    float proportional;
    float integral;
    float derivative;

    float integral_limit;
    float error;
    float last_error;
    float power;
    float integral_active_zone;

}pid_terms;

extern pid_terms drive;
extern pid_terms turn_gyro;
extern pid_terms swing;
extern pid_terms turn_encoder;
extern pid_terms drive_encoder;
extern pid_terms swing_encoder;

 void pid_init(pid_terms *pid, float Kp, float Ki, float Kd, float integral_limit, float integral_active_zone);
 float pid_cal(pid_terms *pid, float target, float sensor);
 float power_limit(float allowed_speed, float actual_speed);

 extern float prev_degs;
 extern bool reset_gyro;


#endif
