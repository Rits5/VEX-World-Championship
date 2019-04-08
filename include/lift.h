#ifndef LIFT_H_
#define LIFT_H_

#include "main.h"

extern pros::task_t lift_task_init;
void lift_task(void* ignore);

extern int target_height;
extern int lift_delay;
extern int lift_speed_up;
extern int lift_speed_down;
extern int max_lift_speed;
extern float Kp_lift;
extern int lift_switch_mode;

//void lift(int give_target_height, int give_lift_delay, int give_lift_speed_up, int give_lift_speed_down);


#endif
