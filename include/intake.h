#ifndef INTAKE_H_
#define INTAKE_H_

#include "main.h"

#define LIGHT_SEN_THRESHOLD 2000

void intake_task (void*ignore);
extern pros::task_t intake_task_init;

extern int time_for_intake;
extern int speed_for_intake;
extern bool intake_flag;
extern float dist_before_intake;
extern bool outtake;
extern bool light_sen_flag;
extern unsigned int millis_counter_intake;

void intake (int give_speed_for_intake, int give_time_for_intake);
void intake_limit (bool give_intake_flag, float give_dist_before_intake);

void intake_flip(bool give_outtake, int give_speed_for_intake, int give_time_for_intake, float give_dist_before_intake, float give_light_sen_flag);
void wait_for_intake (void);

#endif
