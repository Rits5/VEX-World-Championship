#ifndef SLEW_RATE_H_
#define SLEW_RATE_H_

#include "main.h"
#include "motor.h"

#define DEFAULT_SLEW 16

extern int islew;

extern int motor_error[21];
extern int motor_target[21];
extern int motor_power[21];
extern int motor_port[21];

void slew_rate_task(void* ignore);
int slew_cap(int value, int cap);

//pros::task_t slewtask;

void driver_chassis_left(int left);
void driver_chassis_right(int right);

#endif
