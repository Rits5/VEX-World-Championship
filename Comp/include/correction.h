#ifndef CORRECTION_H_
#define CORRECTION_H_

#include "main.h"

void correction_task(void*ignore);
extern pros::task_t correction_task_init;
extern float gyro_value;

#endif
