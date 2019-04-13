#include "main.h"
#include "allinit.h"
#include "correction.h"
#include "drive.h"

pros::task_t correction_task_init;
float gyro_value;

void correction_task(void*ignore){

  pros::ADIGyro gyro (GYRO_PORT, 0.967742);

    while(true){

      while(pros::competition::is_autonomous()){

        if(reset_gyro == true){
          gyro.reset();
        }
        reset_gyro = false;

        gyro_value = gyro.get_value();

        pros::delay(10);
      }
      pros::delay(5);
    }
}
