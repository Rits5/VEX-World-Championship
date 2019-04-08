#include "main.h"
#include "drive.h"
#include "intake.h"
#include "flywheel.h"
#include "motor.h"
#include "allinit.h"
#include "flipper.h"
#include "lcd.h"




/*
drive_pid(20, 100);
correction_drive = 0;
pros::lcd::print(3, "gyro turn value: %f", adi_gyro_get(gyro));
//pros::delay(1000);
//printf("gyro after %f\n", );
*/
/*
swing_pid(2, -90, 100);
pros::delay(500);
//pros::lcd::print(3, "gyro turn value: %f", adi_gyro_get(gyro));
//pros::delay(1000);
printf("gyro after %f\n", adi_gyro_get(gyro));
*/

using namespace pros::literals;

void autonomous() {

//original Ki turn value for turn_pid = 0.23
//original Kp value for swing turn = 0.25


pros::ADIGyro gyro (GYRO_PORT, 0.9510);
pros::lcd::initialize();

//blue = 3;

reset_error_globals();
reset_drive_encoders();
gyro.reset();
//red = 3;
auto_flipper(0, REST);
//turn_pid_encoder(90, 150);
//turn_pid(90, 150, 0.13);
// gyro.reset();
// swing_fast_pid(20, 90);
// turn_pid(90);
//brake_timer = 0;
//auto_flipper(0, 200);
//auto_double_shot(500, 300, 100);


//red auton front with park------------------------------------------------------------------------
if(red == 1){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 25); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(45, 250);

  pros::delay(400);

  drive_pid(-38.5); //-35    //drive back to starting tile to shoot balls
  turn_pid(-91, 100);     //face the flags

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(25);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(500);
  flywheel(0, 500);

  turn_pid(-15, 50, 0.3);   //short timeout

  swing_fast_pid(16, 15, 50);
  //drive_pid(5, 50);     //bottom flag hit

  drive_time(120, 700);
  reset_error_globals();

  intake(-80, 1000);
  drive_pid(-29, 100);
  turn_pid(90);      //turn towards cap
  reset_error_globals();

  intake(-100, 2000);
  drive_pid(23.5, 100, 60); //31    //flip cap 1

  turn_pid(65, 200);
  reset_error_globals();
  swing_fast_pid(24, 25, 150);

  intake(120, 1500);
  drive_time(50, 750);

  reset_error_globals();
  drive_pid(24.5, 1000); //parked

  pros::lcd::print(1, "timer 2 %d", pros::millis());
}



//red auton front 5 flags------------------------------------------------------------------------
else if(red == 2){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 25); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(45, 250);     //get the ball under the cap

  //reset_error_globals();

  pros::delay(400);

  drive_pid(-40); //-35    //drive back to starting tile to shoot balls
  turn_pid(-91.5, 100);     //face the flags

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(25);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(500);
  flywheel(0, 500);
//  flywheel(0, 500);

  // turn_pid(-17, 50);   //short timeout
  //
  // swing_fast_pid(10, 17, 50);
  //
  // intake_limit(true, 0);

  //drive_pid(8.5, 50);     //bottom flag hit

  // drive_time(60, 600);
  // reset_error_globals();

  //auto_flipper(23, EXTEND);
  drive_pid(13);

  auto_flipper(0, EXTEND);
  pros::delay(300);

  auto_flipper(0, REST);
  drive_pid(-40);

  auto_double_shot(560, 430, 100);
  turn_pid(45);      //turn towards cap

  //auto_flipper(17, EXTEND);
  drive_pid(23);
  auto_flipper(0, EXTEND);

  intake_limit(true, 1);
  drive_pid(-7);

  //intake_flip(true, -70, 2000, 0, true); //-80 speed

  pros::delay(1000);
  auto_double_shot(560, 430, 100); //double shot

  intake_limit(false, 0);
  intake(120, 1000);
  pros::delay(1000);

  auto_flipper(1, EXTEND);
  drive_pid(-5, 50);

  flywheel(0, 2000);

  auto_flipper(2, REST - 200);  //flip cap
  drive_pid(15);

  //drive_pid(24, 100, 100); //31    //flip cap 1


  //pros::delay(250);

  // intake_flip(false, 0, 0, 0, false);
  //
  // reset_error_globals();
  // turn_pid(-50);
  //
  // drive_pid(18.5);
  //
  // intake(120, 1000);
  // pros::delay(1000);
  //
  // swing_fast_pid(6.5, -40, 100, 0.3);
  // drive_time(100, 500);

}


//red auton back park quick------------------------------------------------------------------------
else if(red == 3){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 25); //intake after going 25 inches
  flywheel(468, 10000);   //start flywheel in background
  drive_pid(45, 250);     //get the ball under the cap

  pros::delay(500);

  drive_pid(-11, 250);    //drive back to starting tile to shoot balls
  turn_pid(-79);     //face the flags

  //drive_pid(2.5, 100);
  pros::delay(1000);
  intake(120, 350);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(350);

  flywheel(524, 15000);  //150

  pros::delay(1500);
  intake(120, 750);

  pros::delay(750);

  reset_error_globals();
  drive_pid(-24);

  turn_pid(79);

  intake(-120, 2000);
  drive_pid(20, 100, 80);  //flip cap 1

  drive_pid(-17.2, 100, 120);

  turn_pid(-90);
  drive_pid(33);

  intake(120, 1500);
  drive_time(50, 500);
  drive_pid(22.5, 1000); //parked

}




//red auton back counter cap------------------------------------------------------------------------------------------
else if(red == 4){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 0);
  flywheel(485, 5000);  //495
  drive_pid(28);
  turn_pid(-73.5);
  pros::delay(750);

  reset_error_globals();
  intake(120, 400);
  pros::delay(400);

  flywheel(525, 5000);    //534

  reset_error_globals();
  intake_limit(true, 0);
  drive_pid(5, 100, 15);    //collect ball from platform //20 speed

  pros::delay(100);
  reset_error_globals();
  drive_pid(-7, 100, 60);

  pros::delay(1900);

  intake_limit(false, 0);
  intake(120, 400);

  pros::delay(400);
  reset_error_globals();
  turn_pid(74.5);

  flywheel(555, 10000);   //563

  intake_limit(true, 0);
  drive_pid(23);

  pros::delay(500);
  drive_pid(-12.5);
  //reset_error_globals();

  turn_pid(-55.8);  //55.8
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();
  drive_pid(-28);
  turn_pid(57, 100);

  intake(-120, 2000);
  drive_pid(30, 100, 90);


}




//red auton back counter park------------------------------------------------------------------------------------------
else if(red == 5){

  intake_limit(true, 0);
  flywheel(485, 5000);  //490
  drive_pid(28);
  turn_pid(-73.5);
  pros::delay(750);

  reset_error_globals();
  intake(120, 400);
  pros::delay(400);

  flywheel(525, 5000);    //540

  reset_error_globals();
  intake_limit(true, 0);
  drive_pid(5, 100, 15);    //collect ball from platform //20 speed

  pros::delay(100);
  reset_error_globals();
  drive_pid(-7, 100, 60);

  pros::delay(1900);

  intake_limit(false, 0);
  intake(120, 400);

  pros::delay(400);
  reset_error_globals();
  turn_pid(74.5);

  flywheel(555, 10000);   //563

  intake_limit(true, 0);
  drive_pid(23);

  pros::delay(500);
  drive_pid(-12.5);
  //reset_error_globals();

  turn_pid(-56.5);  //55.8
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();
  turn_pid(-35);
//  drive_pid(-10);

//  turn_pid(-90);
  intake(120, 1500);
  drive_time(50, 700);
  drive_pid(24, 1000); //parked

}








//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

//BLUE SIDE CODE BEGINS







//blue auton front with park------------------------------------------------------------------------
else if(blue == 1){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 25); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(45, 250);

  pros::delay(400);

  drive_pid(-38.5); //-35    //drive back to starting tile to shoot balls
  turn_pid(91, 100);     //face the flags

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(25);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(500);
  flywheel(0, 500);

  turn_pid(15, 50, 0.3);   //short timeout

  swing_fast_pid(16, -15, 50);
  //drive_pid(5, 50);     //bottom flag hit

  drive_time(120, 700);
  reset_error_globals();

  intake(-80, 1000);
  drive_pid(-29, 100);
  turn_pid(-90);      //turn towards cap
  reset_error_globals();

  intake(-100, 2000);
  drive_pid(23.5, 100, 60); //31    //flip cap 1

  turn_pid(-65, 200);
  reset_error_globals();
  swing_fast_pid(24, -25, 150);

  intake(120, 1500);
  drive_time(50, 750);

  reset_error_globals();
  drive_pid(24.5, 1000); //parked

  pros::lcd::print(1, "timer 2 %d", pros::millis());
}






//blue auton front 5 flags------------------------------------------------------------------------
else if(blue == 2){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 25); //intake after going 25 inches
  flywheel(530, 15000);   //start flywheel in background //200
  drive_pid(45);     //get the ball under the cap

  //reset_error_globals();

  pros::delay(500);

  drive_pid(-36);    //drive back to starting tile to shoot balls
  turn_pid(90, 150, 0.26);     //face the flags

  intake(120, 400);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);
  flywheel(570, 5000); //200

  drive_pid(25);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(600);
//  flywheel(0, 500);

  turn_pid(17, 50);   //short timeout

  swing_fast_pid(10, -17, 50);

  intake_limit(true, 0);
  drive_pid(8.5, 50);     //bottom flag hit

  drive_time(60, 600);
  reset_error_globals();

  pros::delay(500);
  drive_pid(-26);
  //intake_limit(false, 0);
  turn_pid(-90);      //turn towards cap

  //reset_error_globals();

  intake_limit(false, 0);

  intake_flip(true, -60, 2000, 0, true);
  drive_pid(24, 100, 110); //31    //flip cap 1

  //pros::delay(250);

  intake_flip(false, 0, 0, 0, false);

  reset_error_globals();
  turn_pid(50);

  drive_pid(18.5);

  intake(120, 1000);
  pros::delay(1000);

  swing_fast_pid(8.5, 40, 100, 0.3);
  drive_time(100, 500);

}




//blue auton back park quick------------------------------------------------------------------------
else if(blue == 3){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 25); //intake after going 25 inches
  flywheel(465, 10000);   //start flywheel in background
  drive_pid(45, 250);     //get the ball under the cap

  pros::delay(500);

  drive_pid(-11, 250);    //drive back to starting tile to shoot balls
  turn_pid(78.5);     //face the flags

  //drive_pid(2.5, 100);
  pros::delay(1000);
  intake(120, 350);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(350);

  flywheel(524, 15000);  //150

  pros::delay(1500);
  intake(120, 750);

  pros::delay(750);

  reset_error_globals();
  drive_pid(-24);

  turn_pid(-78.5);

  intake(-100, 2000);
  drive_pid(20, 100, 60);  //flip cap 1

  drive_pid(-17, 100, 120);

  turn_pid(90);
  drive_pid(33);

  intake(120, 1500);
  drive_time(50, 500);
  drive_pid(22.5, 1000); //parked

}




//blue auton back counter cap------------------------------------------------------------------------
else if(blue == 4){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 0);
  flywheel(485, 5000);  //495
  drive_pid(28);
  turn_pid(73.5);
  pros::delay(750);

  reset_error_globals();
  intake(120, 400);
  pros::delay(400);

  flywheel(525, 5000);    //534

  reset_error_globals();
  intake_limit(true, 0);
  drive_pid(5, 100, 15);    //collect ball from platform //20 speed

  pros::delay(100);
  reset_error_globals();
  drive_pid(-7, 100, 60);

  pros::delay(1900);

  intake_limit(false, 0);
  intake(120, 400);

  pros::delay(400);
  reset_error_globals();
  turn_pid(-74);

  flywheel(555, 10000);   //563

  intake_limit(true, 0);
  drive_pid(23);

  pros::delay(500);
  drive_pid(-12.5);
  //reset_error_globals();

  turn_pid(56.5);  //55.8
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();
  drive_pid(-28);
  turn_pid(-57, 100);

  intake(-120, 2000);
  drive_pid(30, 100, 90);


}



//blue auton back counter park------------------------------------------------------------------------------------------
else if(blue == 5){

  intake_limit(true, 0);
  flywheel(480, 5000);  //495
  drive_pid(28);
  turn_pid(73.5);
  pros::delay(750);

  reset_error_globals();
  intake(120, 400);
  pros::delay(400);

  flywheel(525, 5000);    //540

  reset_error_globals();
  intake_limit(true, 0);
  drive_pid(5, 100, 15);    //collect ball from platform //20 speed

  pros::delay(100);
  reset_error_globals();
  drive_pid(-7, 100, 60);

  pros::delay(1900);

  intake_limit(false, 0);
  intake(120, 400);

  pros::delay(400);
  //reset_error_globals();
  turn_pid(-74);

  flywheel(555, 10000);   //563

  intake_limit(true, 0);
  drive_pid(23);

  pros::delay(500);
  drive_pid(-12.5);
  //reset_error_globals();

  turn_pid(56.5);  //55.8
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();
  turn_pid(35);
//  drive_pid(-10);

//  turn_pid(-90);
  intake(120, 1500);
  drive_time(50, 700);
  drive_pid(24, 1000); //parked


}







//skills auton---------------------------------------------------------------------------------------------------------------------------------------------------------

else if(red == 6){

    pros::lcd::print(0, "timer 1 %d", pros::millis());

    // flywheel(520, 15000);   //start flywheel in background //200
    // intake_flip(true, -40, 1000, 25, false);
    // drive_pid(44, 100, 100);     //get the ball 1 under the cap and flip cap 1
    // reset_error_globals();
    //
    // intake_flip(false, 0, 0, 0, false);
    // intake_limit(true, 0);
    // drive_pid(8, 100, 90);

    intake_limit(true, 25); //intake after going 25 inches
    flywheel(570, 10000);   //start flywheel in background //200
    drive_pid(45, 250);

    pros::delay(400);

    drive_pid(-38.5, 250); //-35    //drive back to starting tile to shoot balls
    turn_pid(-91, 300);     //face the flags

    intake(120, 300);       //first ball shot
    //turn_pid(0, 250);
    pros::delay(400);

    drive_pid(25);   //second ball shot
    intake(120, 1500); //2000

    pros::delay(500);
    flywheel(0, 500);

    turn_pid(-15, 50, 0.3);   //short timeout

    swing_fast_pid(16, 15, 50);
    //drive_pid(5, 50);     //bottom flag hit

    drive_time(120, 700);
    reset_error_globals();

    intake(-80, 1000);
    drive_pid(-29, 250);
    turn_pid(90);      //turn towards cap
    reset_error_globals();

    intake(-100, 2000);
    drive_pid(23.5, 250, 60); //31    //flip cap 1

    turn_pid(65, 200);
    reset_error_globals();
    swing_fast_pid(23, 24, 150);

    intake(120, 1500);
    drive_time(50, 750);

    reset_error_globals();
    drive_pid(24.5, 1000); //parked

    intake(120, 3000);
    turn_pid(-90);

    drive_time(50, 750);

    reset_error_globals();

    drive_pid(23.5, 1000);    //parked
    intake(0, 1500);


}




//testing -------------------------------------------------------------------------------------------------------------------------------------------------------------

  else if(blue == 6){

    //drive_pid(30);
    //drive_pid(-30);

    //auto_flipper(0, EXTEND);
    auto_double_shot(500, 300, 100);
    pros::delay(5000);
    //auto_flipper(0, 0, false);
    //auto_double_shot(500, 300, 100);
//flywheel(200, 10000);
  }

//flywheel(200, 10000);

}



// // red front 5 flag backup -- provincials autonomous
// pros::lcd::print(0, "timer 1 %d", pros::millis());
//
// intake_limit(true, 25); //intake after going 25 inches
// flywheel(570, 15000);   //start flywheel in background //200
// drive_pid(45);     //get the ball under the cap
//
// //reset_error_globals();
//
// pros::delay(500);
//
// drive_pid(-37);    //drive back to starting tile to shoot balls
// turn_pid(-90.5, 150, 0.26);     //face the flags
//
// intake(120, 300);       //first ball shot
// //turn_pid(0, 250);
// pros::delay(400);
// flywheel(570, 5000); //200
//
// drive_pid(25);   //second ball shot
// intake(120, 1500); //2000
//
// pros::delay(600);
// //  flywheel(0, 500);
//
// turn_pid(-17, 50);   //short timeout
//
// swing_fast_pid(10, 17, 50);
//
// intake_limit(true, 0);
// drive_pid(8.5, 50);     //bottom flag hit
//
// drive_time(60, 600);
// reset_error_globals();
//
// pros::delay(500);
// drive_pid(-26);
// //intake_limit(false, 0);
// turn_pid(90);      //turn towards cap
//
// //reset_error_globals();
//
// intake_limit(false, 0);
//
// intake_flip(true, -70, 2000, 0, true); //-80 speed
// drive_pid(24, 100, 100); //31    //flip cap 1
//
// //pros::delay(250);
//
// intake_flip(false, 0, 0, 0, false);
//
// reset_error_globals();
// turn_pid(-50);
//
// drive_pid(18.5);
//
// intake(120, 1000);
// pros::delay(1000);
//
// swing_fast_pid(6.5, -40, 100, 0.3);
// drive_time(100, 500);
