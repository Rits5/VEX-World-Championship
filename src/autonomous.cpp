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


pros::ADIGyro gyro (GYRO_PORT, 0.967742);
pros::lcd::initialize();

reset_error_globals();
reset_drive_encoders();
gyro.reset();

// //timed_flywheel = true;
// flipper(true, 2800);
// int something = flipper_position;
// //flipper_position = 2800;
// pros::delay(2000);
// auto_double_shot(500, 300, 100);
// pros::lcd::print(1,"flipper commanded: %d", something);

blue = 0;
red = 6;


//red auton front with park------------------------------------------------------------------------
if(red == 1){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(45, 250);

  pros::delay(400);

  drive_pid(-41.5, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(-90);     //face the flags

  reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(25);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(500);
  flywheel(100, 500);

  turn_pid(-10);
  drive_pid(14);

  // flipper(true, EXTEND);
  // pros::delay(300);
  // flipper(true, REST);

  intake(-80, 1000);
  drive_pid(-20, 100);
  flipper(true, EXTEND);
  turn_pid(100);      //turn towards cap
  reset_error_globals();

  //intake(-100, 2000);
  flipper(true, REST + 500, 10);
  drive_pid(21.5, 100, 90); //31    //flip cap 1

  turn_pid(65);
  flipper(true, REST);
  reset_error_globals();
  swing_fast_pid(24, 23, 300);

  intake(120, 2500);
  drive_time(50, 1000);

  pros::delay(200);

  reset_error_globals();
  drive_pid(24.5, 1000); //parked

  pros::lcd::print(1, "timer 2 %d", pros::millis());
}



//red auton front 5 flags------------------------------------------------------------------------
else if(red == 2){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(45, 250);

  pros::delay(400);

  drive_pid(-41.5, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(-90);     //face the flags

  reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(25);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(500);
  flywheel(100, 500);

  turn_pid(-10);
  drive_pid(14);

  intake(-80, 1000);

  flywheel(520, 7000);

  drive_pid(-36); //-33.5 is on point
  //swing_fast_pid(-20, -40);

  turn_pid(58);      //turn towards cap

  //auto_flipper(17, EXTEND);S
  drive_pid(13, 250, 90);
  //pros::dela
  flipper(true, EXTEND - 100);
  pros::delay(300);

  intake_limit(true, 2);
  drive_pid(-10);
  //drive_pid(-15);

  pros::delay(1000);
  auto_double_shot(520, 407, 170); //double shot

  intake_limit(false, 0);

  //turn_pid(2.2);
  intake(90, 1200);
  pros::delay(1100);

  intake(-70, 2000);
  flywheel(0, 1000);
  flywheel(100, 500);
  flipper(true, REST - 400);
  drive_pid(17, 150, 90);

  //pros::delay(300);
  turn_pid(90);
  //flywheel(0, 500);
  // drive_pid(-10);
  //
  // flywheel(520, 7000);
  // flipper(true, EXTEND);
  // pros::delay(700);
  // flipper(true, REST, 11);
  // drive_pid(18);    //flip cap


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
  //flywheel(485, 5000);  //495
  auto_double_shot(525, 470, 120);
  drive_pid(28);
  turn_pid(-73.5);

  intake(120, 400);
  pros::delay(400);

  flywheel(470, 5000);
  intake_limit(true, 0);
  drive_pid(5);    //collect ball from platform //20 speed

  flipper(true, EXTEND - 200);
  drive_pid(-7);

  pros::delay(750);

  intake_limit(false, 0);
  intake(120, 400);
  pros::delay(400);     //shot middle 2 flags

  turn_pid(73.5);
  reset_error_globals();

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
  turn_pid(56);
  drive_pid(-28);
  turn_pid(-90);

  intake(120, 2000);
  drive_time(50, 750);
  drive_pid(25.5, 1000);
  //drive_pid(30, 100, 90);


}




//red auton back counter park------------------------------------------------------------------------------------------
else if(red == 5){

  intake_limit(true, 0);
  //flywheel(485, 5000);  //495
  //auto_double_shot(525, 470, 120);
  flywheel(445, 5000);
  drive_pid(26);
  turn_pid(-75, 250);

  pros::delay(300);

  intake(120, 400);
  pros::delay(400);

  flywheel(518, 5000);
  intake_limit(true, 0);
  drive_pid(5);    //collect ball from platform //20 speed

  flipper(true, EXTEND - 200);
  drive_pid(-7);

  pros::delay(1500);

  intake_limit(false, 0);
  flipper(true, REST);

  //pros::delay()
  intake(120, 600);
  pros::delay(600);     //shot middle 2 flags

  turn_pid(74);
  //reset_error_globals();

  flywheel(540, 10000);   //563

  intake_limit(true, 0);
  drive_pid(23);

  pros::delay(500);
  drive_pid(-14); //-12.5
  //reset_error_globals();

  turn_pid(-57);  //55.8
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();
  turn_pid(-33);
  //drive_pid(-28);
  //turn_pid(-90);

  intake(120, 2000);
  drive_time(50, 800);

  flywheel(0, 1000);

  pros::delay(300);
  drive_pid(25.5, 1000);
  // reset_error_globals();
  // reset_drive_encoders();
  // drive_pid_encoder(25);

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




//blue auton back cap v2------------------------------------------------------------------------
else if(blue == 3){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(72.5, 250);

  pros::delay(300);

  intake(120, 400);
  pros::delay(400);

  flywheel(520, 5000);
  intake_limit(true, 0);
  drive_pid(4);    //collect ball from platform //20 speed

  flipper(true, EXTEND - 300);
  drive_pid(-7);

  pros::delay(1500);

  intake_limit(false, 0);
  flipper(true, REST);

  //pros::delay()
  intake(120, 600);
  pros::delay(600);     //shot middle 2 flags

  reset_error_globals();

  turn_pid(17.5);
  //reset_error_globals();

  flywheel(545, 10000);

  intake_limit(true, 0);
  drive_pid(-16.5);

  turn_pid(-105);  //55 //going for back cap

  drive_pid(11.2, 300, 60);  //12.2      //going towards cap
  flipper(true, EXTEND - 300);

  pros::delay(200);
  drive_pid(-6);
  pros::delay(300);
  flipper(true, REST + 500);
  //pros::delay(100);
  drive_pid(-8);

  flipper(true, EXTEND);
  pros::delay(200);

  flipper(true, REST, 14); //flipped cap.
  drive_pid(15);

  timed_flywheel = false;

  flywheel(545, 5000);

  pros::delay(200);

  turn_pid(74); //70

  pros::delay(100);
  intake(120, 350);    //shot middle flag on opponent post
  pros::delay(350);

  flywheel(510, 5000);

  flipper(true, EXTEND);
  intake(120, 1000);
  pros::delay(1000);

  drive_pid(16, 50);
  turn_pid(90);


  // intake(0, 50);
  // intake_limit(true, 0);
  //
  // flipper(true, REST + 600);
  //
  // flywheel(540, 5000);
  //
  // drive_pid(19, 150, 70);
  //
  // pros::delay(100);
  // drive_pid(-5);
  //
  // flipper(true, REST);
  // intake(120, 2000);
  //
  // turn_pid(0);

}




//blue auton back counter cap no parking------------------------------------------------------------------------
else if(blue == 4){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(72.5, 250);

  pros::delay(300);

  intake(120, 400);
  pros::delay(400);

  flywheel(520, 5000);
  intake_limit(true, 0);
  drive_pid(4);    //collect ball from platform //20 speed

  flipper(true, EXTEND - 300);
  drive_pid(-7);

  pros::delay(1500);

  intake_limit(false, 0);
  flipper(true, REST);

  //pros::delay()
  intake(120, 600);
  pros::delay(600);     //shot middle 2 flags

  reset_error_globals();

  turn_pid(-72.5);
  //reset_error_globals();

  flywheel(540, 10000);   //563

  intake_limit(true, 0);
  drive_pid(26.5);

  pros::delay(500);
  drive_pid(-12); //-12.5
  //reset_error_globals();

  turn_pid(56);  //55
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();
//copy of back park basically

  flywheel(507, 8000);
  turn_pid(-116); //-112
  intake_limit(true, 0);
  drive_pid(11);        //going towards cap
  flipper(true, EXTEND);

  drive_pid(-10);
  flipper(true, REST + 500);
  pros::delay(100);
  //drive_pid(-10);

  flipper(true, EXTEND);
  pros::delay(100);

  flipper(true, REST, 10); //flipped cap
  drive_pid(12);

  turn_pid(116); //116

  pros::delay(200);
  intake(120, 3000);    //shot middle flag on opponent post
  pros::delay(3000);
}



//blue auton back counter park------------------------------------------------------------------------------------------
else if(blue == 5){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(72.5, 250);

  pros::delay(300);

  intake(120, 400);
  pros::delay(400);

  flywheel(520, 5000);
  intake_limit(true, 0);
  drive_pid(4);    //collect ball from platform //20 speed

  flipper(true, EXTEND - 300);
  drive_pid(-7);

  pros::delay(1500);

  intake_limit(false, 0);
  flipper(true, REST);

  //pros::delay()
  intake(120, 600);
  pros::delay(600);     //shot middle 2 flags

  reset_error_globals();

  turn_pid(-72.5);
  //reset_error_globals();

  flywheel(540, 10000);   //563

  intake_limit(true, 0);
  drive_pid(26.5);

  pros::delay(500);
  drive_pid(-13.5); //-12.5
  //reset_error_globals();

  turn_pid(56);  //55
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();

// //cap flip version starts--------
//   flywheel(0, 1000);
//   flipper(true, EXTEND);
//   turn_pid(-112);
//
//   flipper(true, REST, 15);
//   drive_pid(20);
//
//   drive_pid(-20);
//   turn_pid(145);
//
//   intake(120, 3000);
//   drive_time(50, 800);
//
//   pros::delay(300);
//   drive_pid(24.5, 1000); //parked
// //-------------------------------------

//version without cap flip ----------
  turn_pid(35);
  //drive_pid(-28);
  //turn_pid(-90);

  intake(120, 2000);
  drive_time(50, 800);

  flywheel(0, 1000);

  pros::delay(300);
  drive_pid(24.5, 1000); //parked
//----------------------------------

}







//skills auton---------------------------------------------------------------------------------------------------------------------------------------------------------

else if(red == 6){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 35); //intake after going 25 inches
  flipper(true, 2000);
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(45, 250);

  pros::delay(400);

  drive_pid(-40, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(-90.6);     //face the flags

  flipper(true, REST);
  drive_pid(45, 400, 90);

  //turn_pid(0, 400);

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(23);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(400);
  flywheel(0, 500);

  turn_pid(-5);

  drive_pid(18);
  //turn_pid(6);
  //swing_fast_pid(18, 5, 150);

  // intake(120, 2000);
  // drive_time(60, 1000); //hit bottom flag, lined up
  // reset_error_globals();

  //turn_pid(20);

  intake(-120, 4000);
  // drive_pid(-17, 100);
  // flipper(true, EXTEND);
  // turn_pid(85);      //turn towards cap

  // flipper(true, REST, 4);
  // drive_pid(7);

  drive_pid(-24);

  turn_pid(140);
  drive_pid(26.5);

  flipper(true, EXTEND - 200);
  flywheel(570, 15000);
  intake_limit(true, 0);
  drive_pid(-4);        //got ball from platform
  pros::delay(1500);

  flipper(true, 2000);

  intake(-120, 500);
  turn_pid(-45);

  pros::delay(400);

  intake_limit(true, 14);
  drive_pid(25);
  flipper(true, REST);

  pros::delay(300);
  turn_pid(-80);

  intake(120, 300);
  pros::delay(300); //shot ball on top flag in middle lane
  drive_pid(22);
  intake(120, 600);
  pros::delay(600); //shot ball on middle flag in middle lane

  // turn_pid(-10);
  // drive_time(70, 1000);

  flipper(true, EXTEND);
  turn_pid(-110);
  flipper(true, REST, 16.5);
  drive_pid(18);

  pros::delay(200);
  turn_pid(160);
  swing_fast_pid(20, -70);

  drive_time(60, 1000);
  reset_drive_encoders();

  //drive_pid(24.5, 1000); //parked

}




//testing -------------------------------------------------------------------------------------------------------------------------------------------------------------

else if(blue == 6){

  // intake_limit(true, 0);
  // pros::delay(2000);
  // intake(-120, 500);

  swing_fast_pid(20, -90);
}

//flywheel(200, 10000);

}

//RED FRONT 5 Flags with double shot backup
// pros::lcd::print(0, "timer 1 %d", pros::millis());
//
// intake_limit(true, 30); //intake after going 25 inches
// //flywheel(570, 10000);   //start flywheel in background //200
// //auto_double_shot(570, 400, 170);
// auto_double_shot(570, 410, 142);
// drive_pid(45, 250);     //get the ball under the cap
//
// //reset_error_globals();
//
// pros::delay(400);
//
// drive_pid(-41.5, 150); //-35    //drive back to starting tile to shoot balls
// turn_pid(-90);     //face the flags
//
// reset_error_globals();
//
// // intake(120, 250);       //first ball shot
// // pros::delay(800);
// // intake(120, 1000);
// // pros::delay(350); //was 400
// intake(120, 1000);
// pros::delay(1000);
//
// turn_pid(-4);
// drive_pid(37);
//
// flywheel(520, 7000);
// flipper(true, EXTEND);
// pros::delay(500);
//
// flipper(true, REST);
// turn_pid(-6);
// drive_pid(-35.5); //-33.5 is on point
// //swing_fast_pid(-20, -40);
//
// turn_pid(56);      //turn towards cap
//
// //auto_flipper(17, EXTEND);S
// drive_pid(13, 250, 90);
// //pros::dela
// flipper(true, EXTEND - 100);
// pros::delay(300);
//
// intake_limit(true, 2);
// drive_pid(-10);
// //drive_pid(-15);
//
// pros::delay(1000);
// auto_double_shot(520, 395, 166); //double shot
//
// intake_limit(false, 0);
//
// //turn_pid(2);
// intake(80, 1000);
// pros::delay(1000);
//
// intake(-90, 2000);
// flywheel(100, 500);
// flipper(true, REST - 400);
// drive_pid(15, 150, 90);
//
// //pros::delay(300);
// turn_pid(90);
// //flywheel(0, 500);
// // drive_pid(-10);
// //
// // flywheel(520, 7000);
// // flipper(true, EXTEND);
// // pros::delay(700);
// // flipper(true, REST, 11);
// // drive_pid(18);    //flip cap



// //back cap v2 backup
// intake_limit(true, 0);
// flywheel(440, 5000);
// drive_pid(22.5);
// turn_pid(72.5, 250);
//
// pros::delay(300);
//
// intake(120, 400);
// pros::delay(400);
//
// flywheel(520, 5000);
// intake_limit(true, 0);
// drive_pid(4);    //collect ball from platform //20 speed
//
// flipper(true, EXTEND - 300);
// drive_pid(-7);
//
// pros::delay(1500);
//
// intake_limit(false, 0);
// flipper(true, REST);
//
// //pros::delay()
// intake(120, 600);
// pros::delay(600);     //shot middle 2 flags
//
// reset_error_globals();
//
// turn_pid(17.5);
// //reset_error_globals();
//
// flywheel(515, 10000);
//
// intake_limit(true, 0);
// drive_pid(-17);
//
// turn_pid(-90);  //55 //going for back cap
//
// drive_pid(13);        //going towards cap
// flipper(true, EXTEND - 300);
//
// drive_pid(-6);
// pros::delay(300);
// flipper(true, REST + 500);
// //pros::delay(100);
// drive_pid(-8);
//
// flipper(true, EXTEND);
// pros::delay(200);
//
// //drive_pid(13);
//
// turn_pid(-7);
// flipper(true, REST, 11); //flipped cap.
// drive_pid(13);
//
// //auto_double_shot(540, 490, 120);
// turn_pid(64.2); //116
//
// pros::delay(100);
// intake(120, 1000);    //shot middle flag on opponent post
// pros::delay(400);
//
// intake(0, 50);
// intake_limit(true, 0);
//
// flipper(true, REST + 600);
//
// flywheel(540, 5000);
//
// drive_pid(19, 150, 70);
//
// pros::delay(100);
// drive_pid(-5);
//
// flipper(true, REST);
// intake(120, 2000);
//
// turn_pid(0);
