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

// blue = 0;
// red = 7;


//red auton front park------------------------------------------------------------------------
if(red == 1){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(44, 250);

  pros::delay(200);

  drive_pid(-41, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(-90.5);     //face the flags

  //reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(24);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(500);
  flywheel(100, 500);

  turn_pid(-10);
  drive_pid(15);

  // flipper(true, EXTEND);
  // pros::delay(300);
  // flipper(true, REST);

  turn_pid(10);
  intake(-80, 1000);
  drive_pid(-16.5, 100);
  flipper(true, EXTEND);
  turn_pid(90);      //turn towards cap
  //reset_error_globals();

  //intake(-100, 2000);
  flipper(true, REST + 600, 13);
  drive_pid(19.5, 100, 90); //31    //flip cap 1

  pros::delay(200);
  turn_pid(65);
  flipper(true, REST);
  reset_error_globals();
  swing_fast_pid(25, 23, 200);

  intake(120, 2500);
  drive_time(50, 800);

  pros::delay(200);

  reset_error_globals();
  drive_pid(24.5, 1000); //parked

  pros::lcd::print(1, "timer 2 %d", pros::millis());

}



//red auton front with 5 flags counter------------------------------------------------------------------------
else if(red == 2){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(44, 250);

  pros::delay(200);

  drive_pid(-41, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(-90.5);     //face the flags

  //reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(24);   //second ball shot
  intake(120, 600); //2000

  pros::delay(500);
  //flywheel(0, 5000);
  //flipper(true, REST - 500);

  intake_limit(true, 0);
  flywheel(525, 10000);
  turn_pid(-10);
  swing_fast_pid(15, 10, 150, 0.15, 150);
  //drive_pid(14);

  drive_time(50, 900);
  //reset_error_globals();

  flipper(true, REST);
  //flywheel(555, 7000);
  drive_pid(-22);

  pros::delay(200);
  intake_limit(false, 0);
  intake(-120, 550);
  //flipper(true, EXTEND);
  turn_pid(90);      //turn towards cap
  //reset_error_globals();

  //intake(-100, 2000);
  flipper(true, EXTEND - 300, 4);
  drive_pid(7, 100, 90); //31    //flip cap 1

  //pros::delay(50);
  drive_pid(-7);

  pros::delay(50);
  intake_limit(true, 0);
  pros::delay(400);
  auto_double_shot(520, 500, 0);

  turn_pid(-13);

  intake_limit(false, 0);
  intake(120, 500);
  pros::delay(500);
  intake(120, 700);
  pros::delay(700);

  intake(-120, 4000);
  turn_pid(22);
  //intake(-120, 4000);

  drive_pid(19, 150, 90);

  pros::lcd::print(1, "timer 2 %d", pros::millis());
}



//red auton front with 5 flags middle flags------------------------------------------------------------------------
else if(red == 3){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(44, 250);

  pros::delay(200);

  drive_pid(-41, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(-90.5);     //face the flags

  //reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(24);   //second ball shot
  intake(120, 600); //2000

  pros::delay(500);
  //flywheel(0, 5000);
  //flipper(true, REST - 500);

  intake_limit(true, 0);
  flywheel(510, 10000);
  turn_pid(-10);
  swing_fast_pid(15, 10, 150, 0.15, 150);
  //drive_pid(14);

  drive_time(50, 900);
  //reset_error_globals();

  flipper(true, REST);
  //flywheel(555, 7000);
  drive_pid(-22);

  pros::delay(200);
  intake_limit(false, 0);
  intake(-120, 550);
  //flipper(true, EXTEND);
  turn_pid(90);      //turn towards cap
  //reset_error_globals();

  //intake(-100, 2000);
  flipper(true, EXTEND - 300, 4);
  drive_pid(7, 100, 90); //31    //flip cap 1

  //pros::delay(50);
  drive_pid(-7);

  pros::delay(50);
  intake_limit(true, 0);
  pros::delay(400);
  auto_double_shot(510, 410, 130);

  turn_pid(-28);

  intake_limit(false, 0);
  intake(120, 500);
  pros::delay(500);
  intake(120, 700);
  pros::delay(700);

  intake(-120, 4000);
  turn_pid(37);
  //intake(-120, 4000);

  drive_pid(19.5, 150, 90);

  pros::lcd::print(1, "timer 2 %d", pros::millis());
}



//red auton back cap v1------------------------------------------------------------------------
else if(red == 4){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(-74, 250);

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

  turn_pid(-16);
  reset_error_globals();

  flywheel(545, 10000);

  intake_limit(true, 0);
  drive_pid(-16.5);

  turn_pid(105);  //55 //going for back cap

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

  turn_pid(-75.5); //70

  pros::delay(100);
  intake(120, 350);    //shot middle flag on opponent post
  pros::delay(350);

  flywheel(510, 5000);

  flipper(true, EXTEND);
  intake(120, 1000);
  pros::delay(1000);

  drive_pid(16, 50);
  turn_pid(-90);

}



//red auton back counter cap v2------------------------------------------------------------------------
else if(red == 5){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(-74, 250);

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

  turn_pid(74);
  //reset_error_globals();

  flywheel(540, 10000);   //563

  intake_limit(true, 0);
  drive_pid(26.5);

  pros::delay(500);
  drive_pid(-12); //-12.5
  //reset_error_globals();

  turn_pid(-56);  //55
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  //reset_error_globals();
//copy of back park basically

  flywheel(507, 8000);
  turn_pid(116); //-112
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

  turn_pid(-117); //116

  pros::delay(200);
  intake(120, 3000);    //shot middle flag on opponent post
  pros::delay(3000);
}



//red auton back counter park------------------------------------------------------------------------------------------
else if(red == 6){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(-74, 250);

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

  turn_pid(74);
  //reset_error_globals();

  flywheel(540, 10000);   //563

  intake_limit(true, 0);
  drive_pid(26.5);

  pros::delay(500);
  drive_pid(-13.5); //-12.5

  turn_pid(-56);  //55
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();

  turn_pid(-35);

  intake(120, 2000);
  drive_time(50, 800);

  flywheel(0, 1000);

  pros::delay(300);
  drive_pid(24.5, 1000); //parked

}







//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

//BLUE SIDE CODE BEGINS







//blue auton front park------------------------------------------------------------------------
else if(blue == 1){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(44, 250);

  pros::delay(200);

  drive_pid(-41, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(90.5);     //face the flags

  //reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(24);   //second ball shot
  intake(120, 1500); //2000

  pros::delay(500);
  flywheel(100, 500);

  turn_pid(10);
  drive_pid(15);

  // flipper(true, EXTEND);
  // pros::delay(300);
  // flipper(true, REST);

  turn_pid(-10);
  intake(-80, 1000);
  drive_pid(-16.5, 100);
  flipper(true, EXTEND);
  turn_pid(-90);      //turn towards cap
  //reset_error_globals();

  //intake(-100, 2000);
  flipper(true, REST + 600, 13);
  drive_pid(19.5, 100, 90); //31    //flip cap 1

  pros::delay(200);
  turn_pid(-65);
  flipper(true, REST);
  reset_error_globals();
  swing_fast_pid(25, -23, 200);

  intake(120, 2500);
  drive_time(50, 800);

  pros::delay(200);

  reset_error_globals();
  drive_pid(24.5, 1000); //parked

  pros::lcd::print(1, "timer 2 %d", pros::millis());

}



//blue auton front with 5 flags counter------------------------------------------------------------------------
else if(blue == 2){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(44, 250);

  pros::delay(200);

  drive_pid(-41, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(90.5);     //face the flags

  //reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(-24);   //second ball shot
  intake(120, 600); //2000

  pros::delay(500);
  //flywheel(0, 5000);
  //flipper(true, REST - 500);

  intake_limit(true, 0);
  flywheel(525, 10000);
  turn_pid(10);
  swing_fast_pid(15, -10, 150, 0.15, 150);
  //drive_pid(14);

  drive_time(50, 900);
  //reset_error_globals();

  flipper(true, REST);
  //flywheel(555, 7000);
  drive_pid(-22);

  pros::delay(200);
  intake_limit(false, 0);
  intake(-120, 550);
  //flipper(true, EXTEND);
  turn_pid(-90);      //turn towards cap
  //reset_error_globals();

  //intake(-100, 2000);
  flipper(true, EXTEND - 300, 4);
  drive_pid(7, 100, 90); //31    //flip cap 1

  //pros::delay(50);
  drive_pid(-7);

  pros::delay(50);
  intake_limit(true, 0);
  pros::delay(400);
  auto_double_shot(520, 500, 0);

  turn_pid(13);

  intake_limit(false, 0);
  intake(120, 500);
  pros::delay(500);
  intake(120, 700);
  pros::delay(700);

  intake(-120, 4000);
  turn_pid(-22);
  //intake(-120, 4000);

  drive_pid(19, 150, 90);

  pros::lcd::print(1, "timer 2 %d", pros::millis());
}



//blue auton front with 5 flags middle flags------------------------------------------------------------------------
else if(blue == 3){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 30); //intake after going 25 inches
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(44, 250);

  pros::delay(200);

  drive_pid(-41, 150); //-35    //drive back to starting tile to shoot balls
  turn_pid(90.5);     //face the flags

  //reset_error_globals();

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(400);

  drive_pid(24);   //second ball shot
  intake(120, 600); //2000

  pros::delay(500);
  //flywheel(0, 5000);
  //flipper(true, REST - 500);

  intake_limit(true, 0);
  flywheel(510, 10000);
  turn_pid(10);
  swing_fast_pid(15, -10, 150, 0.15, 150);
  //drive_pid(14);

  drive_time(50, 900);
  //reset_error_globals();

  flipper(true, REST);
  //flywheel(555, 7000);
  drive_pid(-22);

  pros::delay(200);
  intake_limit(false, 0);
  intake(-120, 550);
  //flipper(true, EXTEND);
  turn_pid(-90);      //turn towards cap
  //reset_error_globals();

  //intake(-100, 2000);
  flipper(true, EXTEND - 300, 4);
  drive_pid(7, 100, 90); //31    //flip cap 1

  //pros::delay(50);
  drive_pid(-7);

  pros::delay(50);
  intake_limit(true, 0);
  pros::delay(400);
  auto_double_shot(510, 410, 130);

  turn_pid(28);

  intake_limit(false, 0);
  intake(120, 500);
  pros::delay(500);
  intake(120, 700);
  pros::delay(700);

  intake(-120, 4000);
  turn_pid(-37);
  //intake(-120, 4000);

  drive_pid(19.5, 150, 90);

  pros::lcd::print(1, "timer 2 %d", pros::millis());
}


//blue auton back cap v1------------------------------------------------------------------------
else if(blue == 4){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(74, 250);

  pros::delay(300);

  intake(120, 400);
  pros::delay(400);

  flywheel(520, 5000);
  intake_limit(true, 0);
  drive_pid(4);

  flipper(true, EXTEND - 300);
  drive_pid(-7);

  pros::delay(1500);

  intake_limit(false, 0);
  flipper(true, REST);

  //pros::delay()
  intake(120, 600);
  pros::delay(600);     //shot middle 2 flags

  reset_error_globals();

  turn_pid(16);
  reset_error_globals();

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

  turn_pid(75.5); //70

  pros::delay(100);
  intake(120, 350);    //shot middle flag on opponent post
  pros::delay(350);

  flywheel(510, 5000);

  flipper(true, EXTEND);
  intake(120, 1000);
  pros::delay(1000);

  drive_pid(16, 50);
  turn_pid(90);

}



//blue auton back counter cap v2------------------------------------------------------------------------
else if(blue == 5){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(74, 250);

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

  turn_pid(-74);
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

  //reset_error_globals();
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

  turn_pid(117); //116

  pros::delay(200);
  intake(120, 3000);    //shot middle flag on opponent post
  pros::delay(3000);
}



//blue auton back counter park------------------------------------------------------------------------------------------
else if(blue == 6){

  intake_limit(true, 0);
  flywheel(440, 5000);
  drive_pid(22.5);
  turn_pid(74, 250);

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

  turn_pid(-74);
  //reset_error_globals();

  flywheel(540, 10000);   //563

  intake_limit(true, 0);
  drive_pid(26.5);

  pros::delay(500);
  drive_pid(-13.5); //-12.5

  turn_pid(56);  //55
  pros::delay(600);
  intake(120, 600);   //counter pop the flag

  pros::delay(600);

  reset_error_globals();

  turn_pid(35);

  intake(120, 2000);
  drive_time(50, 800);

  flywheel(0, 1000);

  pros::delay(300);
  drive_pid(24.5, 1000); //parked

}






//skills auton---------------------------------------------------------------------------------------------------------------------------------------------------------

else if(red == 7){

  pros::lcd::print(0, "timer 1 %d", pros::millis());

  intake_limit(true, 35); //intake after going 25 inches
  flipper(true, FLIP);
  flywheel(570, 10000);   //start flywheel in background //200
  drive_pid(45, 250);

  pros::delay(300);

  drive_pid(-42); //-35    //drive back to starting tile to shoot balls
  turn_pid(-91);     //face the flags

  flipper(true, REST);
  drive_pid(45, 250, 90);

  //turn_pid(0, 400);

  intake(120, 300);       //first ball shot
  //turn_pid(0, 250);
  pros::delay(300);

  drive_pid(25);   //second ball shot
  pros::delay(50);
  intake(120, 1500); //2000

  pros::delay(400);
  flywheel(0, 500);

  intake(120, 2500);
  turn_pid(-7);

  drive_pid(12);
  intake(-120, 2000);
  drive_pid(-26.8);

  turn_pid(145.5);
  intake_limit(true, 0);
  flipper(true, EXTEND - 200, 21.2);
  drive_pid(25.2);

  flywheel(560, 15000);
  pros::delay(300);
  drive_pid(-7.2);        //got ball from platform
  pros::delay(1300);//1500

  flipper(true, FLIP);

  intake(-120, 500);
  turn_pid(-50);

  pros::delay(250); //400

  intake_limit(true, 15);
  drive_pid(25.5);
  flipper(true, REST);

  pros::delay(300);
  turn_pid(-81.5);

  pros::delay(100);
  intake(120, 300);
  pros::delay(300); //shot ball on top flag in middle lane
  drive_pid(20, 150, 90);

  pros::delay(50);
  intake(120, 2500);
  pros::delay(800); //shot ball on middle flag in middle lane

  flywheel(0, 500);

  flipper(true, EXTEND);
  turn_pid(-110);
  flipper(true, REST, 13);
  drive_pid(15, 150, 90);

  pros::delay(300);
  flipper(true, REST);
  turn_pid(170);
  swing_fast_pid(7.2, -68, 50, 0.35);

  intake(100, 2000);

  drive_time(60, 1500);       //lined up in middle flags lane
  reset_error_globals();

  intake(-120, 2000);
  drive_pid(-30);
  flipper(true, EXTEND);
  turn_pid(90.5);

  pros::delay(200);

  flipper(true, REST, 45);
  drive_pid(47.5);

  flywheel(580, 15000);
  flipper(true, REST);
  turn_pid(102);
  intake_limit(true, 0);

  flipper(true, EXTEND - 200, 21);
  drive_pid(26);

  pros::delay(300);
  drive_pid(-4);        //got ball from platform blue
  pros::delay(1400);

  flipper(true, FLIP);  //put flipper in the FLIP position

  intake(-120, 500);
  turn_pid(78);
  pros::delay(300);

  intake_limit(true, 17.5);
  drive_pid(25.5);
  flipper(true, REST);  //filp cap with the ball under it

  pros::delay(300);

  drive_pid(-8);
  pros::delay(100);

  turn_pid(-10);
  drive_pid(-22);
  turn_pid(100.5);

  pros::delay(100);
  intake(120, 300);
  pros::delay(300); //shot top flag on blue side

  drive_pid(20, 250, 90);

  pros::delay(150);
  intake(120, 700);
  pros::delay(700); //shot middle flag on blue side

  flywheel(0, 10000);
  flipper(true, REST - 400);
  turn_pid(-12);
  swing_fast_pid(12, 12, 150, 0.25, 700);

  intake(120, 2000); //line up again on blue side
  drive_time(50, 1200);
  reset_error_globals();

  intake(-120, 2000);

  // swing_fast_pid(-30, -33, 150, 0.32, 650);  //going towards far section
  // swing_fast_pid(-21, 33, 150, 0.32, 650);

  drive_pid(-34);
  turn_pid(-33);
  drive_pid(-24);
  turn_pid(33);

  drive_pid(-46);
  turn_pid(-91.5);   //facing cap in back section of blue

  flipper(true, FLIP);

  flywheel(150, 15000);
  drive_pid(38.2);

  flipper(true, REST);
  intake_limit(true, 10);
  drive_pid(-37);
  //intake(120, 15000);
  turn_pid(90);
  drive_pid(20.3);
  turn_pid(-90);            //turned into the platform

  drive_time(50, 1200);
  reset_error_globals();

  drive_pid(22);  //19
  reset_error_globals();

  //pros::delay(1000);
  intake_flip(false, 0, 0, 0, false);
  intake(120, 5000);
  drive_time(50, 500);
  reset_error_globals();

  drive_pid(24, 1000);    //parked
  intake(0, 1500);


}




//testing -------------------------------------------------------------------------------------------------------------------------------------------------------------

else if(blue == 7){

}


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


//5 flag auton backup with 5 flags but no line up or double shot
// pros::lcd::print(0, "timer 1 %d", pros::millis());
//
// intake_limit(true, 30); //intake after going 25 inches
// flywheel(570, 10000);   //start flywheel in background //200
// drive_pid(45, 250);
//
// pros::delay(400);
//
// drive_pid(-42, 150); //-35    //drive back to starting tile to shoot balls
// turn_pid(-90.5);     //face the flags
//
// reset_error_globals();
//
// intake(120, 300);       //first ball shot
// //turn_pid(0, 250);
// pros::delay(400);
//
// drive_pid(25);   //second ball shot
// intake(120, 1500); //2000
//
// pros::delay(500);
// flywheel(100, 500);
//
// turn_pid(-10);
// drive_pid(14);
//
// intake(-80, 1000);
//
// flywheel(520, 7000);
//
// drive_pid(-34); //-33.5 is on point
// //swing_fast_pid(-20, -40);
//
// turn_pid(58);      //turn towards cap
//
// //auto_flipper(17, EXTEND);S
// flipper(true, EXTEND - 100, 10);
// drive_pid(14, 250, 100);
// //pros::dela
// pros::delay(100);
//
// intake_limit(true, 2);
// drive_pid(-10);
// //drive_pid(-15);
//
// pros::delay(1000);
// auto_double_shot(520, 407, 170); //double shot
//
// intake_limit(false, 0);
//
// turn_pid(3);
// intake(90, 1200);
// pros::delay(1100);
//
// intake(-100, 2000);
// flywheel(0, 1000);
// flywheel(100, 500);
// flipper(true, REST - 400);
// drive_pid(18, 150, 90);
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
