#ifndef ALLINIT_H_
#define ALLINIT_H_

#include "main.h"

//ports and misc init

  //KEY values
    #define pi 3.14159265359

  //GEARSET RPM AND SETTING
    #define RPM100_GEARSET 300  //number means the amount of ticks
    #define RPM200_GEARSET 900
    #define RPM600_GEARSET 1800

  //motor distribution

    #define DRIVE_LEFT_F_PORT 9
    #define DRIVE_LEFT_B_PORT 10
    #define DRIVE_RIGHT_F_PORT 2
    #define DRIVE_RIGHT_B_PORT 1

    #define FLYWHEEL_FRONT_PORT 8  //port 20 broken

    #define INTAKE_PORT 3
    #define LIFT_PORT 7 //15, 16 & 18 broken most likely
    #define FLIPPER_PORT 4

    #define TEST_PORT_MOTOR 20

  //sensors
    #define POT_PORT 1 //1 is weird
    #define LIGHT_SENSOR_PORT 9
    #define LIGHT_SENSOR_FLYWHEEL_PORT 2
    #define LIMIT_SWITCH_PORT 4 //3 if doing the gyro fix thing
    #define GYRO_PORT 3 //2 is weird

    #define LEFT_ENC_PORT_TOP 7
    #define LEFT_ENC_PORT_BOT 8
    #define RIGHT_ENC_PORT_TOP 5
    #define RIGHT_ENC_PORT_BOT 6


    #define PORT_ECHO 10
    #define PORT_PING 11

  //reverse or not

    // //vision
    // #define VISION_PORT 15
    // #define RED 1
    // #define BLUE 2


    //reverse or not

    #define REVERSE 1
    #define NOT_REVERSE 0

  //  int timer;

    #define MAX_NUM_MOTORS 21

//sensors

    extern pros::ADIGyro gyro;
    extern pros::ADIUltrasonic sonar;

    extern pros::ADIDigitalIn limit_switch;
    extern pros::ADIAnalogIn light;
    extern pros::ADIAnalogIn pot;
    extern pros::ADIAnalogIn light_flywheel;

    extern pros::ADIEncoder encoder_left;
    extern pros::ADIEncoder encoder_right;


//initialize motors

    extern pros::Motor drive_left_b;
    extern pros::Motor drive_left_f;
    extern pros::Motor drive_right_b;
    extern pros::Motor drive_right_f;

    extern pros::Motor flywheel_f;

    extern pros::Motor intake_motor;
    extern pros::Motor lift_motor;
    extern pros::Motor flipper_motor;

    extern pros::Motor test_motor;

//controller

    extern pros::Controller master;


#endif
