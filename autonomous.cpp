#include "main.h"
#include <stdio.h>
#include "okapi/api.hpp"
bool red = true;
using namespace okapi;
#define tpi 8.8188038732
#define tpdeg 0.82
#define LEFT_WHEELS_PORT 2
#define LEFT_WHEELS_PORT_2 6
#define RIGHT_WHEELS_PORT 5
#define RIGHT_WHEELS_PORT_2 3
#define INTAKE_PORT_L 8
#define INTAKE_PORT_R 7
#define OUTTAKE_PORT 20
#define LIFT_PORT 9
#define VISION_PORT 17
pros::Motor aleft_wheels (LEFT_WHEELS_PORT);
pros::Motor aleft_wheels_2 (LEFT_WHEELS_PORT_2,true);
pros::Motor aright_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor aright_wheels_2 (RIGHT_WHEELS_PORT_2);
pros::Motor aintake_L (INTAKE_PORT_L);
pros::Motor aintake_R (INTAKE_PORT_R);
pros::Motor alift (LIFT_PORT,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor aouttake (OUTTAKE_PORT,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);

void driveForward(int inc){
  IntegratedEncoder drive = IntegratedEncoder(aleft_wheels);
  drive.reset();
  for(;drive.get()>=-inc*tpi;){
    aleft_wheels.move(-80);
    aright_wheels.move(80);
    aleft_wheels_2.move(80);
    aright_wheels_2.move(80);
    pros::delay(10);
    printf("%fd\n",drive.get());
  }
    aleft_wheels.move(0);
    aright_wheels.move(0);
    aleft_wheels_2.move(0);
    aright_wheels_2.move(0);
}
void driveBackward(int inc){
  IntegratedEncoder drive = IntegratedEncoder(aleft_wheels);
  drive.reset();
  for(;drive.get()<=-inc*tpi;){
    aleft_wheels.move(80);
    aright_wheels.move(-80);
    aleft_wheels_2.move(-80);
    aright_wheels_2.move(-80);
    pros::delay(10);
    printf("%fd\n",drive.get());
  }
    aleft_wheels.move(0);
    aright_wheels.move(0);
    aleft_wheels_2.move(0);
    aright_wheels_2.move(0);
}
void turn(int deg){
  IntegratedEncoder drive = IntegratedEncoder(aleft_wheels);
  drive.reset();
  for(;drive.get()>=-tpdeg*deg;){
    aleft_wheels.move(-60);
    aright_wheels.move(-60);
    aleft_wheels_2.move(60);
    aright_wheels_2.move(-60);
    pros::delay(10);
    printf("%ft\n",drive.get());
  }
    aleft_wheels.move(0);
    aright_wheels.move(0);
    aleft_wheels_2.move(0);
    aright_wheels_2.move(0);
}
void rturn(int deg){
  IntegratedEncoder drive = IntegratedEncoder(aleft_wheels);
  drive.reset();
  for(;drive.get()<=-tpdeg*deg;){
    aleft_wheels.move(60);
    aright_wheels.move(60);
    aleft_wheels_2.move(-60);
    aright_wheels_2.move(60);
    pros::delay(10);
    printf("%ft\n",drive.get());
  }
    aleft_wheels.move(0);
    aright_wheels.move(0);
    aleft_wheels_2.move(0);
    aright_wheels_2.move(0);
}
void lift(bool dir){//false for reverse
  IntegratedEncoder enc = IntegratedEncoder(alift);
  enc.reset();
  if(dir){
    while(enc.get()<=200){
      alift.move(127);
    }
  }
  else{
    while(enc.get()>=-200){
      alift.move(-127);
    }
  }
  alift.move(0);
}
void aouttake_macro(bool dir){//false for reverse
  IntegratedEncoder enc = IntegratedEncoder(aouttake);
  enc.reset();
  if(dir){
    while(enc.get()<=12000){
      aouttake.move(127);
    }
  }
  else{
    while(enc.get()>=-12000){
      aouttake.move(-127);
    }
  }
  aouttake.move(0);
}
void autonomous(){

    aouttake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    aintake_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    aintake_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    aleft_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    aright_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    aleft_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    aright_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift(true);
    lift(false);
    lift(true);
    lift(false);
    aintake_L.move(127);
    aintake_R.move(-127);
    driveForward(50);
    pros::delay(200);
    if(red){
      turn(36);
    }
    else{
      rturn(-36);
    }
    driveBackward(-55);
    pros::delay(200);
    if(red){
      rturn(-36);
    }
    else{
      turn(36);
    }
    driveForward(50);
    pros::delay(200);
    driveBackward(-40);
    pros::delay(200);
    if(red){
      rturn(-130);
    }
    else {
      turn(130);
    }
    aintake_L.move(30);
    aintake_R.move(-30);
    driveForward(10);
    pros::delay(100);

    aintake_L.move(-30);
    aintake_R.move(30);
    aouttake_macro(false);
}
