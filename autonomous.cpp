#include "main.h"
#include <stdio.h>
#include "okapi/api.hpp"
bool blue = true;
bool risky= true;
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
pros::Motor aleft_wheels (LEFT_WHEELS_PORT,true);
pros::Motor aleft_wheels_2 (LEFT_WHEELS_PORT_2,true);
pros::Motor aright_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor aright_wheels_2 (RIGHT_WHEELS_PORT_2);
pros::Motor aintake_L (INTAKE_PORT_L);
pros::Motor aintake_R (INTAKE_PORT_R);
pros::Motor alift (LIFT_PORT,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor aouttake (OUTTAKE_PORT,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
auto myChassis = ChassisControllerFactory::create(
  {-2, 6}, // Left motors
  {-5, 3},   // Right motors
  AbstractMotor::gearset::red, // Torque gearset
  {4_in, 12.5_in} // 4 inch wheels, 12.5 inch wheelbase width
);
auto mp = AsyncControllerFactory::motionProfile(
  1.0,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  10.0, // Maximum linear jerk of the Chassis in m/s/s/s
  myChassis // Chassis Controller
);
void driveForward(int inc){
  IntegratedEncoder drive = IntegratedEncoder(aleft_wheels);
  drive.reset();
  for(;drive.get()>=-inc*tpi;){
    aleft_wheels.move(-50);
    aright_wheels.move(50);
    aleft_wheels_2.move(50);
    aright_wheels_2.move(50);
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
bool aouttake_macro(bool dir,IntegratedEncoder enc){//false for reverse
  if(!dir){
    if(enc.get()>=-6200){
      if(enc.get()<-4500){
        aouttake.move(-85);
      }
      else{
        aouttake.move(-127);
      }
      printf("%f\n",enc.get());
      return true;
    }
    else{
      return false;
    }
  }
  else{
    if(enc.get()<=0){
      aouttake.move(127);
      return true;
    }
    else{
      return false;
    }
  }
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
    if(!risky){
      driveForward(5);
      aintake_L.move(-127);
      aintake_R.move(127);
    }
    else{
      aintake_R.move(-127);
      aintake_L.move(127);
      driveForward(56);
      pros::delay(200);
      turn(27);
      driveBackward(-60);
      driveBackward(-12);
      driveForward(58);
      pros::delay(400);
      driveBackward(-60);
      driveForward(7);
      rturn(-67);
      aintake_L.move(35);
      aintake_R.move(-35);
      driveForward(12);
      aintake_L.move(0);
      aintake_R.move(0);
      bool keep = aouttake_macro(false,IntegratedEncoder(aouttake));
      while (keep){
        keep = aouttake_macro(false,IntegratedEncoder(aouttake));
      }
      aintake_L.move(-25);
      aintake_R.move(25);
      driveBackward(-5);
      keep = aouttake_macro(true,IntegratedEncoder(aouttake));
      while (keep){
        keep = aouttake_macro(true,IntegratedEncoder(aouttake));
      }
      aouttake.move(0);
      //mp.generatePath({Point{3.5_ft, 2_ft, 0_deg}, Point{0_ft, 0_ft, 0_deg}}, "spline");
      //mp.setTarget("spline");
      //mp.waitUntilSettled();
    }
}
