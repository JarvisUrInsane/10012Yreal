/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// WheelLF              motor         13              
// WheelLM              motor         12              
// WheelLB              motor         11              
// WheelRB              motor         18              
// WheelRM              motor         19              
// WheelRF              motor         20              
// IMU                  inertial      17              
// Intake               motor         14              
// Flywheel             motor         15              
// expansion1           digital_out   A               
// expansion2           digital_out   B               
// AngleAdjuster        digital_out   C               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream> 
#include <string>
#include <string.h>
#include <cmath>

using namespace vex;

competition Competition;


//use to calculate the wheel size
const double pi = 3.14159;
const double wheelSize = 3.25;

int inchesToDegrees(double inches){
  return 360*inches/(wheelSize*pi);
}

//PID variable
bool PID_enable = true;
bool PID_reset = true;

double KP=0.11,KD=0.0002,KI=0.000;
const double originalKP = KP;
const double originalKD = KD;
double error=0,prevError=0,totalError=0;
int desiredPosition=0;

double intertial_KP = 0.52, intertial_KD=0.0000,intertial_KI=0.000;
double intertial_error=0,intertial_prevError=0,intertial_totalError=0;
int desiredDirection=0;


int PID_running(){
  WheelLF.spin(forward);
  WheelLM.spin(forward);
  WheelLB.spin(forward);
  WheelRF.spin(forward);
  WheelRM.spin(forward);
  WheelRB.spin(forward);

  while(PID_enable){
    if(PID_reset){
      PID_reset = false;
      WheelLF.setPosition(0,degrees);
      WheelLM.setPosition(0, degrees);
      WheelLB.setPosition(0,degrees);
      WheelRF.setPosition(0,degrees);
      WheelRM.setPosition(0, degrees);
      WheelRB.setPosition(0,degrees);
    }
    int currentPosition;
    currentPosition = (WheelLF.position(degrees)+WheelLM.position(degrees)+WheelLB.position(degrees)+WheelRF.position(degrees)+WheelRM.position(degrees)+WheelRB.position(degrees))/6;
    error = desiredPosition - currentPosition;
    totalError += error;
    int movePower = KP * error + KI * (error - prevError) + KD * totalError;

    intertial_error = desiredDirection - IMU.rotation(degrees);
    intertial_totalError += intertial_error;
    int turnPower = intertial_KP *  intertial_error + intertial_KI * (intertial_error - intertial_prevError) + intertial_KD * intertial_totalError;
    WheelLF.setVelocity(movePower + turnPower, percent);
    WheelLM.setVelocity(movePower + turnPower, percent);
    WheelLB.setVelocity(movePower + turnPower, percent);
    WheelRF.setVelocity(movePower - turnPower, percent);
    WheelRM.setVelocity(movePower - turnPower, percent);
    WheelRB.setVelocity(movePower - turnPower, percent);

    prevError=error;
    intertial_prevError=intertial_error;

    task::sleep(20);
  }
  return 0;
}

void resetPID(){
  PID_reset = true;
  desiredPosition = 0;
  desiredDirection = IMU.rotation(degrees);
}

void move(double d,int t){
  resetPID();
  desiredPosition = inchesToDegrees(d);
  task::sleep(t);
  resetPID();
}

void turnTo(int d, int t){
  resetPID();
  desiredDirection = d;
  task::sleep(t);
  resetPID();
}

bool flywheelOn=false; 
bool arcadeDrive = true;
bool angleAdjuster = false;
void flywheelSwitch(){
  flywheelOn = !flywheelOn;
}
void switchDrive(){
  arcadeDrive = !arcadeDrive;
}
void angleAdjusterSwitch(){
  angleAdjuster = !angleAdjuster;
}



int test(){
  while(1){
    // Brain.Screen.print(WheelLF.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelLM.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelLB.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelRF.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelRM.position(degrees));
    // Brain.Screen.print(" ");
    // Brain.Screen.print(WheelRB.position(degrees));
    // Brain.Screen.print(" ");
    Brain.Screen.print(IMU.heading(degrees));
    task::sleep(200);
    Brain.Screen.clearLine();
  }
  return 0;
}
bool flywheelState = false;
bool intakeState = false;
bool indexerState = false;
bool slowMode = false;
bool die = false;
int robotDaemon() {
  while(1){
    if (flywheelState) {
              Flywheel.spin(forward);
            } else {
              //Flywheel.stop();
            }
            if (intakeState) {
              Intake.spin(forward);
            } 
            else if (indexerState) {
              Intake.spin(reverse);
            } else {
              Intake.stop();
              Brain.Screen.print("I STOPPED");
            }
            if(slowMode){
              KP = originalKP/2.5;
              KD = 0;

            } else{
              KP=originalKP;
              KD=originalKD;
            }
            if(die){
              break;
            }
            task::sleep(10);
     

  }
  return 0;
}

void preAuton(){
  expansion1.set(false);
  expansion2.set(false);
  AngleAdjuster.set(false);

  WheelLF.setPosition(0,degrees);
  WheelLM.setPosition(0,degrees);
  WheelLB.setPosition(0,degrees);
  WheelRF.setPosition(0,degrees);
  WheelRM.setPosition(0,degrees);
  WheelRB.setPosition(0,degrees);

  Intake.setMaxTorque(100, percent);
  Intake.setVelocity(100, percent);

  WheelLF.setStopping(brake);
  WheelLM.setStopping(brake);
  WheelLB.setStopping(brake);
  WheelRF.setStopping(brake);
  WheelRM.setStopping(brake);
  WheelRB.setStopping(brake);

  Flywheel.setMaxTorque(100, percent);
  Flywheel.setVelocity(100, percent);
  PID_enable = true;
  Flywheel.setStopping(coast);
  
}
void shoot3(){
  indexerState = true;
  vex::task::sleep(150);
  indexerState = false;
  vex::task::sleep(600);
  indexerState = true;
  vex::task::sleep(150);
  indexerState = false;
  vex::task::sleep(600);
  indexerState = true;
  vex::task::sleep(200);
  indexerState = false;
}


int endgameRumble(){
  while(69){
      if(Brain.Timer.value()>=135 && Brain.Timer.value()<=142){
        Controller1.rumble(".");
      }
      if(Brain.Timer.value()>=142 && Brain.Timer.value()<=145){
        Controller1.rumble("-");
      }
      task::sleep(20);
  }
  return 0;
}


void Autonomous(){
  preAuton();
  task PID_task = task(PID_running);
  task run_Daemon = task(robotDaemon);
  //task testing = task(test);

  Brain.Timer.reset();
  expansion1.set(false);
  expansion2.set(false);
  AngleAdjuster.set(false);

//Auto in front
if(false){
      Flywheel.setVelocity(70, percent);
      flywheelState = true;
      move(2, 500);
      intakeState = true;
      vex::task::sleep(280);
      intakeState = false;
      move(-4, 600);

      turnTo(-11, 1500);
      indexerState = true;
      vex::task::sleep(150);
      indexerState = false;
      Flywheel.setVelocity(72, percent);
      vex::task::sleep(700);
      indexerState = true;
      vex::task::sleep(250);
      indexerState = false;

      Flywheel.setVelocity(56, percent);
      vex::task::sleep(500);
      turnTo(-130, 1000);
      intakeState = true;
      move(33, 700);
      slowMode = true;
      move(30, 1000);
      slowMode = false;
      move(37, 1000);
      Flywheel.setVelocity(59, percent);
      vex::task::sleep(500);
      intakeState = false;
      Flywheel.setVelocity(62, percent);

      turnTo(-45, 1000);
      move(-15, 1000);
      vex::task::sleep(200);
      indexerState = true;
      vex::task::sleep(150);
      indexerState = false;
      Flywheel.setVelocity(65, percent);
      vex::task::sleep(700);
      indexerState = true;
      vex::task::sleep(150);
      indexerState = false;

      Flywheel.setVelocity(68, percent);
      vex::task::sleep(700);
      indexerState = true;
      vex::task::sleep(200);
      indexerState = false;

      die = true;
} 

//Auto in front with voltage
if(false){
      Flywheel.spin(forward, 10.5, volt);
      move(2, 500);
      intakeState = true;
      vex::task::sleep(280);
      intakeState = false;
      move(-4, 600);

      turnTo(-11, 1500);
      vex::task::sleep(1000);
      indexerState = true;
      vex::task::sleep(150);
      indexerState = false;
      Flywheel.spin(forward, 11, volt);
      vex::task::sleep(600);
      indexerState = true;
      vex::task::sleep(250);
      indexerState = false;

      Flywheel.spin(forward, 5, volt);
      vex::task::sleep(500);
      turnTo(-130, 1000);
      intakeState = true;
      move(34, 1000);
      slowMode = true;
      move(25, 700);
      slowMode = false;
      turnTo(-135, 1000);
      move(37, 1000);
      Flywheel.spin(forward, 10, volt);
      vex::task::sleep(1000);
      Flywheel.spin(forward, 10.5, volt);

      turnTo(-45, 1000);
      move(-17, 700);
      intakeState = false;
      vex::task::sleep(200);
      indexerState = true;
      vex::task::sleep(150);
      indexerState = false;
      Flywheel.spin(forward, 9, volt);
      vex::task::sleep(700);
      indexerState = true;
      vex::task::sleep(150);
      indexerState = false;

      Flywheel.spin(forward, 10, volt);
      vex::task::sleep(700);
      indexerState = true;
      vex::task::sleep(200);
      indexerState = false;

      die = true;

}

//Auto not in front
if(false){

  Flywheel.setVelocity(68, percent);
  flywheelState = true;

  move(32, 1000);
  turnTo(90, 1000);
  move(8, 1000);
  intakeState = true;
  vex::task::sleep(250);
  intakeState = false;

  move(-5, 1000);

  turnTo(100, 1000);
  indexerState = true;
  vex::task::sleep(150);
  indexerState = false;
  Flywheel.setVelocity(69, percent);
  vex::task::sleep(700);
  indexerState = true;
  vex::task::sleep(250);
  indexerState = false;
  Flywheel.setVelocity(50, percent);
  turnTo(223, 1000);
  intakeState = true;
  move(60, 1000);
  vex::task::sleep(500);
  move(50, 1000);
  Flywheel.setVelocity(65, percent);
  vex::task::sleep(1000);

  turnTo(135, 1000);
    intakeState = false;
  vex::task::sleep(200);
      indexerState = true;
      vex::task::sleep(250);
      indexerState = false;
      Flywheel.setVelocity(66, percent);
      vex::task::sleep(700);
    indexerState = true;
      vex::task::sleep(250);
      indexerState = false;
      Flywheel.setVelocity(67, percent);
       vex::task::sleep(700);
    indexerState = true;
      vex::task::sleep(200);
      indexerState = false;
      die=true;
}

//Auto not in front with voltage
if(true){

  Flywheel.setStopping(coast);
  Flywheel.spin(forward, 11.3, volt);
  move(35, 1000);
  turnTo(90, 1000);
  move(7, 700);
  intakeState = true;
  vex::task::sleep(350);
  intakeState = false;

  move(-8, 500);
  turnTo(99, 1000);
  indexerState = true;
  vex::task::sleep(150);
  indexerState = false;
  Flywheel.spin(forward, 11.5, volt);
  vex::task::sleep(700);
  indexerState = true;
  vex::task::sleep(250);
  indexerState = false;
  Flywheel.spin(forward, 0, volt);

  turnTo(220, 1000);
  intakeState = true;
  move(62, 1000);
  turnTo(225, 700);
  vex::task::sleep(500);
  move(60, 500);
  Flywheel.spin(forward, 9, volt);
  vex::task::sleep(1000);

  turnTo(135, 1000);
  move(-16, 1000);
  intakeState = false;
  Flywheel.spin(forward, 10.5, volt);
  vex::task::sleep(200);
  indexerState = true;
  vex::task::sleep(250);
  indexerState = false;
  Flywheel.spin(forward, 9.5, volt);
  vex::task::sleep(700);
  indexerState = true;
  vex::task::sleep(250);
  indexerState = false;
  Flywheel.spin(forward, 10, volt);
  vex::task::sleep(700);
  indexerState = true;
  vex::task::sleep(200);
  indexerState = false;
  die=true;
}

//Skills
if(false){

  move(2, 500);
  intakeState = true;
  vex::task::sleep(190);
  intakeState = false;
  move(-4, 500);
  turnTo(135, 1000);
  intakeState = true;
  move(38, 1000);
  intakeState = false;
  turnTo(90, 1000);
  move(9.5, 500);

  intakeState = true;
  vex::task::sleep(230);
  intakeState = false;
  Flywheel.setVelocity(53, percent);
  flywheelState=true;

  move(-9,700);
  turnTo(0, 1000);
  move(-75, 1000);
  turnTo(5, 1000);
  vex::task::sleep(1500);
  shoot3();
  vex::task::sleep(200);
  flywheelState=false;
  turnTo(-5, 1000);
  move(69, 1500);

  turnTo(-140, 1500);
  intakeState = true;
  move(70, 700);
  turnTo(-140, 700);
  vex::task::sleep(1000);
  move(53, 1000);
  vex::task::sleep(1000);
  intakeState = false;
  Flywheel.setVelocity(55, percent);
  flywheelState=true;
  move(-33, 1000);
  turnTo(-45, 1000);
  move(-17, 750);
  vex::task::sleep(1000);
  shoot3();
  flywheelState = false;

  turnTo(-45, 1000);
  move(6, 1000);
  turnTo(-140, 1000);
  intakeState=true;
  move(50, 700);
  slowMode=true;
  move(50, 700);
  slowMode=false;
  move(10, 1000);
  vex::task::sleep(1000);
  intakeState=false;
  move(-12, 1000);

  turnTo(-88, 1000);
  Flywheel.setVelocity(53.5, percent);
  flywheelState=true;
  move(-78, 1000);
  turnTo(-98, 1000);
  vex::task::sleep(1000);
  shoot3();
  flywheelState = false;

  turnTo(-90, 1000);
  move(74, 1000);
  turnTo(-180, 1000);
  move(4, 500);
  intakeState = true;
  vex::task::sleep(300);
  intakeState = false;
  vex::task::sleep(1000);
  move(-35, 1000);

  
  turnTo(-315, 1500);
  move(-20, 1500);
  vex::task::sleep(300);
  expansion1.set(true);
  expansion2.set(true);
}


}
void drivercontrolInit(){
  PID_enable = false;
  WheelLF.setStopping(brake);
  WheelLM.setStopping(brake);
  WheelLB.setStopping(brake);
  WheelRF.setStopping(brake);
  WheelRM.setStopping(brake);
  WheelRB.setStopping(brake);

  Intake.setMaxTorque(100, percent);
  Intake.setVelocity(100, percent);

  Flywheel.setMaxTorque(100, percent);
  Flywheel.setVelocity(60, percent);
  Flywheel.setStopping(coast);

  expansion1.set(false);
  expansion2.set(false);
  AngleAdjuster.set(false);

}
void Drivercontrol(){
  drivercontrolInit();
  Brain.Timer.reset();
  int wheelSpeed = 100;
  task endgame = task(endgameRumble);
  
  //task testing = task(test);
  while(1){
    int axis3 = Controller1.Axis3.position() * wheelSpeed / 100;
    int axis1 = Controller1.Axis1.position() * wheelSpeed / 100;
    int axis2 = Controller1.Axis2.position() * wheelSpeed / 100;
    int axis4 = Controller1.Axis4.position() * wheelSpeed / 100;
    Brain.Screen.clearScreen();
    Brain.Screen.print(Flywheel.velocity(percent));
    if(arcadeDrive){
      WheelLF.setVelocity(axis3+axis1, percent);
      WheelLM.setVelocity(axis3+axis1, percent);
      WheelLB.setVelocity(axis3+axis1, percent);
      WheelRF.setVelocity(axis3-axis1, percent);
      WheelRM.setVelocity(axis3-axis1, percent);
      WheelRB.setVelocity(axis3-axis1, percent);

    } else {
      WheelLF.setVelocity(axis3, percent);
      WheelLM.setVelocity(axis3, percent);
      WheelLB.setVelocity(axis3, percent);
      WheelRB.setVelocity(axis2, percent);
      WheelRM.setVelocity(axis2, percent);
      WheelRF.setVelocity(axis2, percent);
    }

    if(Controller1.ButtonR1.pressing()){
      Intake.spin(forward);
    } else if(Controller1.ButtonUp.pressing()){
      Intake.spin(reverse);
    } else{
      Intake.stop(hold);
    }
    if(flywheelOn){
      Flywheel.spin(forward);
    } else{
      Flywheel.stop(coast);
    }


    Controller1.ButtonL1.released(flywheelSwitch);
    Controller1.ButtonX.released(switchDrive);

  
 if(Controller1.ButtonB.pressing() && Brain.Timer.value() >= 95) {
    expansion1.set(true);
    expansion2.set(true);
  } else if(Controller1.ButtonA.pressing() && Brain.Timer.value() >= 95){
    expansion1.set(false);
    expansion2.set(false);
  }
  if(Controller1.ButtonRight.pressing()){
    AngleAdjuster.set(false);
    Flywheel.setVelocity(55, percent);
  } else if(Controller1.ButtonLeft.pressing()){
    AngleAdjuster.set(true);
    Flywheel.setVelocity(53, percent);
  }

 

    WheelLF.spin(forward);
    WheelLM.spin(forward);
    WheelLB.spin(forward);
    WheelRF.spin(forward);
    WheelRM.spin(forward);
    WheelRB.spin(forward);
  
    task::sleep(20);
  }
}

int main() {
  vexcodeInit();
  Competition.autonomous(Autonomous);
  Competition.drivercontrol(Drivercontrol);
}