#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants( 3000, 1, 9000, 4000, 
                            3000, 1, 9000, 4000,
                            200, 20, 400, 2000);
}

void autonomousRoutine::run(int autoSelection) {
  switch(autoSelection){
    case 0:
      test();
      break;
    case 1:
      redSideAuto();
      break;
    case 2:
      blueSideAuto();
      break;
    default:
      printf("No Auto Selected!");
      break;
  }
}

void autonomousRoutine::test(){
  control->updateTargetPos(0, 0, 360);
  control->waitUntilSettled();
  //control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
  pros::Task::delay(1250);
}

void autonomousRoutine::redSideAuto(){
  control->closeIntake();
  control->updateTargetPos(0, 20, 90);
  control->waitUntilDistance(1);
  control->updateTargetPos(16, 15, 180);
  control->waitUntilDeg(2);
  control->updateTargetPos(16, 5.25, 180);
  control->updateIntakePct(100);
  control->waitUntilDistance(2);
  control->updateTargetPos(16, 5.1, 180);
  control->cycle_blue(0, 2, 2, 5000);
  control->updateIntakePct(100);
  control->updateRoller3(-30);
  control->updateRoller4(-60);
  control->updateRoller2(-20);
  control->updateTargetPos(14, 10, 180);
  control->waitUntilDistance(1);
  control->updateTargetPos(-22, 12, 225);
  control->waitUntilDistance(1);
  control->updateTargetPos(-38, -3.5, 225);
  control->cycle_blue(3, 5, 8, 10000);
  control->updateIntakePct(-127);
  control->updateTargetPos(-32, 2, 225);
  pros::Task::delay(5000);
}

void autonomousRoutine::blueSideAuto(){
  control->closeIntake();
  control->updateTargetPos(0, 20, 90);
  control->waitUntilDistance(1);
  control->updateTargetPos(16, 15, 180);
  control->waitUntilDeg(2);
  control->updateTargetPos(16, 5.25, 180);
  control->updateIntakePct(100);
  control->waitUntilDistance(2);
  control->updateTargetPos(16, 5.1, 180);
  control->cycle_red(0, 2, 2, 5000);
  control->updateIntakePct(100);
  control->updateRoller3(-30);
  control->updateRoller4(-60);
  control->updateRoller2(-20);
  control->updateTargetPos(14, 10, 180);
  control->waitUntilDistance(1);
  control->updateTargetPos(-22, 12, 225);
  control->waitUntilDistance(1);
  control->updateTargetPos(-38, -3.5, 225);
  control->cycle_red(2, 4, 10, 5000);
  control->updateTargetPos(-35, 0, 225);
  pros::Task::delay(5000);
}

void autonomousRoutine::odometryVisionAuto(){
  
}