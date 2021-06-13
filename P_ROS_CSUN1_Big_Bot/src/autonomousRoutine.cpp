#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants( 1400, 0, 200, 4000, 
                            1400, 0, 200, 4000,
                            150, 20, 600, 2000);
}

void autonomousRoutine::run(int autoSelection) {
  switch(autoSelection){
    case 0:
      test();
      break;
    case 1:
      odometryOnlyAuto();
      break;
    case 2:
      odometryVisionAuto();
      break;
    default:
      printf("No Auto Selected!");
      break;
  }
}

void autonomousRoutine::test(){
  control->updateTargetPos(0, 0, 90);
  control->waitUntilDeg(5);
  control->updateTargetPos(0, 0, 0);
  control->waitUntilDeg(5);
  control->updateTargetPos(0, 0, 90);
  control->waitUntilDeg(5);
  control->updateTargetPos(0, 0, 0);
  control->waitUntilDeg(5);
  pros::Task::delay(1250);
}

void autonomousRoutine::odometryOnlyAuto(){
  control->updateTargetPos(0, 10, 90);
  control->waitUntilDistance(1);
  control->updateTargetPos(-12, 10, 180);
  control->waitUntilDeg(1);
  control->updateTargetPos(-13, 1, 180);
  control->waitUntilDistance(1);
  control->updateAllRollers(127);
  control->waitUntilBalls(0,1);
  control->updateAllRollers(0);
  pros::Task::delay(1000);
}

void autonomousRoutine::odometryVisionAuto(){
  
}