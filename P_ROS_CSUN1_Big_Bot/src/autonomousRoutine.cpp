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
  control->cycle_blue(2, 0, 1, 2500);
  pros::Task::delay(1250);
}

void autonomousRoutine::odometryOnlyAuto(){
  control->closeIntake();
  control->updateTargetPos(0, 10, 90);
  control->waitUntilDistance(1);
  control->updateTargetPos(13, 10, 180);
  control->waitUntilDeg(1);
  control->updateTargetPos(13, 2.5, 180);
  control->waitUntilDistance(2);
  control->cycle_blue(1, 1, 1, 5000);
  control->updateTargetPos(13, 10, 180);
  control->waitUntilDistance(1);
  control->updateTargetPos(35, 10, 225);
  control->waitUntilSettled();
  pros::Task::delay(1000);
}

void autonomousRoutine::odometryVisionAuto(){
  
}