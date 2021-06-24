#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  control->setPIDConstants( 6000, 10, 24000, 4000, 
                            6000, 10, 24000, 4000,
                            300, 10, 600, 2000);
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
  control->deployRobot();
  control->updateTargetPos(0, 16, 180);
  control->waitUntilSettled();
  pros::Task::delay(1250);
}

void autonomousRoutine::redSideAuto(){
  control->deployRobot();
  control->updateTargetPos(0, 21, 0);
  control->waitUntilDistance(1);
  control->updateTargetPos(0, 21, 135);
  control->waitUntilDeg(1);
  control->updateIntakePct(127);
  control->updateTargetPos(17, 6.5, 135);
  control->waitUntilSettled();
  control->updateRollers(127);
  control->waitUntilBalls(2);
  control->updateIntakePct(0);
  pros::Task::delay(2000);
  control->updateRollers(0);
  control->updateTargetPos(0, 21, 135);
  control->waitUntilSettled();
}

void autonomousRoutine::blueSideAuto(){
  
}
