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
  control->updateTargetPos(0, 0, 180);
  control->waitUntilSettled();
  pros::Task::delay(1250);
}

void autonomousRoutine::redSideAuto(){

}

void autonomousRoutine::blueSideAuto(){
  
}
