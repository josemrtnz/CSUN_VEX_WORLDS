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
  control->updateTargetPos(10, 10, 0);
  control->waitUntilSettled();
  control->updateTargetPos(0, 0, 0);
  control->waitUntilSettled();
  pros::Task::delay(1250);
}

void autonomousRoutine::redSideAuto(){

}

void autonomousRoutine::blueSideAuto(){
  
}
