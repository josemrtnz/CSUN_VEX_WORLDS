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
  control->updateRollers(127);
  pros::Task::delay(10000);
  control->updateRollers(0);
  pros::Task::delay(1250);
}

void autonomousRoutine::odometryOnlyAuto(){

}

void autonomousRoutine::odometryVisionAuto(){
  
}