#include "usercontrol.h"
#define TURN_SENSITIVITY 1.0
#define MOVE_SENSITIVITY 1.0
userControl::userControl(robotChasis *robot){
  simp = robot;
  simp->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}

void userControl::storageRoller(){
}

void userControl::setBrakeMode(){
  if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_B)) simp->set_drive_break_type(pros::E_MOTOR_BRAKE_HOLD);
  else if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) simp->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}

void userControl::intakeM(){
  if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
    simp->leftIntake.move(127);
    simp->rightIntake.move(127);
  } else if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    simp->leftIntake.move(-127);
    simp->rightIntake.move(-127);
  } else {
    simp->leftIntake.move(0);
    simp->rightIntake.move(0);
  }
}

void userControl::liftControl(){
  if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
    simp->roller1.move(127);
    simp->roller2.move(127);
  } else if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    simp->roller1.move(-127);
    simp->roller2.move(-127);
  } else {
    simp->roller1.move(0);
    simp->roller2.move(0);
  }
}

void userControl::driveM(){
  a3 = simp->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  a4 = simp->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  a1 = simp->mController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  if((std::abs(a3) > 10) || (std::abs(a4) > 10) || (std::abs(a1) > 10)){
    simp->frontRight.move((a3 * MOVE_SENSITIVITY) - (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
    simp->frontLeft.move(-(a3 * MOVE_SENSITIVITY) - (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
    simp->backRight.move((a3 * MOVE_SENSITIVITY) + (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
    simp->backLeft.move(-(a3 * MOVE_SENSITIVITY) + (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
  } else {
    simp->frontRight.move(0);
    simp->frontLeft.move(0);
    simp->backRight.move(0);
    simp->backLeft.move(0);
  }
}

void userControl::driveLoop(){
  while(true){
    intakeM();
    setBrakeMode();
    liftControl();
    driveM();

    pros::Task::delay(20);
  }
}