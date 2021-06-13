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
  } else {
    simp->leftIntake.move(0);
    simp->rightIntake.move(0);
  }
}

void userControl::liftControl(){
  rgb_value2 = simp->colorSensor2.get_rgb();
  rgb_value1 = simp->colorSensor1.get_rgb();
  if((rgb_value1.blue > rgb_value1.red) && (simp->colorSensor1.get_proximity() > 120)) blue_inside =  true;
  if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
    if((rgb_value2.blue > rgb_value2.red) && (simp->colorSensor2.get_proximity() > 120)){
      simp->roller1.move(60);
      simp->roller2.move(-127);
      simp->roller3.move(60);
      simp->roller4.move(80);
      while(!((simp->line3.get_value() < 2850) || (simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_B)))){
        pros::delay(20);
      }
      blue_inside = false;
    } else {
      if(blue_inside){
        simp->roller1.move(60); //65
        simp->roller2.move(60); //65
        simp->roller3.move(60); //65
        simp->roller4.move(100); //100
      } else {
        simp->roller1.move(100);
        simp->roller2.move(100);
        simp->roller3.move(100); 
        simp->roller4.move(100);
      }
    }
  } else if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    simp->roller1.move(-127);
    simp->roller2.move(-127);
    simp->roller3.move(-127);
    simp->roller4.move(-127);
  } else if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    simp->roller1.move(127);
    simp->roller2.move(-127);
    simp->roller3.move(40);
    simp->roller4.move(0);
  } else {
    simp->roller1.move(0);
    simp->roller2.move(0);
    simp->roller3.move(0);
    simp->roller4.move(0);
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