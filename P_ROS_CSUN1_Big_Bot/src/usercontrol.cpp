#include "usercontrol.h"
#define TURN_SENSITIVITY 1.0
#define MOVE_SENSITIVITY 1.0
userControl::userControl(robotChasis *robot, bool dM){
  simp = robot;
  driverMode = dM;
  simp->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}

void userControl::storageRoller(){
}

void userControl::setBrakeMode(){
  if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_B)) simp->set_drive_break_type(pros::E_MOTOR_BRAKE_HOLD);
  else if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) simp->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}

void userControl::intakeM(){
  
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

void userControl::driveMA(){
  a3 = simp->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  a4 = simp->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  a1 = simp->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

  double currAngle = ((simp->getPI()/180)*simp->gyroM.get_heading());
  
  simp->frontLeft.move((a4*cos(simp->get_flbr()-currAngle) + a3*sin(simp->get_flbr()-currAngle)) - (a1 * TURN_SENSITIVITY));
  simp->frontRight.move(-(a4*cos(simp->get_frbl()-currAngle) + a3*sin(simp->get_frbl()-currAngle)) - (a1 * TURN_SENSITIVITY));
  simp->backLeft.move((a4*cos(simp->get_frbl()-currAngle) + a3*sin(simp->get_frbl()-currAngle)) - (a1 * TURN_SENSITIVITY));
  simp->backRight.move(-(a4*cos(simp->get_flbr()-currAngle) + a3*sin(simp->get_flbr()-currAngle)) - (a1 * TURN_SENSITIVITY));
}

void userControl::setDriveMode(){
  if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) driverMode = false;
  else if(simp->mController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) driverMode = true;
}

void userControl::driveLoop(){
  while(true){
    intakeM();
    setBrakeMode();
    setDriveMode();

    if(driverMode) driveM();
    else driveMA();

    pros::Task::delay(20);
  }
}