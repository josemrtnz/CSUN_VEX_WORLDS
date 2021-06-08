#include "chasisControl.h"
#define OUTTAKE_LINE 2000
#define INTAKE_LINE 2000

autonomousControl::autonomousControl(robotChasis *robot, odometry *tr){
  simp = robot;
  tracking = tr;
}

void autonomousControl::setPIDConstants(float xkP, float xkI, float xkD, int xCap,
                                        float ykP, float ykI, float ykD, int yCap,
                                        float turnkP, float turnkI, float turnkD, int turnCap){
  xPID.kP = xkP; xPID.kI = xkI; xPID.kD = xkD; xPID.cap = xCap;
  yPID.kP = ykP; yPID.kI = ykI; yPID.kD = ykD; yPID.cap = yCap;
  turnPID.kP = turnkP; turnPID.kI = turnkI; turnPID.kD = turnkD; turnPID.cap = turnCap;                         
}

void autonomousControl::moveDrive(float x, float y, float turn){
  simp->frontLeft.move_voltage(-(x*cos(simp->get_flbr()-tracking->getangleR()) + y*sin(simp->get_flbr()-tracking->getangleR())) - turn);
  simp->frontRight.move_voltage((x*cos(simp->get_frbl()-tracking->getangleR()) + y*sin(simp->get_frbl()-tracking->getangleR())) - turn);
  simp->backLeft.move_voltage(-(x*cos(simp->get_frbl()-tracking->getangleR()) + y*sin(simp->get_frbl()-tracking->getangleR())) - turn);
  simp->backRight.move_voltage((x*cos(simp->get_flbr()-tracking->getangleR()) + y*sin(simp->get_flbr()-tracking->getangleR())) - turn);
}

float autonomousControl::averageRPM(){
  return (fabs(simp->frontRight.get_actual_velocity()) + fabs(simp->frontLeft.get_actual_velocity()) + fabs(simp->backRight.get_actual_velocity()) + fabs(simp->backLeft.get_actual_velocity()))/4;
}

float autonomousControl::updatePID(PIDSettings *good){
  good->error = good->curr - good->target;
  good->derivative = good->error - good->prevError;
  good->totalError = good->totalError + good->error;

  if((good->totalError*good->kI)>good->cap) good->totalError = good->cap/good->kI;
  else if((good->totalError*good->kI)<-good->cap)good->totalError = -good->cap/good->kI;

  if(std::signbit(good->error) != std::signbit(good->prevError)) good->totalError = 0;

  good->prevError = good->error;
  return -(good->kP*good->error + good->kD*good->derivative + good->kI*good->totalError);
}

int autonomousControl::turnCap(float distanceMag){
  if(distanceMag>30.0) return 2000;
  else if(distanceMag>10.0) return 6000;
  else return 10000;
}

void autonomousControl::movAB(){
  updateCurrPos();
  vectorD[0] = xPID.target - xPID.curr;
  vectorD[1] = yPID.target - yPID.curr;
  vMag = sqrt((vectorD[0]*vectorD[0]) + (vectorD[1]*vectorD[1])); 

  int turningCap = turnCap(vMag);

  float xVoltage = updatePID(&xPID);
  float yVoltage = updatePID(&yPID);
  float angleVoltage = updatePID(&turnPID);

  if(angleVoltage>turningCap) angleVoltage = turningCap;
  else if(angleVoltage<-turningCap) angleVoltage = -turningCap;

  if(xVoltage>10000) xVoltage = 10000;
  else if(xVoltage<-10000) xVoltage = -10000;

  if(yVoltage>10000) yVoltage = 10000;
  else if(yVoltage<-10000) yVoltage = -10000;

  moveDrive(xVoltage, yVoltage, angleVoltage);  
}

void autonomousControl::updateTargetPos(float x, float y, int angleO){
  xPID.target = x;
  yPID.target = y;
  turnPID.target = angleO;
}

void autonomousControl::updateIntakePct(int pow){ intakePct = pow; }

void autonomousControl::updateFlyTBH(){

  flyError = flyWheelRPM - simp->flyOuttake.get_actual_velocity();
  flyVoltage += flykI*flyError;

  // Clip
  if (flyVoltage > 12000) flyVoltage = 12000;
  else if (flyVoltage < lowerBound) flyVoltage = lowerBound;

  // Zero crossing
  if (std::signbit(flyError) != std::signbit(flyLastError)){
    if( firstCross ){
      flyVoltage = flyApprox;
      firstCross = false;
    } else flyVoltage = 0.5 * (flyVoltage + flyZero);
    flyZero = flyVoltage;
  }
  flyLastError = flyError;
}

void autonomousControl::intakeMove(){
  simp->leftIntake.move(intakePct);
  simp->rightIntake.move(intakePct);
}

void autonomousControl::flyMove(){
  updateFlyTBH();
  simp->flyOuttake.move_voltage(flyVoltage);
}

void autonomousControl::rollerMove(){ 
  simp->rollerIntake.move(rollerPct); 
  }

void autonomousControl::waitUntilSettled(){
  pros::Task::delay(100);
  while(averageRPM() != 0){
    pros::Task::delay(20);
  }
}

void autonomousControl::waitTilFull(){
  pros::Task::delay(20);
  int loopTime = pros::millis();

  while(simp->limit.get_value() == false && (2000 > (pros::millis() - loopTime))){
    pros::Task::delay(20);
  }
  shooting = false;
}

void autonomousControl::waitUntilDistance(float dis){
  pros::Task::delay(100);
  while(dis < vMag){
    pros::Task::delay(20);
  }
}

void autonomousControl::waitUntilBalls(int ball){
}

void autonomousControl::waitUntilDeg(float deg){
  pros::Task::delay(100);
  while(deg < fabs(turnPID.curr - turnPID.target) ){
    pros::Task::delay(20);
  }
}

void autonomousControl::updateFly(int rpm){
  flyApprox = 10000;
  flyError = 0;
  flyLastError = 0;
  flyVoltage = 0;
  flyZero = 8000;
  lowerBound = 5000;
  firstCross = true;
  flyWheelRPM = rpm;
}

void autonomousControl::stopFly(){
  flyApprox = 0;
  flyError = 0;
  flyLastError = 0;
  flyVoltage = 0;
  flyZero = 0;
  lowerBound = 0;
  firstCross = true;
  flyWheelRPM = 0;
}

void autonomousControl::updateRoller(int pwr){ rollerPct = pwr; }

void autonomousControl::shootBall(int balls){
  ballsDeteced = 0;
  ballsToShoot = balls;
  shooting = true;
}

void autonomousControl::shootingBall(){
  if (shooting == true){
    rollerPct = 40;

    if ((prevShot == true) && (simp->limit.get_value() == false)){
      ballsDeteced++;
      if (ballsDeteced == ballsToShoot){
        rollerPct = 0;
        shooting = false;
      }
    }
    prevShot = simp->limit.get_value();
  }
}

void autonomousControl::odometryMove(bool oMove){ movAB_Enabled = oMove; }

void autonomousControl::moveVision(){
  updateVisionPos();

  switch(visionStatus){
    case 1:
      turnVision();
      break;
    case 2:
      strafeVision();
      break;
    case 3:
      forwardVision();
      break;
    default:
      break;
  }
}

void autonomousControl::turnVision(){
  float angleVoltage = updatePID(&turnPID);

  if(angleVoltage>12000) angleVoltage = 12000;
  else if(angleVoltage<-12000) angleVoltage = -12000;

  driveM(0, 0, angleVoltage);
}

void autonomousControl::strafeVision(){}

void autonomousControl::forwardVision(){}

void autonomousControl::driveM(double a3, double a4, double a1){
  simp->frontRight.move_voltage(a3 - a4 - a1);
  simp->frontLeft.move_voltage(-a3 - a4 - a1);
  simp->backRight.move_voltage(a3 + a4 - a1);
  simp->backLeft.move_voltage(-a3 + a4 - a1);
}

void autonomousControl::updateVisionPos(){
}

void autonomousControl::visionTowerAlign(int angDeg){
}

void autonomousControl::countBalls(){
  bool iBalls = (simp->line1.get_value()>INTAKE_LINE) ? false : true;
  bool oBalls = (simp->line2.get_value()>OUTTAKE_LINE) ? false : true;
  if(iBalls_prev == true && iBalls == false) nBalls++;
  if(oBalls_prev == true && oBalls == false) nBalls--;
  iBalls_prev = iBalls;
  oBalls_prev = oBalls;
}

void autonomousControl::autoMain(){
  simp->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);

  while(true){
    movAB();
    shootingBall();
    intakeMove();
    countBalls();
    rollerMove();
    flyMove();
    pros::Task::delay(20);
  }
}

void autonomousControl::updateCurrPos(){
  xPID.curr = tracking->getXPos();
  yPID.curr = tracking->getYPos();
  turnPID.curr = tracking->getangleD();
}
