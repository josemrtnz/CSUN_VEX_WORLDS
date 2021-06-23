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

void autonomousControl::intakeMove(){
  simp->leftIntake.move(intakePct);
  simp->rightIntake.move(intakePct);
}


void autonomousControl::waitUntilSettled(){
  pros::Task::delay(100);
  while(averageRPM() != 0){
    pros::Task::delay(20);
  }
}

void autonomousControl::waitTilFull(){
}

void autonomousControl::waitUntilDistance(float dis){
  pros::Task::delay(100);
  while(dis < vMag){
    pros::Task::delay(20);
  }
}

void autonomousControl::waitUntilBalls(int ball_i, int ball_o){
  balls_intaken = 0;
  balls_outtaken = 0;
  bool iballs = false;
  bool oballs = false;

  while((balls_intaken < ball_i) || (balls_outtaken < ball_o)){
    iballs = simp->limit1.get_value();
    oballs = simp->limit2.get_value();
    if(iballs && !iBalls_prev) balls_intaken++;
    if(oballs && !oBalls_prev) balls_outtaken++;
    iBalls_prev = iballs;
    oBalls_prev = oballs;
    pros::Task::delay(20);
  }
  pros::Task::delay(500);
}

void autonomousControl::openIntake(){
  simp->intakeP.set_value(1);
}

void autonomousControl::closeIntake(){
  simp->intakeP.set_value(0);
}

void autonomousControl::cycle_red(int red_balls, int ball_i, int ball_o, int timeout){
  balls_intaken = 0;
  balls_outtaken = 0;
  red_balls_outtaken = 0;
  fprevBall = false;
  bool iballs = false;
  bool oballs = false;
  bool red_shoot = false;
  std::uint32_t time = pros::millis();

  updateAllRollers(100);
  updateIntakePct(127);

  while(((balls_intaken < ball_i) || (balls_outtaken < ball_o || (red_balls_outtaken < red_balls))) && ((time + timeout) > pros::millis())){
    oballs = simp->limit2.get_value();
    if(simp->line3.get_value() < 2850) fcurrBall = true;
    else fcurrBall = false;
    if(simp->line2.get_value() < 2850) iballs = true;
    else iballs = false;
    if(iballs && !iBalls_prev) balls_intaken++;
    if(oballs && !oBalls_prev) balls_outtaken++;
    iBalls_prev = iballs;
    oBalls_prev = oballs;
    red_shoot = false;

    rgb_value1 = simp->colorSensor1.get_rgb();
    rgb_value2 = simp->colorSensor2.get_rgb();

    if((rgb_value1.red > rgb_value1.blue) && (simp->colorSensor1.get_proximity() > 120)) red_inside = true;
    if(((rgb_value2.red > rgb_value2.blue) && (simp->colorSensor2.get_proximity() > 120)) && (red_balls_outtaken < red_balls)){
      updateRoller1(60);
      updateRoller2(-127);
      updateRoller3(100);
      red_inside = false;
      red_shoot = true;
      if(fcurrBall && !fprevBall) {
        red_balls_outtaken++;
      }
  } 
  if(red_inside){
    updateRoller1(50);
    updateRoller2(50);
    updateRoller3(50);
  } else if(!red_shoot){
    updateRoller1(90);
    updateRoller2(90);
    updateRoller3(90);
  }

  if(balls_intaken >= ball_i) updateIntakePct(25);
  if(balls_outtaken >= ball_o) updateRoller4(-20);
  fprevBall = fcurrBall;
  pros::Task::delay(20);
  }

  pros::Task::delay(500);
  updateAllRollers(0);
  updateIntakePct(0);
}

void autonomousControl::cycle_blue(int blue_balls, int ball_i, int ball_o, int timeout){
  balls_intaken = 0;
  balls_outtaken = 0;
  blue_balls_outtaken = 0;
  fprevBall = false;
  bool iballs = false;
  bool oballs = false;
  bool blue_shoot = false;
  std::uint32_t time = pros::millis();

  updateAllRollers(100);
  updateIntakePct(127);

  while(((balls_intaken < ball_i) || (balls_outtaken < ball_o || (blue_balls_outtaken < blue_balls))) && ((time + timeout) > pros::millis())){
    oballs = simp->limit2.get_value();
    if(simp->line3.get_value() < 2850) fcurrBall = true;
    else fcurrBall = false;
    if(simp->line2.get_value() < 2850) iballs = true;
    else iballs = false;
    if(iballs && !iBalls_prev) balls_intaken++;
    if(oballs && !oBalls_prev) balls_outtaken++;
    iBalls_prev = iballs;
    oBalls_prev = oballs;
    blue_shoot = false;

    rgb_value1 = simp->colorSensor1.get_rgb();
    rgb_value2 = simp->colorSensor2.get_rgb();

    if((rgb_value1.blue > rgb_value1.red) && (simp->colorSensor1.get_proximity() > 120)) blue_inside = true;
    if(((rgb_value2.blue > rgb_value2.red) && (simp->colorSensor2.get_proximity() > 120)) && (blue_balls_outtaken < blue_balls)){
      updateRoller1(60);
      updateRoller2(-127);
      updateRoller3(100);
      blue_inside = false;
      blue_shoot = true;
      if(fcurrBall && !fprevBall) {
        blue_balls_outtaken++;
      }
  } 
  if(blue_inside){
    updateRoller1(50);
    updateRoller2(50);
    updateRoller3(50);
  } else if(!blue_shoot){
    updateRoller1(90);
    updateRoller2(90);
    updateRoller3(90);
  }

  if(balls_intaken >= ball_i) updateIntakePct(25);
  if(balls_outtaken >= ball_o) updateRoller4(-20);
  fprevBall = fcurrBall;
  pros::Task::delay(20);
  }

  pros::Task::delay(500);
  updateAllRollers(0);
  updateIntakePct(0);
}

void autonomousControl::waitUntilDeg(float deg){
  pros::Task::delay(100);
  while(deg < fabs(turnPID.curr - turnPID.target) ){
    pros::Task::delay(20);
  }
}


void autonomousControl::updateRoller1(int pwr){ roller1Pct = pwr; }
void autonomousControl::updateRoller2(int pwr){ roller2Pct = pwr; }
void autonomousControl::updateRoller3(int pwr){ roller3Pct = pwr; }
void autonomousControl::updateRoller4(int pwr){ roller4Pct = pwr; }


void autonomousControl::updateAllRollers(int pow){
  updateRoller1(pow);
  updateRoller2(pow);
  updateRoller3(pow);
  updateRoller4(pow);
}

void autonomousControl::odometryMove(bool oMove){ movAB_Enabled = oMove; }

void autonomousControl::driveM(double a3, double a4, double a1){
  simp->frontRight.move_voltage(a3 - a4 - a1);
  simp->frontLeft.move_voltage(-a3 - a4 - a1);
  simp->backRight.move_voltage(a3 + a4 - a1);
  simp->backLeft.move_voltage(-a3 + a4 - a1);
}

void autonomousControl::rollerMove(){
  simp->roller1.move(roller1Pct);
  simp->roller2.move(roller2Pct);
  simp->roller3.move(roller3Pct);
  simp->roller4.move(roller4Pct);
}

void autonomousControl::autoMain(){
  simp->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);

  while(true){
    movAB();
    intakeMove();
    rollerMove();
    pros::Task::delay(20);
  }
}

void autonomousControl::updateCurrPos(){
  xPID.curr = tracking->getXPos();
  yPID.curr = tracking->getYPos();
  turnPID.curr = tracking->getangleD();
}
