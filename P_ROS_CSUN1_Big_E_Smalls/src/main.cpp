#include "main.h"
#include "robot-config.h"
#include "chasisControl.h"
#include "autonomousRoutine.h"
#include "usercontrol.h"

#define TEST 0
#define SKILLS 1

robotChasis simp = robotChasis(2.75, 5.5, 5.5, 5.2);
odometry tracker = odometry(&simp, 0, 0, 0);
autonomousControl autoChasis = autonomousControl(&simp, &tracker);
autonomousRoutine autoRoutine = autonomousRoutine(&autoChasis);

int trackerWrapper(){
  tracker.updatePosition();
  return 0;
}

int printerWrapper(){
  tracker.updateScreen();
  return 0;
}

int autoWrapper(){
  autoChasis.autoMain();
  return 0;
}

pros::Task startTracking(trackerWrapper);
pros::Task startPrinting(printerWrapper);
pros::Task startAuto(autoWrapper);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch.
 */
void competition_initialize() {}

void autonomous() {
	autoRoutine.run(TEST);
}

void opcontrol() {
	startAuto.remove();
	userControl driveJose = userControl(&simp, true);
    driveJose.driveLoop();
}
