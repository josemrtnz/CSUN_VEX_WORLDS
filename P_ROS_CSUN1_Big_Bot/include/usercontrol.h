#pragma once
#include "robot-config.h"
#include <math.h>
#include <cstdlib>

/// Class for user control.
///
/// This class will control the motors on the chasis using input from a controller.
/// Controller data is accessed via a pointer from a \ref robotChasis object.
class userControl{
  public:

    /// Runs the main drive loop.
    ///
    /// Will run a while loop that will check for user input and move the motors accordingly.
    /// @return NULL
    void driveLoop();

    /// Constructor for the class \ref userControl.
    ///
    /// This constructor will create an instance of the class \ref userControl.
    /// @param *robot Pointer to an instance of \ref robotChasis object.
    userControl(robotChasis *robot);

  private:
    robotChasis *simp;
    int a3, a4, a1;
    pros::c::optical_rgb_s_t rgb_value2, rgb_value1;

    void driveM();
    void intakeM();
    void storageRoller();
    void liftControl();
    void setBrakeMode();
    bool blue_inside = false;
};

