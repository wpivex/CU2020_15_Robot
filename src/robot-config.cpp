#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FrontRight = motor(PORT13, ratio18_1, false);
motor BackRight = motor(PORT9, ratio18_1, false);
motor BackLeft = motor(PORT20, ratio18_1, false);
motor FrontLeft = motor(PORT2, ratio18_1, false);

motor MainConveyor = motor(PORT3, ratio6_1, true);
motor Indexer = motor(PORT4, ratio6_1, true);
motor Expeller = motor(PORT10, ratio6_1, false);

motor LeftIntake = motor(PORT1, ratio18_1, false);
motor RightIntake = motor(PORT19, ratio18_1, true);

controller Controller1 = controller(primary);

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
int DEADBAND = 5; //Controller goes from -128 to 128
// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int RightY = Controller1.Axis2.position();
      int RightX = Controller1.Axis1.position()*0.6;
      int LeftY = Controller1.Axis4.position();

      bool leftBumper = Controller1.ButtonL1.pressing();
      bool leftTrigger = Controller1.ButtonL2.pressing();
      bool rightBumper = Controller1.ButtonR1.pressing();
      bool rightTrigger = Controller1.ButtonR2.pressing();
      

      //Deadband
      if(abs(RightX) < DEADBAND){ RightX = 0; }
      if(abs(RightY) < DEADBAND){ RightY = 0; }
      if(abs(LeftY) < DEADBAND){ LeftY = 0; }

      FrontLeft.spin(forward, RightY+RightX+LeftY, percentUnits::pct);
      BackLeft.spin(forward, RightY+RightX-LeftY, percentUnits::pct);
      FrontRight.spin(forward, -RightY+RightX+LeftY, percentUnits::pct);
      BackRight.spin(forward, -RightY+RightX-LeftY, percentUnits::pct);


      if(leftBumper){
        LeftIntake.spin(forward, 100, percentUnits::pct);
        RightIntake.spin(forward, 100, percentUnits::pct);
        MainConveyor.spin(forward, 100, percentUnits::pct);
        Expeller.spin(forward, 100, percentUnits::pct);
        Indexer.stop(brakeType::coast);
      }else if (leftTrigger) {
        LeftIntake.spin(reverse, 35, percentUnits::pct);
        RightIntake.spin(reverse, 35, percentUnits::pct);
        MainConveyor.spin(reverse, 100, percentUnits::pct);
        Expeller.stop(brakeType::coast);
        Indexer.stop(brakeType::coast);
      }else if(rightBumper){
        LeftIntake.spin(forward, 100, percentUnits::pct);
        RightIntake.spin(forward, 100, percentUnits::pct);
        MainConveyor.spin(forward, 100, percentUnits::pct);
        Expeller.spin(forward, 100, percentUnits::pct);
        Indexer.spin(forward, 100, percentUnits::pct);
      }else if (rightTrigger) {
        LeftIntake.spin(forward, 100, percentUnits::pct);
        RightIntake.spin(forward, 100, percentUnits::pct);
        MainConveyor.spin(forward, 100, percentUnits::pct);
        Expeller.spin(reverse, 100, percentUnits::pct);
        Indexer.stop(brakeType::coast);
      }else{
        LeftIntake.stop(brakeType::coast);
        RightIntake.stop(brakeType::coast);
        MainConveyor.stop(brakeType::coast);
        Expeller.stop(brakeType::coast);
        Indexer.stop(brakeType::coast);
      }
    
      // wait before repeating the process
      wait(20, msec);
    }
  } //End While True
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}