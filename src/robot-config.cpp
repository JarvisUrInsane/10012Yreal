#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor WheelLF = motor(PORT13, ratio6_1, true);
motor WheelLB = motor(PORT11, ratio6_1, true);
motor WheelRB = motor(PORT18, ratio6_1, false);
motor WheelRF = motor(PORT20, ratio6_1, false);
inertial IMU = inertial(PORT17);
motor Intake = motor(PORT14, ratio6_1, false);
motor Flywheel = motor(PORT15, ratio6_1, true);
motor WheelRM = motor(PORT19, ratio6_1, false);
motor WheelLM = motor(PORT12, ratio6_1, true);
digital_out expansion1 = digital_out(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}