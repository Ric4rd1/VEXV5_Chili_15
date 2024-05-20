#include "vex.h"
#include "vex_global.h"
#include "vex_motorgroup.h"
#include "constants.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Controller
controller Controller1 = controller(primary);

// Inertial sensor
inertial InertialSensor(PORT10);

// Chasis motors
motor leftMotorFront(PORT12, ratio18_1, true);
motor leftMotorBack(PORT11, ratio18_1, true);
motor rightMotorFront(PORT20, ratio18_1, false);
motor rightMotorBack(PORT19, ratio18_1, false);

motor_group leftDrive(leftMotorFront, leftMotorBack);
motor_group rightDrive(rightMotorFront, rightMotorBack);
smartdrive Drivetrain (leftDrive, rightDrive, InertialSensor, TRACK_WIDTH, TRACK_BASE, WHEEL_TRAVEL, distanceUnits::mm, EXT_GEAR_RATIO);

bool RemoteControlCodeEnabled = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}

void chassis_control(){
  int drivetrainLeftSideSpeed = (Controller1.Axis3.position() + (Controller1.Axis1.position()));
  int drivetrainRightSideSpeed = (Controller1.Axis3.position() - (Controller1.Axis1.position()));
  
  if (drivetrainLeftSideSpeed < JOYSTICK_DEADBAND && drivetrainLeftSideSpeed > -JOYSTICK_DEADBAND) {
    if (DrivetrainLNeedsToBeStopped_Controller1) {
      leftDrive.stop();
      DrivetrainLNeedsToBeStopped_Controller1 = false;
    }
  } else {
    DrivetrainLNeedsToBeStopped_Controller1 = true;
  }
  if (drivetrainRightSideSpeed < JOYSTICK_DEADBAND && drivetrainRightSideSpeed > -JOYSTICK_DEADBAND) {
    if (DrivetrainRNeedsToBeStopped_Controller1) {
      rightDrive.stop();
      DrivetrainRNeedsToBeStopped_Controller1 = false;
    }
  } else {
    DrivetrainRNeedsToBeStopped_Controller1 = true;
  }
  
  if (DrivetrainLNeedsToBeStopped_Controller1) {
    leftDrive.setVelocity(drivetrainLeftSideSpeed, percent);
    leftDrive.spin(forward);
  }
  if (DrivetrainRNeedsToBeStopped_Controller1) {
    rightDrive.setVelocity(drivetrainRightSideSpeed, percent);
    rightDrive.spin(forward);
  }
}