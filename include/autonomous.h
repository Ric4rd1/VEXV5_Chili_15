#include "constants.h"
#include "robot-config.h"
#include "vex.h"

using namespace vex;
extern brain Brain;

void auton(){
    // Drivetrain
    InertialSensor.calibrate();
    Drivetrain.setDriveVelocity(25, percent);
    Drivetrain.setTurnVelocity(10, percent);

    Drivetrain.driveFor(forward, 35, distanceUnits::cm);
    Drivetrain.turnFor(right, 45, rotationUnits::deg);
    Drivetrain.driveFor(forward, 75, distanceUnits::cm);



    /*
    // Drivetrain
    leftDrive.setVelocity(50, percent);
    rightDrive.setVelocity(50, percent);
    leftDrive.spinFor(forward, 5, seconds);
    rightDrive.spinFor(forward, 5, seconds);
    leftDrive.stop();
    rightDrive.stop();
    */
}