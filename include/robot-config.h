using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;

// Inertial sensor
extern inertial InertialSensor;

// Chasis motors
extern motor leftMotorFront;
extern motor leftMotorBack;
extern motor rightMotorFront;
extern motor rightMotorBack;

// Motor groups
extern motor_group leftDrive;
extern motor_group rightDrive;

// Drivetrain
extern smartdrive Drivetrain;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

// Drive controll functions
void chassis_control();

// Autonomous functions
void auton();
