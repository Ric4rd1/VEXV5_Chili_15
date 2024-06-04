#include "constants.h"
#include "robot-config.h"
#include "vex.h"
#include "PID.h"

using namespace vex;
extern brain Brain;

double getAngle(){
    double angle = InertialSensor.rotation();
    // printf("Angle: %f\n", angle);
    return angle;
}

void calibrateLimu() {
  // Wait 2 seconds and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  printf("Calibrating...\n");
  wait(2000, msec);
  for (uint16_t i = 0; i < 160; i++) {
    if (i > 40 && i <= 120) {
        printf("HOLAP\n");
        leftDrive.spin(forward);
        rightDrive.spin(reverse);
        wait(50, msec);
    } else {
        printf("ADIOS\n");
        leftDrive.spin(reverse);
        rightDrive.spin(forward);
        wait(50, msec);
    }
    // Calibrate sensors
    InertialSensor.calibrate();
  }
  leftDrive.stop();
  rightDrive.stop();
}

void calibrate(){
    InertialSensor.calibrate();
    wait(100, msec);
}

float constrain(float value, float min, float max){
    if(value < min){
        return min;
    }else if(value > max){
        return max;
    }else{
        return value;
    }
}

void rotateTest(float angleRef){
    // Parameters
    float error = 0;
    float KP = 1;
    float angle;
    uint16_t turnPrc = 0;
    uint16_t maxPrc = 20;
    uint16_t minPrc = 3;
    uint16_t basePrc = 5;

    // Limit control
    leftDrive.setVelocity(15, percent);
    rightDrive.setVelocity(15, percent);

    leftDrive.spin(forward);
    rightDrive.spin(reverse);

    wait(100, msec);

    for(int i = 0; i < 10; i++){
        angle = getAngle();
        wait(150, msec);
        error = angleRef - angle;

        printf("Angle: %f\n", angle);
        printf("Error = %f - %f = %f\n", angleRef, angle, error);

    }

    leftDrive.stop();
    rightDrive.stop();
}

void rotateP(float angleRef){
    // Setup inertial sensor
    //InertialSensor.calibrate();
    //InertialSensor.setRotation(0, degrees);
    //InertialSensor.setHeading(0, degrees);
    //calibrateLimu();

    // Parameters
    float error = 0;
    float KP = 1.1;
    float angle;
    uint16_t turnPrc = 0;
    
    /*
    uint16_t maxPrc = 15;
    uint16_t minPrc = 3;
    uint16_t basePrc = 5;
    */
    uint16_t maxPrc = 30;
    uint16_t minPrc = 10;
    uint16_t basePrc = 20;
    
    while(1){
        angle = getAngle();
        wait(25, msec);
        error = angleRef - angle;

        printf("Angle: %f\n", angle);
        printf("Error = %f - %f = %f\n", angleRef, angle, error);

        // Proportional control
        turnPrc = error * KP;
        turnPrc = basePrc + turnPrc;

        turnPrc = constrain(turnPrc, minPrc, maxPrc);

        // Limit control
        leftDrive.setVelocity(turnPrc, percent);
        rightDrive.setVelocity(-turnPrc, percent);

        // move motors to the desired position

        if (error > 10){
            leftDrive.spin(forward);
            rightDrive.spin(forward);
            wait(30, msec);
        }else{
            leftDrive.stop();
            rightDrive.stop();
            wait(30, msec);
            if(error < -1){
            leftDrive.spin(reverse);
            rightDrive.spin(reverse);
            wait(30, msec);
            }
            break;
        }

        // wait(150, msec);
    }
}

void rotatePD(float angleRef){
    // Setup inertial sensor
    //InertialSensor.calibrate();
    //InertialSensor.setRotation(0, degrees);
    //InertialSensor.setHeading(0, degrees);
    //calibrateLimu();

    // Parameters
    float error = 0;
    float lastError = 0;
    float KP = 1.1;
    float KD = 0.7;
    float angle;
    uint16_t turnPrc = 0;
    
    uint16_t maxPrc = 15;
    uint16_t minPrc = 3;
    uint16_t basePrc = 5;
    /*
    uint16_t maxPrc = 50;
    uint16_t minPrc = 10;
    uint16_t basePrc = 20;
    */
    while(1){
        angle = getAngle();
        wait(25, msec);
        error = angleRef - angle;

        printf("Angle: %f\n", angle);
        printf("Error = %f - %f = %f\n", angleRef, angle, error);

        // Proportional control
        turnPrc = error * KP + (error - lastError) * KD;
        turnPrc = basePrc + turnPrc;

        lastError = error;

        turnPrc = constrain(turnPrc, minPrc, maxPrc);

        // Limit control
        leftDrive.setVelocity(turnPrc, percent);
        rightDrive.setVelocity(-turnPrc, percent);

        // move motors to the desired position
        if (error > 2){
            leftDrive.spin(forward);
            rightDrive.spin(forward);
            wait(30, msec);
        }else{
            leftDrive.stop();
            rightDrive.stop();
            wait(1000, msec);
            angle = getAngle();
            error = angleRef - angle;
            if(error < -1){
            leftDrive.spin(reverse);
            rightDrive.spin(reverse);
            wait(150, msec);
            }
            leftDrive.stop();
            rightDrive.stop();
            printf("############LAST############\n");
            printf("Angle: %f\n", angle);
            printf("Error = %f - %f = %f\n", angleRef, angle, error);
            break;
        }
        /*
        if (error > 10){
            leftDrive.spin(forward);
            rightDrive.spin(forward);
            wait(30, msec);
        }else if(error < -1){
            leftDrive.spin(reverse);
            rightDrive.spin(reverse);
            wait(30, msec);
        }else{
            leftDrive.stop();
            rightDrive.stop();
            break;
        }
        */
        // wait(150, msec);
    }
}

void testPID(){
    PID pid = PID(1.1, 0.7, 0);
    pid.setTarget(90);
    double angle = getAngle();
    double error = 0;

    leftDrive.setVelocity(7, percent);
    rightDrive.setVelocity(-7, percent);

    leftDrive.spin(forward);
    rightDrive.spin(forward);
    

    while(1){
        error = pid.calc(90, angle);
        angle = getAngle();
        printf("Angle: %f\n", angle);
        printf("Error: %f\n", error);
        if (error < 2){
            printf("########IM HERE########\n");
            leftDrive.stop();
            rightDrive.stop();
            wait(50, msec);
            break;
        }
    }

}

void auton(){
    // Drivetrain
    InertialSensor.calibrate();
    Drivetrain.setDriveVelocity(25, percent);
    Drivetrain.setTurnVelocity(10, percent);

    Drivetrain.driveFor(forward, 35, distanceUnits::cm);
    /*
    printf("Angle: %f\n", InertialSensor.rotation());
    Drivetrain.turnFor(right, 45, rotationUnits::deg);
    printf("Angle: %f\n", InertialSensor.rotation());
    Drivetrain.driveFor(forward, 75, distanceUnits::cm);
    */


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