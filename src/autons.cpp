#include "autons.hpp"
#include "setup.hpp"
#include "pros/rtos.hpp"

void scoreHigh() {
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(127); //top
}

void scoreMiddleHigh() {
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-60); //middle
    rightIntakeBottom.move(-60); //top
}

void reverseMotors() {
    leftIntakeBottom.move(127); //bottm
    leftIntakeTop.move(127); //middle
    rightIntakeBottom.move(-127); //top
}

void reverseMotorsSlow() {
    leftIntakeBottom.move(127); //bottm
    leftIntakeTop.move(127); //middle
    rightIntakeBottom.move(-127); //top
}

void stopAllMotors() {
    leftIntakeBottom.move(0); //bottm
    leftIntakeTop.move(0); //middle
    rightIntakeBottom.move(0); //top
}

void reverseMotorsButTop() {
    leftIntakeBottom.move(127); //bottm
    leftIntakeTop.move(127); //middle
    rightIntakeBottom.move(0); //top
}

// Move forward/backward a certain distance in the current heading direction
void moveDistance(float distance, int timeout, float maxSpeed) {
    lemlib::Pose currentPose = chassis.getPose();
    float heading = currentPose.theta * M_PI / 180.0; // Convert to radians
    
    // Calculate target point based on current heading
    float targetX = currentPose.x + distance * sin(heading);
    float targetY = currentPose.y + distance * cos(heading);
    
    chassis.moveToPoint(targetX, targetY, timeout, {
        .forwards = (distance > 0),
        .maxSpeed = maxSpeed
    });
}

void skillsAutonomous() {
    // Add your skills autonomous routine here
    chassis.setPose(0, 0, 0);
    // Skills routine implementation...
}






void testing_auton() {
    // ===== PID TUNING AUTONOMOUS =====
    // Instructions:
    // 1. First tune ANGULAR PID (turning), then tune LATERAL PID (driving straight)
    // 2. Comment/uncomment test sections below as needed
    // 3. Watch the robot and adjust PID values in the controller settings at the top of the file
    
    chassis.setPose(0, 0, 0);
    
    pros::delay(6000);
    // ===== ANGULAR PID TUNING (Turn Tests) =====
    // Use these to tune angular controller (kP, kD, then kI if needed)
    // FLOWCHART:
    // - If robot oscillates (wiggles back and forth): INCREASE kD
    // - If robot is too slow to reach target: INCREASE kP
    // - If robot overshoots: DECREASE kP
    // - If robot settles but not at target (steady-state error): ADD small kI (0.01-0.05)
    
    // Test 1: 90 degree turn
    controller.print(0, 0, "Turn 90");
    chassis.turnToHeading(90, 2000);
    pros::delay(2000);
    
    // Test 2: 180 degree turn
    controller.print(0, 0, "Turn 180");
    chassis.turnToHeading(180, 2000);
    pros::delay(2000);
    
    // Test 3: 270 degree turn
    controller.print(0, 0, "Turn 270");
    chassis.turnToHeading(270, 2000);
    pros::delay(2000);
    
    // Test 4: Back to 0
    controller.print(0, 0, "Turn 0");
    chassis.turnToHeading(0, 2000);
    pros::delay(2000);
    
    
    // ===== LATERAL PID TUNING (Straight Line Tests) =====
    // Uncomment these AFTER tuning angular PID
    // Use these to tune lateral controller (kP, kD, slew, then kI if needed)
    // FLOWCHART:
    // - If robot oscillates (moves jerky): INCREASE kD
    // - If robot is too slow: INCREASE kP
    // - If robot overshoots: DECREASE kP
    // - If wheels slip: DECREASE slew (start at 20 and go down)
    // - If robot tips: DECREASE slew
    // - If robot settles but not at target: ADD small kI (0.01-0.05)
    
    controller.print(0, 0, "Tuning Done!");
}

void lateralTestingAuton() {
    chassis.setPose(0, 0, 0);
        chassis.moveToPose(
        24,
        48,
        90,
        4000,
        {.minSpeed=30}
        // a minSpeed of 72 means that the chassis will slow down as
        // it approaches the target point, but it won't come to a full stop

        // an earlyExitRange of 8 means the movement will exit 8" away from
        // the target point
    );
}


void giveAWP() {
    // Add your autonomous routine here
    chassis.setPose(0, 0, 0);
    hood.set_value(false);
    chassis.moveToPoint(0, 3, 3000, {.maxSpeed = 120});
    // AWP routine implementation...
}

void left_auton() {
    chassis.setPose(0, 0, 0);
    hood.set_value(true);
    scoreHigh();
    pros::delay(100);
    chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 120});
    chassis.moveToPoint(-6, 28, 3000, {.maxSpeed = 65});
    pros::delay(700);
    doinker.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(-6, 18, 3000, {.forwards = false, .maxSpeed = 120});
    chassis.turnToHeading(250, 1000);
    chassis.moveToPoint(-37.5, 11, 3000, {.maxSpeed = 102});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-39, 1, 2000, {.maxSpeed = 80});
    pros::delay(600);
    chassis.moveToPoint(-38.7, 37, 2000, {.forwards = false, .maxSpeed = 120});
    pros::delay(1000);
    hood.set_value(false);
    scoreHigh();
    pros::delay(2000);
    chassis.moveToPoint(-39, 20, 2000, {.forwards = true, .maxSpeed = 120});
    chassis.moveToPoint(-26.5, 20, 2000, {.forwards = false, .maxSpeed = 120});
    chassis.turnToHeading(180, 300); //turn mid goal
    pros::delay(500);
    chassis.moveToPoint(-26.5, 63, 3000, {.forwards = false, .maxSpeed = 110});
    chassis.turnToHeading(180, 300); //turn mid goal
}

void right_auton() {
}

void newSAWP() {
    chassis.setPose(0, 0, 0);
    pros::delay(100);
    hood.set_value(true);
    doinker.set_value(true);


    chassis.moveToPoint(0, 34.2, 3000, {.maxSpeed = 80});
    pros::delay(650);
    chassis.turnToHeading(271, 1000);
    scoreHigh();
    chassis.moveToPoint(-7.2, 38, 1000, {.maxSpeed = 80, .minSpeed = 70}); //going into matchload
    // pros::delay(400);
    // chassis.moveToPoint(23.0,32.2, 2000, {.forwards = false, .maxSpeed = 75, .minSpeed = 60}); // long tube 1
    // pros::delay(700);
    // doinker.set_value(false);
    // hood.set_value(false);
    // pros::delay(1000);
    // hood.set_value(true);
    // chassis.moveToPose(
    //     21,
    //     6,
    //     -180,
    //     2000,
    //     {.maxSpeed = 110, .minSpeed=70}
    //     // a minSpeed of 72 means that the chassis will slow down as
    //     // it approaches the target point, but it won't come to a full stop

    //     // an earlyExitRange of 8 means the movement will exit 8" away from
    //     // the target point
    // );
    // chassis.moveToPoint(21, -35, 3000, {.forwards = true, .maxSpeed = 105}); //get out of long tube
    // pros::delay(125);
    // chassis.turnToHeading(45, 1000);
    // pros::delay(600);
    // moveDistance(17.4,  2000);
    // pros::delay(600);
    // reverseMotorsSlow();
    // pros::delay(1000);

    // // out of mid goal and to the match loader
    // moveDistance(-8, 500);
    // scoreHigh();
    // chassis.moveToPoint(4, -50, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed=20}); //get out of long tube
    // pros::delay(300);
    // doinker.set_value(true);
    // chassis.turnToHeading(-90, 1000);
    // pros::delay(100);
    // chassis.moveToPoint(-14, -50, 1000, {.forwards = true, .maxSpeed = 110, .minSpeed=60}); //get out of long tube
    // pros::delay(425);
    // chassis.moveToPoint(22, -57, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed=60}); //get out of long tube
    // pros::delay(600);
    // hood.set_value(false);
    // chassis.moveToPoint(30, -57, 1000, { .forwards = false,.maxSpeed = 100, .minSpeed=20}); //get out of long tube
}
