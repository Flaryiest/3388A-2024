#include "autons.hpp"
#include "setup.hpp"
#include "pros/rtos.hpp"

void scoreHigh() {
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(127); //top
}

void suck() {
    leftIntakeBottom.move(127); //bottm
    leftIntakeTop.move(127); //middle
    rightIntakeBottom.move(127); //top
}

static bool odomLiftStateAuton = false; // tracks odomLift state for toggle

void liftOdom() {
    odomLiftStateAuton = !odomLiftStateAuton;
    odomLift.set_value(odomLiftStateAuton);
}

void scoreMiddleHigh() {
    //reverseMotorsButTop();
    reverseMotors();
    pros::delay(100);
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-70); //middle
    rightIntakeBottom.move(-70); //top
}

void reverseMotors() {
    leftIntakeBottom.move(127); //bottm
    leftIntakeTop.move(127); //middle
    pros::delay(100); // delay top motor by 100ms
    rightIntakeBottom.move(-127); //top
}

void reverseMotorsSlow() {
    leftIntakeBottom.move(80); //bottm
    leftIntakeTop.move(127); //middle
    pros::delay(100); // delay top motor by 100ms
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
    rightIntakeBottom.move(-90); //top
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

void skillsDrivePastPark() {
    // Drive straight until the color sensor crosses the park zone line twice,
    // then drive 3 more inches and stop.
    // Park zone lines are colored (red or blue) on the gray field tiles.
    
    int crossingCount = 0;
    bool onLine = false;
    float targetHeading = chassis.getPose().theta; // maintain current heading
    
    color_sensor.set_led_pwm(100); // ensure LED is on for detection
    
    while (crossingCount < 2) {
        // Drive straight with IMU heading correction
        float headingError = targetHeading - chassis.getPose().theta;
        // Normalize heading error to -180..180
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        int correction = (int)(headingError * 2.0); // P-correction
        left_motors.move(70 + correction);
        right_motors.move(70 - correction);
        
        // Check color sensor for park zone line
        double hue = color_sensor.get_hue();
        int proximity = color_sensor.get_proximity();
        
        // Detect colored line: red (hue < 30 or > 330) or blue (hue 80-250)
        bool isColored = (proximity > 60) && 
                         ((hue < 30 || hue > 330) || (hue > 80 && hue < 250));
        
        if (isColored && !onLine) {
            onLine = true; // entered the colored line
        } else if (!isColored && onLine) {
            onLine = false; // exited the colored line
            crossingCount++;
            if (crossingCount == 1) {
                doinker.set_value(true); // put down doinker after first crossing
            }
        }
        
        pros::delay(10);
    }
    
    // Stop driving
    left_motors.move(0);
    right_motors.move(0);
    // Drive 5 more inches forward
    moveDistance(2, 2000, 60);
}

void skillsAutonomous() {
    chassis.setPose(0, 0, 0);
    hood.set_value(true);
    wings.set_value(false);
    doinker.set_value(true);
    chassis.moveToPoint(0, 33.75, 3000, {.maxSpeed = 80});
    pros::delay(100);
    chassis.turnToHeading(360 - 74, 1000);
    scoreHigh();
    chassis.moveToPoint(-5.5, 37, 1000, {.maxSpeed = 90, .minSpeed = 30}); //going into matchload
    pros::delay(1600);
    chassis.moveToPoint(-0, 37, 1000, {.maxSpeed = 90, .minSpeed = 30}); //going into matchload
    chassis.moveToPose(14, 41, -90, 3000, {.forwards = false, .maxSpeed = 90, .minSpeed = 50});
    doinker.set_value(false);
    chassis.moveToPoint(84, 40, 4000, {.forwards = false, .maxSpeed = 110, .minSpeed = 20});
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(85, 33, 4000, {.forwards = false, .maxSpeed = 110, .minSpeed = 20});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(76, 33, 4000, {.forwards = false, .maxSpeed = 110, .minSpeed = 20});
}






void testing_auton() {
}

void lateralTestingAuton() {
    chassis.setPose(0, 0, 0);
        chassis.moveToPose(
        24,
        48,
        90,
        4000,
        {.minSpeed=30}
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
}

void left_split() {
    chassis.setPose(0, 0, 0);
}

void right_auton() {
    chassis.setPose(0, 0, 0);
    hood.set_value(true);
    wings.set_value(false);
    doinker.set_value(true);
    chassis.moveToPoint(0, 34.8, 3000, {.maxSpeed = 80});
    pros::delay(100);
    chassis.turnToHeading(74, 1000);
    scoreHigh();
    chassis.moveToPoint(5.3, 38.35, 1000, {.maxSpeed = 90, .minSpeed = 40}); //going into matchload
    pros::delay(875);
    chassis.moveToPoint(-22.2,33, 1800, {.forwards = false, .maxSpeed = 127, .minSpeed = 70}); // long tube 1
    chassis.waitUntil(28);
    doinker.set_value(false);
    hood.set_value(false);
    chassis.turnToHeading(90, 400);
    pros::delay(400);
    left_motors.move(-60);
    right_motors.move(-60);
    pros::delay(420);
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(320);
    scoreHigh();
    chassis.setPose(0, 0, 0); // reset
    chassis.moveToPoint(-9, 10, 2000, {.maxSpeed = 127, .minSpeed = 70});
    chassis.turnToHeading(0, 800);
    wings.set_value(true);
    chassis.moveToPoint(-14, -14, 20000, {.forwards = false, .maxSpeed = 127, .minSpeed = 70});
    while (true) {
        chassis.moveToPose(-14, -17, 0, 20000, {.forwards = false, .maxSpeed = 127, .minSpeed = 70});
    }
}

void newSAWP() {
    chassis.setPose(0, 0, 0);
    hood.set_value(true);
    wings.set_value(false);
    doinker.set_value(true);
    chassis.moveToPoint(0, 34.8, 3000, {.maxSpeed = 80});
    pros::delay(100);
    chassis.turnToHeading(74, 1000);
    scoreHigh();
    chassis.moveToPoint(5.3, 38.35, 1000, {.maxSpeed = 90, .minSpeed = 40}); //going into matchload
    pros::delay(725);
    chassis.moveToPoint(-22.2,33, 1800, {.forwards = false, .maxSpeed = 127, .minSpeed = 70}); // long tube 1
    chassis.waitUntil(28);
    doinker.set_value(false);
    hood.set_value(false);
    chassis.turnToHeading(90, 400);
    pros::delay(400);
    left_motors.move(-60);
    right_motors.move(-60);
    pros::delay(420);
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(120);
    scoreHigh();
    chassis.setPose(0, 0, 0); // reset
    chassis.turnToHeading(80, 800, {.maxSpeed = 120, .minSpeed = 50});
    hood.set_value(true);
    suck();
    pros::delay(200);
    scoreHigh();
    chassis.moveToPoint(13, 2.4, 1000, {.maxSpeed = 90, .minSpeed = 45}); // long tube 1
    chassis.moveToPoint(61,1, 2500, {.maxSpeed = 90, .minSpeed = 30}); // second pair of 3 balls
    chassis.waitUntil(41);
    doinker.set_value(true);
    chassis.turnToHeading(35, 400);
    chassis.moveToPoint(90.3,13, 2000, {.maxSpeed = 100}); // second pair of 3 balls
    chassis.turnToHeading(0, 300);
    chassis.moveToPoint(87.6,-8, 400, {.forwards = false, .maxSpeed = 127}); // second pair of 3 balls
    chassis.waitUntil(8);
    hood.set_value(false);
    chassis.waitUntil(1);
    chassis.turnToHeading(0, 200);
    pros::delay(200);
    left_motors.move(-100);
    right_motors.move(-100);
    pros::delay(320);
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(350);
    hood.set_value(true);
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(-0.2, 38.3, 0, 1550, {.maxSpeed = 120, .minSpeed = 30}); // second pair of 3 balls
    pros::delay(1550);
    chassis.moveToPoint(-37, -9.7, 1500, {.forwards = false, .maxSpeed = 127, .minSpeed = 80}); // second pair of 3 balls
    chassis.waitUntil(61);
    scoreMiddleHigh();
    chassis.turnToHeading(45, 500);
    moveDistance(-8, 1000, 70);
}
