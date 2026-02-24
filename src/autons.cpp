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
    // reverseMotors();
    // pros::delay(150);
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
    // cross park and get 6
    chassis.setPose(0, 0, 0);
    hood.set_value(true);
    liftOdom();
    scoreHigh();
    skillsDrivePastPark();
    pros::delay(400);

    // reset off park
    liftOdom();
    //stopAllMotors();
    moveDistance(-8, 1500, 40);
    pros::delay(300);
    chassis.setPose(0, 0, 0);

    //pros::delay(3000); // change to like 100 after
    // //line up to directly and back toward mid goal
    //chassis.moveToPoint(0, 10, 1500);
    // chassis.turnToHeading(45, 1000);
    // chassis.moveToPoint(-38, -24, 3000, {.forwards = false, .maxSpeed = 70});
    // chassis.turnToHeading(135, 1300);
    //chassis.moveToPoint(-38, -24, 3000, {.forwards = false, .maxSpeed = 70});
    
    
    

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
    chassis.moveToPoint(-36.2, 11, 3000, {.maxSpeed = 102});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-36.8, -2, 2000, {.maxSpeed = 80});
    pros::delay(600);
    chassis.moveToPoint(-35.2, 36, 2000, {.forwards = false, .maxSpeed = 65});
    pros::delay(1000);
    hood.set_value(false);
    scoreHigh();
    pros::delay(2000);
    reverseMotors();
    pros::delay(150);
    scoreHigh();

}

void left_split() {
    chassis.setPose(0, 0, 0);
    pros::delay(100);
    hood.set_value(true);
    doinker.set_value(true);


    chassis.moveToPoint(0, 34.2, 3000, {.maxSpeed = 60});
    pros::delay(700);
    chassis.turnToHeading(-74, 1000);
    pros::delay(100);
    scoreHigh();
    chassis.moveToPoint(-7.2, 38, 1000, {.maxSpeed = 70, .minSpeed = 50}); //going into matchload
    pros::delay(800);
    chassis.moveToPoint(23.0,31.6, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 20}); // long tube 1
    pros::delay(850);
    doinker.set_value(false);
    hood.set_value(false);
    pros::delay(1300);
    chassis.moveToPoint(10,32.2, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 20}); // long tube 1
    hood.set_value(true);
    chassis.moveToPose(
        23.3,
        7,
        180,
        3000,
        {.maxSpeed = 70, .minSpeed=20}
    );
    chassis.turnToHeading(-45, 2000);
    chassis.moveToPoint(26.3, 7, 2000);
    chassis.turnToHeading(-45, 2000);
    chassis.moveToPoint(32.8, -4.3, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 20});
    pros::delay(1000);
    reverseMotors();
    pros::delay(200);
    scoreMiddleHigh();

}

void right_auton() {
    chassis.setPose(0, 0, 0);
    hood.set_value(true);
    scoreHigh();
    pros::delay(100);
    chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 120});
    chassis.moveToPoint(6, 28, 3000, {.maxSpeed = 65});
    pros::delay(700);
    doinker.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(6, 18, 3000, {.forwards = false, .maxSpeed = 120});
    chassis.turnToHeading(360-250, 1000);
    chassis.moveToPoint(38.5, 12, 3000, {.maxSpeed = 102});
    chassis.turnToHeading(360-180, 1000);
    chassis.moveToPoint(38.6, -2, 2000, {.maxSpeed = 80});
    pros::delay(600);
    chassis.moveToPoint(37.7, 36, 2000, {.forwards = false, .maxSpeed = 60});
    pros::delay(1200);
    hood.set_value(false);
    scoreHigh();
    pros::delay(2000);
    reverseMotors();
    pros::delay(150);
    scoreHigh();
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
    chassis.moveToPoint(5.5, 38.45, 1000, {.maxSpeed = 90, .minSpeed = 30}); //going into matchload
    pros::delay(650);
    chassis.moveToPoint(-22.2,31.7, 1800, {.forwards = false, .maxSpeed = 127, .minSpeed = 70}); // long tube 1
    chassis.waitUntil(28);
    doinker.set_value(false);
    hood.set_value(false);
    pros::delay(200);
    left_motors.move(-80);
    right_motors.move(-80);
    pros::delay(320);
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(220);
    scoreHigh();
    chassis.setPose(0, 0, 0); // reset

    chassis.turnToHeading(90, 800);
    hood.set_value(true);
    suck();
    pros::delay(200);
    scoreHigh();
    chassis.moveToPoint(23.5,0, 2000, {.maxSpeed = 90}); // long tube 1
    chassis.turnToHeading(90, 400);
    chassis.moveToPoint(66,-2, 2500, {.maxSpeed = 90}); // second pair of 3 balls
    chassis.waitUntil(40);
    doinker.set_value(true);
    chassis.moveToPoint(90.5,14, 2000, {.maxSpeed = 100}); // second pair of 3 balls
    chassis.turnToHeading(0, 400);
    chassis.moveToPoint(92.1,-4.5, 1200, {.forwards = false, .maxSpeed = 127}); // second pair of 3 balls
    chassis.waitUntil(12);
    hood.set_value(false);
    chassis.waitUntil(1);
    pros::delay(200);
    left_motors.move(-80);
    right_motors.move(-80);
    pros::delay(320);
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(100);
    hood.set_value(true);


    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(-1.5, 30.3, 2000, {.maxSpeed = 70, .minSpeed = 20}); // second pair of 3 balls
    pros::delay(1000);
    chassis.moveToPoint(-35.8, -18, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 80}); // second pair of 3 balls
    chassis.waitUntil(61);
    scoreMiddleHigh();
    chassis.turnToHeading(45, 500);




}
