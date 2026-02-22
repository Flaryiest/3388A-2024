#include "setup.hpp"
#include "autons.hpp"
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

void initialize() { 
    pros::delay(100);
    pros::lcd::initialize();
    color_sensor.set_led_pwm(100);

    // Calibrate chassis (this runs in background)
	chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::c::motor_set_brake_mode(1, pros::E_MOTOR_BRAKE_HOLD);

    // Print robot position to brain screen continuously
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Heading: %.2f", chassis.getPose().theta);
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0); 
    // Run the autonomous routine selected on the brain screen
    // Selection is saved to SD card and persists across reboots
    //right_auton();
    //skillsAutonomous();
    //left_auton();
    newSAWP();
    //left_split();
    //giveAWP();
    //lateralTestingAuton();
}


void opcontrol() {
    hood.set_value(false);
    odomLift.set_value(true);
    // Persistent state for doinker toggle
    static bool doinkerState = false;           // false = retracted, true = extended
    static bool prevDoinkerButton = true;      // previous loop state of doinker button (A)


    // Persistent state for middleDescore toggle
    static bool middleDescoreState = false;   // false = retracted, true = extended
    static bool prevMiddleDescoreButton = true; // previous loop state of middleDescore button
    // Persistent state for odomLift toggle
    static bool odomLiftState = false;        // false = off/down (matches initial value)
    static bool prevOdomLiftButton = true;    // previous loop state of odomLift button (DOWN)
    // Persistent state for intakeTwo reverse burst
    static bool intakeTwoBursting = false;    // true during initial reverse burst
    static uint32_t intakeTwoBurstStart = 0;  // when burst started
    static bool prevIntakeTwo = false;        // previous loop state of L2
    // Persistent state for intakeThree delayed top motor
    static bool intakeThreeDelaying = false;  // true during initial 100ms delay
    static uint32_t intakeThreeDelayStart = 0; // when delay started
    static bool prevIntakeThree = false;      // previous loop state of R1
    // Persistent state for park macro
    static bool parkMacroRunning = false;     // true when park macro is active
    
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakePistonButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        bool doinkerButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);

        bool middleDescoreButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool odomLiftButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool parkMacroButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // Park macro: when X is pressed, run intake in reverse at half speed until ball detected
        if (parkMacroButton) {
            int distance = park_distance.get();
            
            // Check if ball is detected (distance dropped from ~200 to ~70-100)
            if (distance < 120 && distance > 0) {
                // Ball detected, stop the macro
                pros::delay(22);
                leftIntakeBottom.move(0);
                leftIntakeTop.move(0);
                rightIntakeBottom.move(0);
                
                expansion.set_value(true);
                parkMacroRunning = false;
                controller.rumble("-");  // Short rumble to indicate ball detected
            } else {
                // Run intake motors in reverse at half speed (opposite of intakeOne)
                // intakeOne runs: leftIntakeBottom(-127), leftIntakeTop(-127), rightIntakeBottom(127)
                // So opposite at half speed: leftIntakeBottom(63), leftIntakeTop(63), rightIntakeBottom(-63)
                leftIntakeBottom.move(80);
                leftIntakeTop.move(83);
                rightIntakeBottom.move(-127);
                parkMacroRunning = true;
            }
        } else if (parkMacroRunning) {
            // Button released while macro was running - stop motors
            leftIntakeBottom.move(0);
            leftIntakeTop.move(0);
            rightIntakeBottom.move(0);
            parkMacroRunning = false;
        } else {
            // ===== NORMAL INTAKE CONTROL =====
            if (intakeOne || intakePistonButton) {
                leftIntakeBottom.move(-127); //bottom
                leftIntakeTop.move(-127); //middle
                rightIntakeBottom.move(127); //top normal scoring

            } else if (intakeTwo) {
                // On first press, run reverse burst for 100ms
                if (!prevIntakeTwo) {
                    intakeTwoBursting = true;
                    intakeTwoBurstStart = pros::millis();
                }
                if (intakeTwoBursting && (pros::millis() - intakeTwoBurstStart < 100)) {
                    // Reverse burst (same as intakeThree)
                    leftIntakeBottom.move(127);
                    leftIntakeTop.move(127);
                    rightIntakeBottom.move(-127);
                } else {
                    intakeTwoBursting = false;
                    // Normal intakeTwo
                    leftIntakeBottom.move(-127); //bottom
                    leftIntakeTop.move(-60); //middle
                    rightIntakeBottom.move(-60); //top
                }

            } else if (intakeThree) {
                // On first press, delay rightIntakeBottom for 100ms
                if (!prevIntakeThree) {
                    intakeThreeDelaying = true;
                    intakeThreeDelayStart = pros::millis();
                }
                leftIntakeBottom.move(80);
                leftIntakeTop.move(127);
                if (intakeThreeDelaying && (pros::millis() - intakeThreeDelayStart < 100)) {
                    rightIntakeBottom.move(0); // don't spin top for first 100ms
                } else {
                    intakeThreeDelaying = false;
                    rightIntakeBottom.move(-127);
                }
            } else {
                leftIntakeBottom.move(0);
                leftIntakeTop.move(0);
                rightIntakeBottom.move(0);
            }
        }

        hood.set_value(!intakePistonButton);
        if (doinkerButton) {
            // Edge-triggered toggle for doinker
            if (!prevDoinkerButton) {
                doinkerState = !doinkerState;
                doinker.set_value(doinkerState);
                pros::delay(100);
            }
        }
        
        // Expansion follows intakeThree (R1): ON when pressed, OFF when released
        expansion.set_value(intakeThree);
        
        // Wings: OFF when not pressed, ON when pressed (hold to activate)
        wings.set_value(wingButton);
        
        if (middleDescoreButton) {
            // Edge-triggered toggle for middleDescore
            if (!prevMiddleDescoreButton) {
                middleDescoreState = !middleDescoreState;
                middleDescore.set_value(middleDescoreState);
                pros::delay(100);
            }
        }
        
        if (odomLiftButton) {
            // Edge-triggered toggle for odomLift
            if (!prevOdomLiftButton) {
                odomLiftState = !odomLiftState;
                odomLift.set_value(odomLiftState);
                pros::delay(100);
            }
        }
        
        chassis.arcade(leftY, rightX);
        prevIntakeTwo = intakeTwo; // update edge detector for intakeTwo burst
        prevIntakeThree = intakeThree; // update edge detector for intakeThree delay
        prevDoinkerButton = doinkerButton; // update edge detector for doinker


        prevMiddleDescoreButton = middleDescoreButton; // update edge detector for middleDescore
        prevOdomLiftButton = odomLiftButton; // update edge detector for odomLift
        pros::delay(5);
    }
}