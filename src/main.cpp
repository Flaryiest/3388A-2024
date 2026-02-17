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
    right_auton();
    //left_auton();
    //newSAWP();
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
    // Persistent state for expansion toggle
    static bool expansionState = false;       // false = retracted, true = extended (deployed)
    static bool prevExpansionButton = true;   // previous loop state of expansion button (B)

    // Persistent state for middleDescore toggle
    static bool middleDescoreState = false;   // false = retracted, true = extended
    static bool prevMiddleDescoreButton = true; // previous loop state of middleDescore button
    // Persistent state for odomLift toggle
    static bool odomLiftState = false;        // false = off/down (matches initial value)
    static bool prevOdomLiftButton = true;    // previous loop state of odomLift button (DOWN)
    // Persistent state for color sort toggle
    static bool colorSortEnabled = true;      // true = color sorting active
    static bool prevColorSortButton = true;   // previous loop state of color sort button (LEFT)
    static bool colorSortEjecting = false;    // true when actively ejecting wrong color
    static uint32_t ejectStartTime = 0;       // when eject started
    static uint32_t ejectCooldownEnd = 0;     // when cooldown ends (to avoid re-detecting same ball)
    // Color sort config: set to true to KEEP red (eject blue), false to KEEP blue (eject red)
    static const bool KEEP_RED = true;        // Change this based on your alliance color!
    static const int EJECT_DURATION_MS = 250; // How long to reverse top motor
    static const int EJECT_COOLDOWN_MS = 100; // Cooldown before detecting next ball
    // Persistent state for intakeTwo reverse burst
    static bool intakeTwoBursting = false;    // true during initial reverse burst
    static uint32_t intakeTwoBurstStart = 0;  // when burst started
    static bool prevIntakeTwo = false;        // previous loop state of L2
    // Persistent state for park macro
    static bool parkMacroRunning = false;     // true when park macro is active
    
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakePistonButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

        bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        bool doinkerButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        bool expansionButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool middleDescoreButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool odomLiftButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool colorSortButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
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
            // ===== COLOR SORT DETECTION (runs always, not just when intake pressed) =====
            // Check if we need to eject wrong color ball
            if (colorSortEnabled && !colorSortEjecting && pros::millis() > ejectCooldownEnd) {
                // Read color sensor hue (0-360)
                double hue = color_sensor.get_hue();
                int proximity = color_sensor.get_proximity();
                
                // Only check color if ball is close enough (proximity > 200)
                if (proximity > 100) {
                    bool isRed = (hue < 30 || hue > 330);  // Red hue range
                    bool isBlue = (hue > 80 && hue < 250); // Blue hue range
                    
                    // Check if we should eject this ball
                    bool shouldEject = (KEEP_RED && isBlue) || (!KEEP_RED && isRed);
                    
                    if (shouldEject) {
                        colorSortEjecting = true;
                        ejectStartTime = pros::millis();
                    }
                }
            }
            
            // ===== HANDLE EJECT (overrides normal intake control) =====
            if (colorSortEjecting) {
                if (pros::millis() - ejectStartTime < EJECT_DURATION_MS) {
                    // Eject: reverse top motor to spit out wrong color
                    leftIntakeBottom.move(-127); //bottom keeps running
                    leftIntakeTop.move(-30); //middle slowed
                    rightIntakeBottom.move(-127); //top reversed to eject
                } else {
                    // Eject done, start cooldown before detecting next ball
                    colorSortEjecting = false;
                    ejectCooldownEnd = pros::millis() + EJECT_COOLDOWN_MS;
                }
            }
            // ===== NORMAL INTAKE CONTROL (only when not ejecting) =====
            else if (intakeOne || intakePistonButton) {
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
                leftIntakeBottom.move(127);
                leftIntakeTop.move(127);
                rightIntakeBottom.move(-127);
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
        
        if (expansionButton) {
            // Edge-triggered toggle for expansion
            if (!prevExpansionButton) {
                expansionState = !expansionState;
                expansion.set_value(expansionState);
                pros::delay(100);
            }
        }
        
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
        
        if (colorSortButton) {
            // Edge-triggered toggle for color sort
            if (!prevColorSortButton) {
                colorSortEnabled = !colorSortEnabled;
                // Feedback: rumble to indicate state
                controller.rumble(colorSortEnabled ? "." : "-");
                pros::delay(100);
            }
        }

        chassis.arcade(leftY, rightX);
        prevIntakeTwo = intakeTwo; // update edge detector for intakeTwo burst
        prevDoinkerButton = doinkerButton; // update edge detector for doinker
        prevExpansionButton = expansionButton; // update edge detector for expansion

        prevMiddleDescoreButton = middleDescoreButton; // update edge detector for middleDescore
        prevOdomLiftButton = odomLiftButton; // update edge detector for odomLift
        prevColorSortButton = colorSortButton; // update edge detector for color sort
        pros::delay(5);
    }
}