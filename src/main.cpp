#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h" 
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "pros/vision.h" // IWYU pragma: keep

#include "robodash/api.h" // GUI

// External reference to the cat image
extern "C" {
    extern const lv_img_dsc_t cat_img;
}

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-5, 1, 3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-13, -12, 15}, pros::MotorGearset::blue);
pros::Motor leftIntakeBottom(10, pros::MotorGearset::blue);
pros::Motor leftIntakeTop(7, pros::MotorGearset::green);
pros::Motor rightIntakeBottom(9, pros::MotorGearset::green);
pros::Optical color_sensor(20);

pros::Distance park_distance(8); //base distance is around 200 mm. With ball is around 50-120

pros::adi::DigitalOut intake('A', false); //,maybe true for size
pros::adi::DigitalOut doinker('F', false); 
pros::adi::DigitalOut expansion('D', false);
pros::adi::DigitalOut wings('E', true);
pros::adi::DigitalOut middleDescore('G', false);
lemlib::ExpoDriveCurve throttle_curve(3,
                                     6,
                                     1.019 
);

lemlib::ExpoDriveCurve steer_curve(3,
                                  6,
                                  1.016
);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              10,
                              lemlib::Omniwheel::NEW_325,
                              450,
                              2
);

pros::Imu imu(6);
pros::Rotation vertical_encoder(-17);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
);



// Forward declarations for autonomous routines
void left_auton();
void right_auton();
void testing_auton();
void skillsAutonomous();

// Create autonomous selector with all available routines
// First parameter is the storage name - required for SD card persistence!
// The name is used to create a file on the SD card to save your selection
rd::Selector selector("auton_selector", {
    {"Left Auton", left_auton},
    {"Right Auton", right_auton},
    {"Testing Auton", testing_auton},
    {"Skills", skillsAutonomous},
});

// Create cat image view - this appears as a separate tab you can swipe to
rd::Image cat_view(&cat_img, "Cat :3");

// Callback to print when an auton is selected
void on_auton_selected(std::optional<rd::Selector::routine_t> routine) {
    if (routine.has_value()) {
        controller.print(1, 0, "Sel: %s", routine->name.c_str());
        
        // Manually save the selection to SD card since RoboDash may not be doing it
        FILE* file = fopen("/usd/auton_selector", "w");
        if (file != nullptr) {
            fprintf(file, "%s", routine->name.c_str());
            fclose(file);
            controller.clear_line(2);
            controller.print(2, 0, ">> SAVED OK <<");
        } else {
            controller.clear_line(2);
            controller.print(2, 0, ">> SAVE FAILED <<");
        }
    } else {
        controller.print(1, 0, "No selection");
    }
}

void initialize() { 
    pros::delay(100);
    // DO NOT initialize PROS LCD - it conflicts with RoboDash selector
    // pros::lcd::initialize(); // COMMENTED OUT to allow RoboDash to work
    
    // Check SD card status and print to controller
    if (pros::usd::is_installed()) {
        controller.print(0, 0, "SD: OK");
        
        // Check if the selector file exists
        FILE* file = fopen("/usd/auton_selector", "r");
        if (file != nullptr) {
            controller.print(0, 8, "File exists");
            fclose(file);
        } else {
            controller.print(0, 8, "No file");
        }
    } else {
        controller.print(0, 0, "NO SD CARD!");
    }
    
    // Try to manually load saved selection from SD card (try multiple paths)
    const char* load_paths[] = {"/usd/auton.txt", "/usd/auton_selector", "auton.txt"};
    bool loaded = false;
    
    for (int i = 0; i < 3 && !loaded; i++) {
        FILE* file = fopen(load_paths[i], "r");
        if (file != nullptr) {
            char saved_name[50];
            if (fgets(saved_name, sizeof(saved_name), file) != nullptr) {
                // Find and select the matching auton
                if (strcmp(saved_name, "Left Auton") == 0) {
                    selector.next_auton(false);
                    selector.prev_auton(false); // Go to first (Left Auton)
                } else if (strcmp(saved_name, "Right Auton") == 0) {
                    selector.next_auton(false); // Go to second
                } else if (strcmp(saved_name, "Testing Auton") == 0) {
                    selector.next_auton(false);
                    selector.next_auton(false); // Go to third
                } else if (strcmp(saved_name, "Skills") == 0) {
                    selector.next_auton(false);
                    selector.next_auton(false);
                    selector.next_auton(false); // Go to fourth
                }
                controller.print(2, 0, "Loaded (p%d)", i);
                loaded = true;
            }
            fclose(file);
        }
    }
    
    // Register callback to know when auton is selected
    selector.on_select(on_auton_selected);
    
    // Show the autonomous selector on the brain screen
    // TAP an auton on the screen to select it
    selector.focus();
    
    // Calibrate chassis (this runs in background)
	chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::c::motor_set_brake_mode(1, pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

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
    chassis.moveToPoint(0, 48, 2000);
}


void giveAWP() {
    // Add your autonomous routine here
    chassis.setPose(0, 0, 0);
    wings.set_value(false);
    chassis.moveToPoint(0, 3, 3000, {.maxSpeed = 120});
    // AWP routine implementation...
}

void soloAWP() {
    chassis.setPose(0, 0, 0);
    //start facing the right side wall, bot is perpendicular with around half of the bot sticking out of park zone
    pros::delay(100);
    //benckl t
    wings.set_value(false);
    doinker.set_value(true);
    chassis.moveToPoint(0, 34.5, 3000, {.maxSpeed = 100});
    pros::delay(100);
    chassis.turnToHeading(65, 500);
    scoreHigh();
    chassis.moveToPoint(9, 34.5, 1000, {.maxSpeed = 120}); //going into matchload
    pros::delay(600);
    chassis.moveToPoint(-24, 30, 2000, {.forwards = false, .maxSpeed = 120}); // long tube 1
    pros::delay(700);
    doinker.set_value(false);
    wings.set_value(true);
    pros::delay(700);
    wings.set_value(false);
    chassis.moveToPoint(-13, 31.2, 1000, {.forwards = true, .maxSpeed = 127}); //get out of long tube
    chassis.turnToHeading(180, 600);
    chassis.moveToPoint(-27, 7, 900, {.forwards = true, .maxSpeed = 100}); //first three balls in middle
    chassis.moveToPoint(-28, 4, 300, {.forwards = true, .maxSpeed = 100});
    chassis.turnToHeading( 160, 400); //turn to next 3
    chassis.moveToPoint(-24, -42, 2000, {.forwards = true, .maxSpeed = 127}); //next three balls in matchload
    pros::delay(100);
    chassis.turnToHeading( 135, 200); //turn to next 3
    chassis.moveToPoint(-37.5, -33.2, 1000, {.forwards = false, .maxSpeed = 120});
    doinker.set_value(true);
    chassis.turnToHeading( 125, 100); //turn to next 3  
    reverseMotors();
    pros::delay(100);
    scoreMiddleHigh();
    pros::delay(800);
    scoreHigh();
    chassis.moveToPoint(-3, -69.5, 2000, {.forwards = true, .maxSpeed = 127});
    chassis.turnToHeading(-270, 300); //turn mid goal
    // chassis.moveToPoint(13, -69.5, 2000, {.forwards = true, .maxSpeed = 127});
    // pros::delay(900);
    // chassis.moveToPoint(-25, -68.5, 2000, {.forwards = false, .maxSpeed = 127});
    // pros::delay(500);
    // wings.set_value(true);
}

void left_auton() {
    chassis.setPose(0, 0, 0);
    wings.set_value(false);
    scoreHigh();
    pros::delay(100);
    chassis.moveToPoint(0, 10, 3000, {.maxSpeed = 120});
    chassis.moveToPoint(-6, 28, 3000, {.maxSpeed = 65});
    pros::delay(700);
    doinker.set_value(true);
    pros::delay(600);
    chassis.moveToPoint(-6, 18, 3000, {.forwards = false, .maxSpeed = 120});
    chassis.turnToHeading(250, 1000);
    chassis.moveToPoint(-36.5, 10, 3000, {.maxSpeed = 102});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-36.7, -1.4, 2000, {.maxSpeed = 100});
    pros::delay(600);
    chassis.moveToPoint(-36.1, 28.5, 1000, {.forwards = false, .maxSpeed = 120});
    pros::delay(1000);
    wings.set_value(true);
    reverseMotorsButTop();
    pros::delay(200);
    scoreHigh();
    chassis.moveToPoint(-36.1, 28.8, 1000, {.forwards = false, .maxSpeed = 100});
    pros::delay(1500);
    reverseMotorsButTop();
    pros::delay(50);
    scoreHigh();
    pros::delay(500);
    chassis.moveToPoint(-36.1, 15, 2000, {.forwards = true, .maxSpeed = 120});
    chassis.moveToPoint(-25.8, 15, 2000, {.forwards = false, .maxSpeed = 120});
    chassis.turnToHeading(180, 300); //turn mid goal
    pros::delay(500);
    chassis.moveToPoint(-25.8, 45, 2000, {.forwards = false, .maxSpeed = 100});
    chassis.turnToHeading(180, 300); //turn mid goal
}

void right_auton() {
}

void autonomous() {
    intake.set_value(false);
    // Run the autonomous routine selected on the brain screen
    // Selection is saved to SD card and persists across reboots
    //right_auton();
    left_auton();
    //soloAWP();
    //giveAWP();
    //lateralTestingAuton();
}


void opcontrol() {
    intake.set_value(false);
    // Persistent state for intake piston toggle
    static bool intakePistonState = false;     // false = retracted, true = extended
    static bool prevIntakePistonButton = true; // previous loop state of Y button
    // Persistent state for doinker toggle
    static bool doinkerState = true;           // false = retracted, true = extended
    static bool prevDoinkerButton = true;      // previous loop state of doinker button (A)
    // Persistent state for expansion toggle
    static bool expansionState = false;       // false = retracted, true = extended (deployed)
    static bool prevExpansionButton = true;   // previous loop state of expansion button (B)
    // Persistent state for wing toggle
    static bool wingState = false;            // false = retracted, true = extended
    static bool prevWingButton = true;        // previous loop state of wing button (R2)
    // Persistent state for middleDescore toggle
    static bool middleDescoreState = false;   // false = retracted, true = extended
    static bool prevMiddleDescoreButton = true; // previous loop state of middleDescore button
    // Persistent state for park macro
    static bool parkMacroRunning = false;     // true when park macro is active
    
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakePistonButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool doinkerButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool expansionButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool middleDescoreButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
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
        } else if (intakeOne) {
            leftIntakeBottom.move(-127); //bottm
            leftIntakeTop.move(-127); //middle
            rightIntakeBottom.move(127); //top

        } else if (intakeTwo) {
            leftIntakeBottom.move(-127); //bottm
            leftIntakeTop.move(-127); //middle
            rightIntakeBottom.move(-127); //top


        } else if (intakeThree) {
            leftIntakeBottom.move(127);
            leftIntakeTop.move(127);
            rightIntakeBottom.move(-127);
        }

        else {
             leftIntakeBottom.move(0);
            leftIntakeTop.move(0);
            rightIntakeBottom.move(0);
        }

        if (intakePistonButton) {
            // Edge-triggered toggle: only toggle when button transitions from not pressed to pressed
            if (!prevIntakePistonButton) {
                intakePistonState = !intakePistonState;
                intake.set_value(intakePistonState);
                pros::delay(100); 
            }
        } 
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
        
        if (wingButton) {
            // Edge-triggered toggle for wings
            if (!prevWingButton) {
                wingState = !wingState;
                wings.set_value(wingState);
                pros::delay(100);
            }
        }
        
        if (middleDescoreButton) {
            // Edge-triggered toggle for middleDescore
            if (!prevMiddleDescoreButton) {
                middleDescoreState = !middleDescoreState;
                middleDescore.set_value(middleDescoreState);
                pros::delay(100);
            }
        }

        chassis.arcade(leftY, rightX);
        prevIntakePistonButton = intakePistonButton; // update edge detector
        prevDoinkerButton = doinkerButton; // update edge detector for doinker
        prevExpansionButton = expansionButton; // update edge detector for expansion
        prevWingButton = wingButton; // update edge detector for wing
        prevMiddleDescoreButton = middleDescoreButton; // update edge detector for middleDescore
        pros::delay(5);
    }
}

