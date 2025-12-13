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

pros::MotorGroup left_motors({1, 2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-13, -12, 15}, pros::MotorGearset::blue);
pros::Motor leftIntakeBottom(10, pros::MotorGearset::blue);
pros::Motor leftIntakeTop(7, pros::MotorGearset::green);
pros::Motor rightIntakeBottom(9, pros::MotorGearset::green);
pros::Optical color_sensor(20);

pros::Distance park_distance(8); //base distance is around 200 mm. With ball is around 50-120

pros::adi::DigitalOut intake('A', false); 
pros::adi::DigitalOut doinker('F', false); 
pros::adi::DigitalOut expansion('D', false);
pros::adi::DigitalOut wings('E', true);
lemlib::ExpoDriveCurve throttle_curve(3,
                                     6,
                                     1.019 
);

lemlib::ExpoDriveCurve steer_curve(3,
                                  6,
                                  1.014
);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              10,
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2
);

pros::Imu imu(18);
pros::Rotation horizontal_encoder(4);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.2);
lemlib::TrackingWheel horizontal2_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -0.2);

lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu
);

lemlib::ControllerSettings lateral_controller(10,
                                              0, 
                                              3,
                                              3,
                                              1,
                                              100,
                                              3,
                                              500,
                                              20
);

lemlib::ControllerSettings angular_controller(2.8,
                                              0.03,
                                              25,
                                              30,
                                              1,
                                              100,
                                              3,
                                              500,
                                              0
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
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(-127); //top
}

void stopAllMotors() {
    leftIntakeBottom.move(0); //bottm
    leftIntakeTop.move(0); //middle
    rightIntakeBottom.move(0); //top
}

void skillsAutonomous() {
    // Add your skills autonomous routine here
    chassis.setPose(0, 0, 0);
    // Skills routine implementation...
}

void left_auton() {
}


void right_auton() {
    chassis.setPose(0, 0, 0);
    intake.set_value(true);
    expansion.set_value(true);
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    //rightIntakeBottom.move(-127); //top
    //first three balls
    chassis.moveToPoint(0, 15.5, 2000, {.maxSpeed = 120}); 
    chassis.turnToHeading(20, 1000);
    pros::delay(1000);
    chassis.moveToPoint(5, 25, 4000, {.maxSpeed = 23}); 
    pros::delay(200);
    chassis.turnToHeading(124, 4000);
    pros::delay(1000);
    
    chassis.moveToPoint(21, 12, 4000, {.maxSpeed = 70}); 
    pros::delay(1000);
    chassis.turnToHeading(180, 4000);
    doinker.set_value(false);
    chassis.moveToPoint(21, 3, 4000, {.maxSpeed = 100}); 
    pros::delay(1200);
    chassis.moveToPoint(21, 23, 4000, {.forwards = false, .maxSpeed = 100}); 
    intake.set_value(false);
    pros::delay(500);
    //doinker.set_value(true);
    pros::delay(2000);
    rightIntakeBottom.move(-127); //top
    pros::delay(5000);
    rightIntakeBottom.move(0); //top
    
    // //load next three balls
    // chassis.moveToPoint(30, -10, 4000, {.maxSpeed = 120});
    // pros::delay(1000);
    // chassis.moveToPoint(30, 20, 4000, {.forwards = false, .maxSpeed = 120});

}

void testing_auton() {
    chassis.setPose(0, 0, 0);
    intake.set_value(false);
    expansion.set_value(true);
    chassis.turnToHeading(90, 100000);
}



void soloAWP() {
    //start facing the right side wall, bot is perpendicular with around half of the bot sticking out of park zone
    chassis.setPose(0, 0, 0);
    wings.set_value(false);
    //move in front of match loader
    chassis.moveToPoint(0, 26.3, 3000, {.maxSpeed = 120});
    pros::delay(200);
    chassis.turnToHeading(104, 1000); //face match loader
    pros::delay(500);
    //go to match loader and intake
    scoreHigh();
    doinker.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(13.5, 26.1, 1000);
    pros::delay(1000);
    // //go to tube and score
    stopAllMotors();
    wings.set_value(true);
    chassis.moveToPoint(-15, 30, 3000, {.forwards = false, .maxSpeed = 100});
    pros::delay(800);
    doinker.set_value(false);
    scoreHigh();
    pros::delay(2000);
    // //get out, and line up
    chassis.moveToPoint(-8, 30, 3000);
    chassis.turnToHeading(-120, 1000); //face balls
    wings.set_value(false);
    // intake.set_value(false);
    // pros::delay(500);
    chassis.moveToPoint(-20, 20, 4000, {.maxSpeed = 100});
    chassis.moveToPoint(-28, 14, 4000, {.maxSpeed = 50});
    // //straight line get balls
    pros::delay(1500);
    chassis.turnToHeading(180, 1000); //face balls
    chassis.moveToPoint(-40, -24, 4000, {.maxSpeed = 50});
    pros::delay(1000);
    chassis.moveToPoint(-47, -9.6, 4000, {.forwards = false, .maxSpeed = 100});
    rightIntakeBottom.move(127);
    pros::delay(1000);
    // //score balls middle top goal
    // chassis.turnToHeading(225, 1000); //face goal
    // chassis.moveToPoint(-25, -25, 4000, {.forwards = false, .maxSpeed = 100});
    // scoreMiddleHigh();
    // pros::delay(1000);
    // // match loader
    // scoreHigh();
    // chassis.moveToPoint(-8, -55, 4000, {.maxSpeed = 100});
    // //go into it
    // doinker.set_value(true);
    // chassis.moveToPoint(10, -55, 4000, {.maxSpeed = 100});
    // pros::delay(1000);
    // //back out and go to tube
    // chassis.moveToPoint(-16, -55, 4000, {.forwards = false, .maxSpeed = 100});
    // intake.set_value(true);
    // pros::delay(1000);
    // stopAllMotors();
}

void autonomous() {
    // Run the autonomous routine selected on the brain screen
    // Selection is saved to SD card and persists across reboots
    soloAWP();
    // testing_auton();
}


void opcontrol() {
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

        chassis.arcade(leftY, rightX);
        prevIntakePistonButton = intakePistonButton; // update edge detector
        prevDoinkerButton = doinkerButton; // update edge detector for doinker
        prevExpansionButton = expansionButton; // update edge detector for expansion
        prevWingButton = wingButton; // update edge detector for wing
        pros::delay(5);
    }
}

