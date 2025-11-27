#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h" 
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "pros/vision.h" // IWYU pragma: keep

#include "robodash/api.h" // GUI

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-20, -19, -18}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({10, 9, 8}, pros::MotorGearset::blue);
pros::Motor leftIntakeBottom(15, pros::MotorGearset::blue);
pros::Motor leftIntakeTop(16, pros::MotorGearset::green);
pros::Motor rightIntakeBottom(17, pros::MotorGearset::green);
pros::Optical color_sensor(7);

pros::Distance park_distance(3);

pros::adi::DigitalOut intake('A', true); 
pros::adi::DigitalOut doinker('B', true); 
pros::adi::DigitalOut expansion('C', true);
pros::adi::DigitalOut wings('D', false);
lemlib::ExpoDriveCurve throttle_curve(3,
                                     6,
                                     1.019 
);

lemlib::ExpoDriveCurve steer_curve(3,
                                  6,
                                  1.012
);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              10,
                              lemlib::Omniwheel::NEW_275,
                              360,
                              2
);

pros::Imu imu(14);
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

lemlib::ControllerSettings angular_controller(2.3,
                                              0.003,
                                              15,
                                              3,
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
    
    // Show the autonomous selector on the brain screen BEFORE calibration
    // TAP an auton on the screen to select it
    // Then PRESS UP on controller to save the selection
    selector.focus();
    
    // Wait for user to save with UP button
    controller.print(2, 0, "Press UP to save");
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            auto current = selector.get_auton();
            if (current.has_value()) {
                // Try different file paths
                const char* paths[] = {"/usd/auton.txt", "/usd/auton_selector", "auton.txt"};
                bool saved = false;
                
                for (int i = 0; i < 3 && !saved; i++) {
                    FILE* save_file = fopen(paths[i], "w");
                    if (save_file != nullptr) {
                        fprintf(save_file, "%s", current->name.c_str());
                        fflush(save_file);
                        fclose(save_file);
                        controller.clear_line(2);
                        controller.print(2, 0, "SAVED! (path %d)", i);
                        saved = true;
                        pros::delay(1000);
                    }
                }
                
                if (!saved) {
                    controller.print(2, 0, "All paths failed");
                }
            } else {
                controller.print(2, 0, "No auton selected");
            }
            break;
        }
        pros::delay(20);
    }
    
    // Calibrate chassis (this runs in background)
	chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::c::motor_set_brake_mode(1, pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}


void skillsAutonomous() {
    // Add your skills autonomous routine here
    chassis.setPose(0, 0, 0);
    // Skills routine implementation...
}

void left_auton() {
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
    chassis.moveToPoint(-5, 25, 4000, {.maxSpeed = 23}); 
    pros::delay(200);
    chassis.turnToHeading(124, 4000);
    pros::delay(1000);
    
    chassis.moveToPoint(-21, 12, 4000, {.maxSpeed = 70}); 
    pros::delay(1000);
    chassis.turnToHeading(180, 4000);
    doinker.set_value(false);
    chassis.moveToPoint(-21, 3, 4000, {.maxSpeed = 100}); 
    pros::delay(1200);
    chassis.moveToPoint(-21, 23, 4000, {.forwards = false, .maxSpeed = 100}); 
    intake.set_value(false);
    pros::delay(500);
    //doinker.set_value(true);
    pros::delay(2000);
    rightIntakeBottom.move(-127); //top
    pros::delay(5000);
    rightIntakeBottom.move(0); //top
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

void autonomous() {
    // Run the autonomous routine selected on the brain screen
    // Selection is saved to SD card and persists across reboots
    selector.run_auton();
}


void opcontrol() {
    // Persistent state for intake piston toggle
    static bool intakePistonState = true;      // false = retracted, true = extended
    static bool prevIntakeFour = true;         // previous loop state of R2 button
    // Persistent state for doinker toggle
    static bool doinkerState = true;           // false = retracted, true = extended
    static bool prevDoinkerButton = true;      // previous loop state of doinker button (A)
    // Persistent state for expansion toggle
    static bool expansionState = true;        // false = retracted, true = extended (deployed)
    static bool prevExpansionButton = true;   // previous loop state of expansion button (B)
    // Persistent state for wing toggle
    static bool wingState = false;            // false = retracted, true = extended
    static bool prevWingButton = true;        // previous loop state of wing button (X)
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeFour = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool intakeFive = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        bool doinkerButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool expansionButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool intakeLiftState = false; // unused currently
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (intakeOne) {
            leftIntakeBottom.move(-127); //bottm
            leftIntakeTop.move(-127); //middle
            rightIntakeBottom.move(-127); //top

        } else if (intakeTwo) {
            leftIntakeBottom.move(-127); //bottm
            leftIntakeTop.move(-127); //middle
            rightIntakeBottom.move(127); //top


        } else if (intakeThree) {
            leftIntakeBottom.move(127);
            leftIntakeTop.move(127);
            rightIntakeBottom.move(127);
        } else if (intakeFive) {
            leftIntakeBottom.move(0);
            leftIntakeTop.move(127);
            rightIntakeBottom.move(-127);
        }

        else {
             leftIntakeBottom.move(0);
            leftIntakeTop.move(0);
            rightIntakeBottom.move(0);
        }

        if (intakeFour) {
            // Edge-triggered toggle: only toggle when button transitions from not pressed to pressed
            if (!prevIntakeFour) {
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
        prevIntakeFour = intakeFour; // update edge detector
        prevDoinkerButton = doinkerButton; // update edge detector for doinker
        prevExpansionButton = expansionButton; // update edge detector for expansion
        prevWingButton = wingButton; // update edge detector for wing
        pros::delay(5);
    }
}

