#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h" 
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

pros::Distance park_distance(10);

pros::adi::DigitalOut intake('A', true); 
pros::adi::DigitalOut doinker('B', true); 
pros::adi::DigitalOut expansion('C', true);
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



void good_auton() {
    
}
void simple_auton() {
    
}

rd::Selector selector("auton_selector", {
    {"Simple auton", simple_auton},
    {"Good auton", good_auton},
});

void on_center_button() {
	// Open the autonomous selector when center button is pressed
	selector.focus();
}

void initialize() { 
    pros::delay(100);
    // Don't initialize PROS LCD - it conflicts with RoboDash
    pros::lcd::initialize();
    
    // Add callback to get notified when autonomous is selected

	chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::c::motor_set_brake_mode(1, pros::E_MOTOR_BRAKE_HOLD);
    
    // Configure vision sensor signatures for color sorting


}

void disabled() {}

void competition_initialize() {}


void skillsAutonomous() {

}

void left_auton() {
    chassis.setPose(0, 0, 0);
    intake.set_value(false);
    expansion.set_value(true);
}

void right_auton() {
    chassis.setPose(0, 0, 0);
    intake.set_value(false);
    expansion.set_value(true);
}

void solo_awp() {
    chassis.setPose(0, 0, 0);
    intake.set_value(true);
    expansion.set_value(true);
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(-127); //top
    //first three balls
    chassis.moveToPoint(0, 7, 4000, {.maxSpeed = 120}); 
    chassis.moveToPoint(11, 22, 4000, {.maxSpeed = 50}); 
    pros::delay(500);
    chassis.moveToPoint(12, 15, 4000, {.forwards = false, .maxSpeed = 120}); 
    pros::delay(500);
    chassis.turnToHeading(90, 4000);
    // set up to match load
    pros::delay(500);
    chassis.moveToPoint(24, 5, 4000, {.maxSpeed = 120}); 
    pros::delay(500);
    chassis.turnToHeading(172, 1000);
    doinker.set_value(false);
    chassis.moveToPoint(25.5, -7, 4000, {.maxSpeed = 120}); 
    pros::delay(1000);
    chassis.moveToPoint(25.5, 20, 4000, {.forwards = false, .maxSpeed = 120});
    pros::delay(200);
    intake.set_value(false);
    
    // pros::delay(2000);
    // chassis.turnToHeading(200, 4000);
    // pros::delay(2000);
    // doinker.set_value(false);
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
    //left_auton();
    solo_awp();
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
    // Persistent state for color sort toggle
    static bool prevColorSortButton = true;   // previous loop state of color sort button (X)
    
    // Color sort config: 1 = red, 2 = blue
    int colorToReject = 1; 
    static bool colorSortMode = false; // false = off, true = on
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeFour = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool intakeFive = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        bool colorSortButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        bool doinkerButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool expansionButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool intakeLiftState = false; // unused currently
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Get object count to see if anything is detected

        double hue = color_sensor.get_hue();

        
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

        if (colorSortMode) {
            if (hue >= 330 || hue <= 30) { // Red hue range
                if (colorToReject == 1) {
                    leftIntakeBottom.move(-127); //bottm
                    leftIntakeTop.move(-127); //middle
                    rightIntakeBottom.move(127); //top
                    pros::delay(150);
                }
            } else if (hue >= 180 && hue <= 250) { // Blue hue range
                if (colorToReject == 2) {
                    leftIntakeBottom.move(-127); //bottm
                    leftIntakeTop.move(-127); //middle
                    rightIntakeBottom.move(127); //top
                    pros::delay(150);
                }
            }
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
        
        if (colorSortButton) {
            // Edge-triggered toggle for color sorting
            if (!prevColorSortButton) {
                colorSortMode = !colorSortMode;
                pros::delay(100);
            }
        }

        chassis.arcade(leftY, rightX);
        prevIntakeFour = intakeFour; // update edge detector
        prevDoinkerButton = doinkerButton; // update edge detector for doinker
        prevExpansionButton = expansionButton; // update edge detector for expansion
        prevColorSortButton = colorSortButton; // update edge detector for color sort
        pros::delay(5);
    }
}

