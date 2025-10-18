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
pros::Vision wallstake_sensor(5);
pros::vision_signature_s_t red_ring_sig = pros::Vision::signature_from_utility(1, 237, 61, 74, 0, 0, 0, 10.0, 3.0);
pros::vision_signature_s_t blue_ring_sig = pros::Vision::signature_from_utility(3, 13, 114, 227, 0, 0, 0, 10.0, 3.0);

pros::Distance park_distance(10);

pros::adi::DigitalOut intake('A', true); 
pros::adi::DigitalOut doinker('B', true); 
pros::adi::DigitalOut expansion('C', false);
lemlib::ExpoDriveCurve throttle_curve(5,
                                     10,
                                     1.019 
);

lemlib::ExpoDriveCurve steer_curve(3,
                                  10,
                                  1.009
);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              10,
                              lemlib::Omniwheel::NEW_275,
                              360,
                              2
);

pros::Imu imu(5);
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

lemlib::ControllerSettings angular_controller(2,
                                              0.0010,
                                              10,
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

void left_auton() {
    chassis.setPose(0, 0, 0);
    intake.set_value(false);
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(127); //top
    chassis.moveToPoint(0, 19.0, 2000, {.forwards = true, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(1000);
    chassis.moveToPoint(-7, 29.0, 2000, {.forwards = true, .maxSpeed = 40, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(2000);
    //chassis.moveToPoint(-10, 34.0, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 0.01});
    //pros::delay(1500);
    chassis.moveToPoint(-5, 27.5, 2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(250);
    chassis.turnToHeading(
    225,
    4000,
    {.maxSpeed = 50}, // will never exceed 50
    false 
    );
    pros::delay(250);
    chassis.moveToPoint(8.8, 32.7, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(500);
    rightIntakeBottom.move(-127); //top
    pros::delay(2000);
    rightIntakeBottom.move(0);

    chassis.moveToPoint(-19.8, 24, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(500);
    
    doinker.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(-27.3, 12, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 10, .earlyExitRange = 0.01});
    rightIntakeBottom.move(127); //top
    pros::delay(1500);
    chassis.moveToPoint(-26, 20, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(1000);
    chassis.moveToPoint(-18, 30.5, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(500);
    rightIntakeBottom.move(-127);
        leftIntakeBottom.move(127); //bottm
    leftIntakeTop.move(127); //middle
    doinker.set_value(true);
    intake.set_value(true);
    pros::delay(200);
            leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(127);
}

void right_auton() {
    chassis.setPose(0, 0, 0);
    intake.set_value(false);
    leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(127); //top
    chassis.moveToPoint(0, 19, 2000, {.forwards = true, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(1000);
    chassis.moveToPoint(6, 29.0, 2000, {.forwards = true, .maxSpeed = 38, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(2000);
    //chassis.moveToPoint(-10, 34.0, 2000, {.forwards = true, .maxSpeed = 80, .minSpeed = 10, .earlyExitRange = 0.01});
    //pros::delay(1500);
    chassis.moveToPoint(5, 27.5, 2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(250);
    chassis.turnToHeading(
    315,
    4000,
    {.maxSpeed = 50}, // will never exceed 50
    false 
    );
    pros::delay(250);
    chassis.moveToPoint(-8.8, 31, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(500);
    //rightIntakeBottom.move(-127); //top
    pros::delay(1000);
    rightIntakeBottom.move(0);

    chassis.moveToPoint(17.4, 24.5, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(500);
    
    doinker.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(25.2, 11, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 10, .earlyExitRange = 0.01});
    rightIntakeBottom.move(127); //top
    pros::delay(1500);
    chassis.moveToPoint(23.9, 19.5, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(1000);
    chassis.moveToPoint(17.9, 29.8, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(500);
    rightIntakeBottom.move(-127);
        leftIntakeBottom.move(127); //bottm
    leftIntakeTop.move(127); //middle
    doinker.set_value(true);
    intake.set_value(true);
    pros::delay(200);
            leftIntakeBottom.move(-127); //bottm
    leftIntakeTop.move(-127); //middle
    rightIntakeBottom.move(127);  
}

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
    wallstake_sensor.set_signature(1, &red_ring_sig);
    wallstake_sensor.set_signature(3, &blue_ring_sig);

}

void disabled() {}

void competition_initialize() {}


void skillsAutonomous() {

}

void autonomous() {
    left_auton();

}


void opcontrol() {
    // Persistent state for intake piston toggle
    static bool intakePistonState = true;      // false = retracted, true = extended
    static bool prevIntakeFour = true;         // previous loop state of R2 button
    // Persistent state for doinker toggle
    static bool doinkerState = true;           // false = retracted, true = extended
    static bool prevDoinkerButton = true;      // previous loop state of doinker button (A)
    // Persistent state for expansion toggle
    static bool expansionState = false;        // false = retracted, true = extended (deployed)
    static bool prevExpansionButton = false;   // previous loop state of expansion button (B)
    
    // Color sort config: 1 = red, 3 = blue
    // Change this based on which alliance you're on
    int colorToReject = 1; // Reject red rings (keep blue). Change to 3 to reject blue rings (keep red)
    
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeFour = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool doinkerButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool expansionButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool intakeLiftState = false; // unused currently
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Get object count to see if anything is detected
        int32_t obj_count = wallstake_sensor.get_object_count();
        
        // Initialize object with default values
        pros::vision_object_s_t obj;
        obj.signature = 0;
        obj.width = 0;
        obj.height = 0;
        
        // Only get object if count is valid and greater than 0
        if (obj_count > 0 && obj_count < 100) {
            obj = wallstake_sensor.get_by_size(0);
        }
        
        // Debug: Display vision sensor data on brain screen (update every 200ms to avoid spam)
        static uint32_t last_update = 0;
        static int counter = 0;
        if (pros::millis() - last_update > 200) {
            counter++; // Shows screen is updating
            pros::lcd::set_text(0, "Vision Debug [" + std::to_string(counter) + "]");
            pros::lcd::set_text(1, "Objects: " + std::to_string(obj_count));
            pros::lcd::set_text(2, "Sig: " + std::to_string(obj.signature));
            pros::lcd::set_text(3, "W: " + std::to_string(obj.width) + " H: " + std::to_string(obj.height));
            pros::lcd::set_text(4, "Reject: " + std::to_string(colorToReject));
            
            // Check if we have a valid detection
            if (obj_count <= 0 || obj_count >= 100) {
                pros::lcd::set_text(5, "SENSOR ERROR");
            } else if (obj.signature == 255 || obj.signature == 0) {
                pros::lcd::set_text(5, "NO OBJECT DETECTED");
            } else {
                pros::lcd::set_text(5, "Sig " + std::to_string(obj.signature) + " detected!");
            }
            
            last_update = pros::millis();
        }
        
        // Color sort: if the largest object matches colorToReject, reverse rightIntakeBottom briefly
        // Only reject if we have a valid signature and object count
        if (obj_count > 0 && obj_count < 100 && obj.signature == colorToReject && 
            obj.signature != 255 && obj.signature != 0 && obj.width > 3 && obj.height > 3) {
            pros::lcd::set_text(6, "REJECTING!");
            rightIntakeBottom.move(-127);
            pros::delay(200);
            rightIntakeBottom.move(0);
        }
        
        if (intakeOne) {
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



        chassis.arcade(leftY, rightX);
        prevIntakeFour = intakeFour; // update edge detector
        prevDoinkerButton = doinkerButton; // update edge detector for doinker
        prevExpansionButton = expansionButton; // update edge detector for expansion
        pros::delay(5);
    }
}

