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
pros::vision_signature_s_t red_ring_sig = pros::Vision::signature_from_utility(1, 237, 61, 74, 0, 0, 0, 10.0, 0);
pros::vision_signature_s_t blue_ring_sig = pros::Vision::signature_from_utility(3, 13, 114, 227, 0, 0, 0, 10.0, 0);

pros::adi::DigitalOut intake('A', true); 
pros::adi::DigitalOut doinker('B', true); 

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

void red_left_auton() {

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
    red_left_auton();
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

    chassis.moveToPoint(-18, 23.5, 2000, {.forwards = true, .maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(500);
    
    doinker.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(-26.5, 14, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 10, .earlyExitRange = 0.01});
    rightIntakeBottom.move(127); //top
    pros::delay(3000);
    chassis.moveToPoint(-27, 20, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(1000);
    chassis.turnToHeading(
    180,
    4000,
    {.maxSpeed = 50}, // will never exceed 50
    false 
    );
    doinker.set_value(true);
    chassis.moveToPoint(-27, 40, 2000, {.forwards = false, .maxSpeed = 120, .minSpeed = 10, .earlyExitRange = 0.01});
    
    
}


void opcontrol() {
    // Persistent state for intake piston toggle
    static bool intakePistonState = false;      // false = retracted, true = extended
    static bool prevIntakeFour = false;         // previous loop state of R2 button
    // Persistent state for doinker toggle
    static bool doinkerState = false;           // false = retracted, true = extended
    static bool prevDoinkerButton = true;      // previous loop state of doinker button (A)
    
    // Color sort config: 1 = red, 3 = blue
    // Change this based on which alliance you're on
    int colorToReject = 1; // Reject blue rings (keep red). Change to 1 to reject red rings (keep blue)
    
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeFour = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool doinkerButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool intakeLiftState = false; // unused currently
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Get the largest object detected by the vision sensor
        wallstake_sensor.set_signature(1, &red_ring_sig);
        wallstake_sensor.set_signature(3, &blue_ring_sig);

        pros::vision_object_s_t obj = wallstake_sensor.get_by_size(0);
        
        // Color sort: if the largest object matches colorToReject, reverse rightIntakeBottom briefly
        if (obj.signature == colorToReject && obj.width > 3 && obj.height > 3) {
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

        } else if (intakeFour) {
            // Edge-triggered toggle: only toggle when button transitions from not pressed to pressed
            if (!prevIntakeFour) {
                intakePistonState = !intakePistonState;
                intake.set_value(intakePistonState);
                pros::delay(100);
            }
        } else if (doinkerButton) {
            // Edge-triggered toggle for doinker
            if (!prevDoinkerButton) {
                doinkerState = !doinkerState;
                doinker.set_value(doinkerState);
                pros::delay(100);
            }
        }

        else {
            leftIntakeBottom.move(0);
            leftIntakeTop.move(0);
            rightIntakeBottom.move(0);
        }

        chassis.arcade(leftY, rightX);
        prevIntakeFour = intakeFour; // update edge detector
        prevDoinkerButton = doinkerButton; // update edge detector for doinker
        pros::delay(5);
    }
}

