#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "pros/vision.h" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-8, -7, -6}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({19, 18, 17}, pros::MotorGearset::blue);
pros::Motor leftIntakeBottom(9, pros::MotorGearset::green);
pros::Motor leftIntakeTop(10, pros::MotorGearset::green);
pros::Motor rightIntakeBottom(20, pros::MotorGearset::green);

pros::adi::DigitalOut intakeLift('A', false);

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
                            &horizontal_tracking_wheel,
                            &horizontal2_tracking_wheel,
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
                                              0.0015,
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

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() { 
    pros::delay(100);
    pros::lcd::initialize();
	chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::c::motor_set_brake_mode(1, pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::set_text(1, "Waiting for checking and initialize!");

}

void disabled() {}

void competition_initialize() {}


void skillsAutonomous() {

}

void autonomous() {

}

void opcontrol() {
    while (true) {
        bool intakeOne = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeTwo = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeThree = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool intakeFour = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        bool intakeLiftState = false;
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
 
        if (intakeOne) {
            leftIntakeBottom.move(127);
            leftIntakeTop.move(-127);
            rightIntakeBottom.move(127);

        } else if (intakeTwo) {
            leftIntakeBottom.move(127);


        } else if (intakeThree) {
            leftIntakeBottom.move(127);
            leftIntakeTop.move(-127);
            rightIntakeBottom.move(-127);

        } else if (intakeFour) { // done
            leftIntakeBottom.move(127);
            leftIntakeTop.move(127);
            rightIntakeBottom.move(0);
        }

        else {
            leftIntakeBottom.move(0);
            leftIntakeTop.move(0);
            rightIntakeBottom.move(0);
        }

        chassis.arcade(leftY, rightX);
        pros::lcd::print(6, "SWAGALICIOUS");
        pros::delay(5);
    }
}

