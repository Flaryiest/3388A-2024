#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-15,-6, -7}, pros::MotorGearset::red);
pros::MotorGroup right_motors({3, 5, 4}, pros::MotorGearset::red);
pros::Motor intake(-18, pros::MotorGearset::red);
pros::adi::DigitalOut clamp('F');
pros::adi::DigitalOut wing('E');
pros::adi::DigitalIn wingPower('E');

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              10,
                              lemlib::Omniwheel::NEW_275,
                              360,
                              2
);

pros::Imu imu(9);

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


lemlib::ControllerSettings angular_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
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
	pros::lcd::initialize();
	chassis.calibrate();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000);
}

void opcontrol() {
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		bool intakeButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool intakeReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		bool clampButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool clampReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

		if (intakeButton) {
			intake.move(127);
		}
		else if (intakeReverseButton) {
			intake.move(-127);
		}
		else {
			intake.move(0);
		}

		if (clampButton) {
			clamp.set_value(HIGH);
		}
		else if (clampReverseButton) {
			clamp.set_value(LOW);
		}

        if (wingButton) {
            if (wingPower.get_value() > 0) {
                wing.set_value(HIGH);
            }
            else {
                wing.set_value(LOW);
            }

        }
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(leftY, rightX);

        pros::delay(25);
		




		
    }
}