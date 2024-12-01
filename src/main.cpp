#include "main.h"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-15,-6, -7}, pros::MotorGearset::red);
pros::MotorGroup right_motors({3, 5, 4}, pros::MotorGearset::red);
pros::Motor intake(-18, pros::MotorGearset::green);
pros::adi::DigitalOut clamp('F');
pros::adi::DigitalOut wing('D', false);
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.009 // expo curve gain
);

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

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0.0015, // integral gain (kI)
                                              10, // derivative gain (kD)
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

void redSideLeft() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -19, 2000, {.forwards = false, .maxSpeed = 100});
    pros::delay(900); 
    clamp.set_value(LOW);
    pros::delay(500);
    intake.move(127);
    pros::delay(1400);
    clamp.set_value(HIGH);
}

void redSideRight() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -14.0, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.moveToPoint(0, -33.0, 2000, {.forwards = false, .maxSpeed = 35});
    pros::delay(3000); 
    clamp.set_value(LOW);
    pros::delay(500);
    intake.move(127);
    pros::delay(1400);
}

void blueSideLeft() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -19, 2000, {.forwards = false, .maxSpeed = 100});
    pros::delay(900); 
    clamp.set_value(LOW);
    pros::delay(500);
    intake.move(127);
    pros::delay(1400);
    clamp.set_value(HIGH);
}

void blueSideRight() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -19, 2000, {.forwards = false, .maxSpeed = 100});
    pros::delay(900); 
    clamp.set_value(LOW);
    pros::delay(500);
    intake.move(127);
    pros::delay(1400);
    clamp.set_value(HIGH);
}






void autonomous() {
    redSideRight();
}



void opcontrol() {
    bool wingState = false;
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
            wingState = !wingState;
            wing.set_value(wingState ? HIGH : LOW);
            pros::delay(200);
}


        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(leftY, rightX);

        pros::delay(25);
		




		
    }
}