#include "main.h"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/vision.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-15,-6, -7}, pros::MotorGearset::red);
pros::MotorGroup right_motors({3, 5, 4}, pros::MotorGearset::red);
pros::Motor intake(-18, pros::MotorGearset::green);
pros::Motor ladyBrown(8, pros::MotorGearset::green);
pros::adi::DigitalOut clamp('F');
pros::adi::DigitalOut wing('D', false);
pros::Vision wallstake_sensor (17);
pros::vision_signature_s_t red_ring_sig = pros::Vision::signature_from_utility(1, 255, 307, 281, -355, -243, -299, 3.0, 0);
pros::vision_signature_s_t blue_ring_sig = pros::Vision::signature_from_utility(2, -3693, -2747, -3220, 1721, 3247, 2484, 3.0, 0);
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

void leftAutonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.turnToHeading(120, 1000);
    chassis.moveToPoint(10, -3, 2000, {.forwards = false, .maxSpeed = 100});
    ladyBrown.move(127);
    pros::delay(500);
    ladyBrown.move(-127);
    pros::delay(400);
    ladyBrown.move(0);
    chassis.moveToPoint(0, 25, 2000, {.forwards = false, .maxSpeed = 80});
    clamp.set_value(LOW);
    chassis.moveToPoint(-5, 40, 2000, {.forwards = true, .maxSpeed = 100});
    intake.move(127);
    chassis.moveToPoint(-20, 40, 2000, {.forwards = true, .maxSpeed = 100});
    chassis.moveToPoint(-5, 42, 2000, {.forwards = false, .maxSpeed = 80 });
    chassis.moveToPoint(-17, 30, 2000, {.maxSpeed = 80});
    chassis.moveToPoint(0, 5, 2000, {.maxSpeed = 127});
    chassis.turnToHeading(90, 1000);
    clamp.set_value(HIGH);
    chassis.moveToPoint(20, 5, 2000, {.maxSpeed = 20});
    intake.move(0);
    chassis.turnToHeading(320, 1000);
    chassis.moveToPoint(30, 20, 2000);
    clamp.set_value(LOW);
    intake.move(127);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(50, 20, 2000, {.maxSpeed = 20});
    chassis.moveToPoint(30, 20, 2000, {.forwards = false, .maxSpeed = 127});
    chassis.moveToPoint(30, 45, 2000, {.forwards = true, .maxSpeed 127});


}

void rightAutonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(0, 23, 2000, {.forwards = false, .maxSpeed = 127});
    clamp.set_value(LOW);
    intake.move(127);
    chassis.turnToHeading(320, 1000);
    pros::delay(500);
    clamp.set_value(HIGH);
    /** go for right red ring under blue right after. Then go score that ring on the far mogo */
}



void detectRingColor() {
    wallstake_sensor.set_signature(1, &red_ring_sig);
    wallstake_sensor.set_signature(2, &blue_ring_sig);
    auto red_objects = wallstake_sensor.get_by_sig(0, 1);
    auto blue_objects = wallstake_sensor.get_by_sig(0, 2);
    while (true) {
        pros::screen::erase();
        pros::screen::print(::TEXT_SMALL, 3, "red_objects");=
        pros::delay(1000);
   }
}


void autonomous() {
    leftAutonomous();
}



void opcontrol() {
    bool wingState = false;
    bool clampState = false;
    bool ladyBrownState = false;

    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    bool intakeButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool intakeReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    bool clampButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

    bool ladyBrownButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool ladyBrownLoadButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    

    detectRingColor();

    while (true) {
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
            clampState = !clampState;
            clamp.set_value(clampState ? HIGH : LOW);
            pros::delay(200);
}

        if (wingButton) {
            wingState = !wingState;
            wing.set_value(wingState ? HIGH : LOW);
            pros::delay(200);
}
        if (ladyBrownButton) {
            ladyBrownState = !ladyBrownState;
            wing.set_value(wingState ? 0 : 127);
            pros::delay(200);
}
        if (ladyBrownLoadButton) {
            while true(
                chassis.arcade(leftY, rightX);
                pros::delay(25);
            )
        }

        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(leftY, rightX);
        pros::delay(25);
    




    
    }
}