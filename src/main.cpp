#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-6, -7, -17}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({20, 19, 18}, pros::MotorGearset::blue);
pros::Motor intake(4, pros::MotorGearset::blue);
pros::Motor ladyBrown(1, pros::MotorGearset::red);
pros::adi::DigitalOut clamp('A', false);
pros::adi::DigitalOut wing('B', false);
pros::Vision wallstake_sensor (2);
pros::vision_signature_s_t red_ring_sig = pros::Vision::signature_from_utility(1, 255, 307, 281, -355, -243, -299, 3.0, 0);
pros::vision_signature_s_t blue_ring_sig = pros::Vision::signature_from_utility(2, -3693, -2747, -3220, 1721, 3247, 2484, 3.0, 0);
pros::Rotation ladyBrownRotation(3);
lemlib::ExpoDriveCurve throttle_curve(20,
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
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladyBrownRotation.set_data_rate(5);
    ladyBrownRotation.set_reversed(true);
    ladyBrownRotation.set_position(0);
    pros::c::motor_set_brake_mode(1, pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::set_text(1, "Waiting for checking and initialize!");

}

void disabled() {}

void competition_initialize() {}

void redSideRightAutonomous() {
    
    chassis.setPose(0, 0, 0);
    ladyBrown.move_relative(-400, -127);
    clamp.set_value(LOW);

    chassis.turnToHeading(-15, 1000);
    chassis.moveToPoint(8, -26.0, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(2000); 
    clamp.set_value(HIGH);
    pros::delay(500);
    intake.move(127);
    pros::delay(1200);
    chassis.moveToPoint(8, -29.0, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 0.01});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(50, -29, 2000, {.forwards = true, .maxSpeed = 80});
    pros::delay(3000);
    intake.move(0);
}

void redSideLeftAutonomous() {
    chassis.setPose(0, 0, 0);
    ladyBrown.move_relative(-400, -127);
    clamp.set_value(LOW);

    chassis.turnToHeading(15, 1000);
    chassis.moveToPoint(-8, -26.0, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(2000); 
    clamp.set_value(HIGH);
    pros::delay(500);
    intake.move(127);
    pros::delay(1200);
    chassis.moveToPoint(-8, -29.0, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 0.01});
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(-50, -29, 2000, {.forwards = true, .maxSpeed = 80});
    pros::delay(3000);
    intake.move(0);
}

void blueSideRightAutonomous() {

}

void blueSideLeftAutonomous() {

}

void skillsAutonomous() {
    int lineOneY = -15;
    int lineTwoY = -40;
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -10, 2000, {.forwards = false, .maxSpeed = 127});
    clamp.set_value(LOW);
    intake.move(127);
}

void autonomous() {
    redSideLeftAutonomous();
}



void opcontrol() {
    bool wingState = false;
    bool clampState = false;
    int ladyBrownStage = 1;
    while (true) {
        bool intakeButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        bool clampButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
        bool wingButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);

        bool ladyBrownButton = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2);

        wallstake_sensor.set_signature(1, &red_ring_sig);
        wallstake_sensor.set_signature(2, &blue_ring_sig);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        pros::vision_object_s_t rtn = wallstake_sensor.get_by_size(0);
        std::cout << "sig: " << rtn.signature;
        
        if (intakeButton) {
            intake.move(127);
        } else if (intakeReverseButton) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        if (clampButton) {
            clampState = !clampState;
            clamp.set_value(clampState ? HIGH : LOW);
            pros::delay(100);
        }

        if (wingButton) {
            wingState = !wingState;
            wing.set_value(wingState ? HIGH : LOW);
            pros::delay(100);
        }

        if (ladyBrownButton) {
            if (ladyBrownStage < 3) {
                ladyBrownStage++;
            }
            else {
                ladyBrownStage = 1;
            }
            pros::delay(50);
        }
        
        if (ladyBrownStage == 1) {
            if (ladyBrownRotation.get_position() > 200 && ladyBrownRotation.get_position() < 1100) {
                ladyBrown.move_velocity(0);
            } 
            else {
                if (ladyBrownRotation.get_angle() < 250 || (ladyBrownRotation.get_angle()  > 35000)) {
                    ladyBrown.move_velocity(-50);
                } 
                else {
                    ladyBrown.move_velocity(50);
                }
            }
        }
        else if (ladyBrownStage == 2) {
            if (ladyBrownRotation.get_angle() > 3000 && ladyBrownRotation.get_angle() < 5000) {
                ladyBrown.move_velocity(0);
            } else {
                if (ladyBrownRotation.get_angle() < 3000) {
                    ladyBrown.move(-80);
                } 
                else {
                    ladyBrown.move(50);
                }
            }
        }
        else if (ladyBrownStage == 3) {
            if (ladyBrownRotation.get_angle() > 12000 && ladyBrownRotation.get_angle() < 14000) {
                ladyBrown.move_velocity(0);
            } else {
                if (ladyBrownRotation.get_angle() < 12000) {
                    ladyBrown.move(-127);
                } 
                else {
                    ladyBrown.move(50);
                }
            }
        }

        chassis.arcade(leftY, rightX);
        pros::lcd::print(6, "SWAGALICIOUS");
        pros::delay(5);
    }
}
