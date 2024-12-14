#include "main.h"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/vision.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-6, -20, -17}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({12, 16, 19}, pros::MotorGearset::blue);
pros::Motor intake(-4, pros::MotorGearset::green);
pros::adi::DigitalOut clamp('A', false);
pros::adi::DigitalOut wing('B', false);
pros::Vision wallstake_sensor (17);
pros::vision_signature_s_t red_ring_sig = pros::Vision::signature_from_utility(1, 255, 307, 281, -355, -243, -299, 3.0, 0);
pros::vision_signature_s_t blue_ring_sig = pros::Vision::signature_from_utility(2, -3693, -2747, -3220, 1721, 3247, 2484, 3.0, 0);
lemlib::ExpoDriveCurve throttle_curve(3,
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

pros::Imu imu(3);

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
	pros::lcd::initialize();
	chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::lcd::print(0, "Testing");
}

void disabled() {}

void competition_initialize() {}

void rightAutonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -18.0, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.moveToPoint(0, -26.0, 2000, {.forwards = false, .maxSpeed = 15});
    pros::delay(1000); 
    clamp.set_value(LOW);
    pros::delay(500);
    intake.move(127);
    pros::delay(1200);
    chassis.moveToPoint(0, -33, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-40, -33, 2000, {.forwards = true, .maxSpeed = 40});
    pros::delay(3000);
    intake.move(0);
}
   
    // clamp.set_value(LOW);
    // chassis.moveToPoint(0, -23, 2000, {.forwards = false, .maxSpeed = 80});
    // clamp.set_value(LOW);
    // pros::delay(500);
    // chassis.moveToPoint(5, -40, 2000, {.forwards = true, .maxSpeed = 100});
    // intake.move(127);
    // pros::delay(1000);
    // clamp.set_value(HIGH);
    // pros::delay(5000);
    // clamp.set_value(LOW);
    
    //chassis.moveToPoint(30, -40, 2000, {.forwards = true, .maxSpeed = 100});
    // chassis.moveToPoint(-5, 42, 2000, {.forwards = false, .maxSpeed = 80 });
    // chassis.moveToPoint(-17, 30, 2000, {.maxSpeed = 80});
    // chassis.moveToPoint(0, 5, 2000, {.maxSpeed = 127});
    // chassis.turnToHeading(90, 1000);
    // clamp.set_value(HIGH);
    // chassis.moveToPoint(20, 5, 2000, {.maxSpeed = 20});
    // intake.move(0);
    // chassis.turnToHeading(320, 1000);
    // chassis.moveToPoint(30, 20, 2000);
    // clamp.set_value(LOW);
    // intake.move(127);
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(50, 20, 2000, {.maxSpeed = 20});
    // chassis.moveToPoint(30, 20, 2000, {.forwards = false, .maxSpeed = 127});
    // chassis.moveToPoint(30, 45, 2000, {.forwards = true, .maxSpeed = 127});
    // intake.move(0);

void leftAutonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.turnToHeading(179, 1000);
    chassis.moveToPoint(0, 23, 2000, {.forwards = false, .maxSpeed = 127});
    clamp.set_value(LOW);
    intake.move(127);
    chassis.turnToHeading(320, 1000);
    pros::delay(500);
    clamp.set_value(HIGH);
    chassis.moveToPoint(20, 44, 2000, {.forwards = false, .maxSpeed = 100});
    clamp.set_value(LOW);
    chassis.moveToPoint(0, 23, 2000, {.maxSpeed = 127});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(30, 23, 2000, {.forwards = true, .maxSpeed = 127});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-20, 23, 1000);
    intake.move(0);
}

void skillsAutonomous() {
    int lineOneY = -15;
    int lineTwoY = -40;
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, lineOneY, 2000, {.forwards = false, .maxSpeed = 127});
    clamp.set_value(LOW);
    pros::delay(200);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(50, lineOneY, 2000, {.forwards = true, .maxSpeed = 100});
    pros::delay(100);
    chassis.turnToHeading(180, 1000);
    intake.move(127);
    chassis.moveToPoint(25, lineTwoY, 1000, {.forwards = true, .maxSpeed = 127});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(40, lineTwoY, 1000);
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(40, 5, 1000);
    chassis.moveToPoint(40, lineOneY, 1000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(50, lineOneY, 1000, {.forwards = true});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(50, -20, 1000);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, 0, 1000, {.forwards = true, .maxSpeed = 127});
    
  
}




void autonomous() {
    rightAutonomous();
}

void opcontrol() {
    bool wingState = false;
    bool clampState = false;
    bool ladyBrownState = false;


    while (true) {
        bool intakeButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        bool clampButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

        bool ladyBrownButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        bool ladyBrownReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool ladyBrownLoadButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool ladyBrownHoverButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        wallstake_sensor.set_signature(1, &red_ring_sig);
        wallstake_sensor.set_signature(2, &blue_ring_sig);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

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
            pros::delay(200);
        }

        if (wingButton) {
            wingState = !wingState;
            wing.set_value(wingState ? HIGH : LOW);
            pros::delay(200);
        }

        chassis.arcade(leftY, rightX);
        pros::lcd::print(1, "SWAG");
        pros::delay(25);
    }
}
