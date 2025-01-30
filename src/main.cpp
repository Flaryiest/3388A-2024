#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-6, -7, -8}, pros::MotorGearset::blue);
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

pros::Imu imu(2);

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
    pros::delay(200);
	if (pros::lcd::initialize()) {
	chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::lcd::set_text(1, "Waiting for checking and initialize!");
    }

}

void disabled() {}

void competition_initialize() {}

void rightAutonomous() {
    chassis.setPose(0, 0, 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -26.0, 2000, {.forwards = false, .maxSpeed = 30, .minSpeed = 10, .earlyExitRange = 0.01});
    pros::delay(4000); 
    clamp.set_value(LOW);
    pros::delay(500);
    intake.move(127);
    pros::delay(1200);
/*     chassis.moveToPoint(0, -29, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-40, -29, 2000, {.forwards = true, .maxSpeed = 40});
    pros::delay(3000);
    intake.move(0); */
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
    chassis.moveToPoint(0, -10, 2000, {.forwards = false, .maxSpeed = 127});
    clamp.set_value(LOW);
    intake.move(127);
    // pros::delay(200);
    // chassis.turnToHeading(270, 1000);
    // chassis.moveToPoint(50, lineOneY, 2000, {.forwards = true, .maxSpeed = 100});
    // pros::delay(100);
    // chassis.turnToHeading(180, 1000);
    // intake.move(127);
    // chassis.moveToPoint(25, lineTwoY, 1000, {.forwards = true, .maxSpeed = 127});
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(40, lineTwoY, 1000);
    // chassis.turnToHeading(0, 1000);
    // chassis.moveToPoint(40, 5, 1000);
    // chassis.moveToPoint(40, lineOneY, 1000);
    // chassis.turnToHeading(90, 1000);
    // chassis.moveToPoint(50, lineOneY, 1000, {.forwards = true});
    // chassis.turnToHeading(180, 1000);
    // chassis.moveToPoint(50, -20, 1000);
    // clamp.set_value(HIGH);
    // chassis.moveToPoint(0, 0, 1000, {.forwards = true, .maxSpeed = 127}); 
}

void kSoloCarryAuton() {
    chassis.setPose(0, 0 , 0);
    clamp.set_value(HIGH);
    chassis.moveToPoint(0, -15, 2000, {.forwards = false});
}




void autonomous() {
    rightAutonomous();
}



void opcontrol() {
    bool wingState = false;
    bool clampState = false;
    bool ladyBrownState = false;
    bool ladyBrownLoad= false;
    bool ladyBrownHover = false;

    while (true) {
        bool intakeButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakeReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        bool clampButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool wingButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);

        bool ladyBrownButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        bool ladyBrownReverseButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool ladyBrownLoadButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        bool ladyBrownHoverButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

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
            pros::delay(200);
        }

        if (wingButton) {
            wingState = !wingState;
            wing.set_value(wingState ? HIGH : LOW);
            pros::delay(200);
        }

        if (ladyBrownLoadButton) {
            ladyBrownLoad = !ladyBrownLoad;
        }
        if (ladyBrownHoverButton) {
            ladyBrownHover = !ladyBrownHover;
        }
        
        if (ladyBrownHover && !(ladyBrownButton) && !(ladyBrownReverseButton)) {
            if (2000 < ladyBrownRotation.get_angle() && ladyBrownRotation.get_angle() < 2500) {
                ladyBrown.move(10);
            }
            else if (ladyBrownRotation.get_angle() > 2500) {
                ladyBrown.move(-50);
            }
            else {
                ladyBrown.move(50);
            }
        }

        else if (ladyBrownLoad && !(ladyBrownButton) && !(ladyBrownReverseButton)) {
            if (1000 < ladyBrownRotation.get_angle() && ladyBrownRotation.get_angle() < 1800) {
                ladyBrown.move(10);
            }
            else if (ladyBrownRotation.get_angle() > 1800) {
                ladyBrown.move(-50);
            }
            else {
                ladyBrown.move(50);
            }
        }

        if (!(ladyBrownButton || ladyBrownReverseButton || ladyBrownLoad || ladyBrownHover)) {
            ladyBrown.move(0);
        }

        if (ladyBrownButton) {
            ladyBrown.move(127);
        }

        if (ladyBrownReverseButton) {
            ladyBrown.move(-127);
        }

        chassis.arcade(leftY, rightX);
        pros::lcd::print(6, "SWAGALICIOUS");
        pros::delay(25);
    }
}
