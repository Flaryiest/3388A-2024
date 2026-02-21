#include "setup.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-5, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-13, -12, 15}, pros::MotorGearset::blue);
pros::Motor leftIntakeBottom(10, pros::MotorGearset::blue);
pros::Motor leftIntakeTop(7, pros::MotorGearset::green);
pros::Motor rightIntakeBottom(9, pros::MotorGearset::green);
pros::Optical color_sensor(11);

pros::Distance park_distance(8); //base distance is around 200 mm. With ball is around 50-120

pros::adi::DigitalOut hood('A', false); //,maybe true for size
pros::adi::DigitalOut doinker('F', false); 
pros::adi::DigitalOut expansion('D', false);
pros::adi::DigitalOut wings('E', true);
pros::adi::DigitalOut middleDescore('G', false);
pros::adi::DigitalOut odomLift('B', false);
lemlib::ExpoDriveCurve throttle_curve(3,
                                     6,
                                     1.019 
);

lemlib::ExpoDriveCurve steer_curve(3,
                                  6,
                                  1.016
);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              14,
                              lemlib::Omniwheel::NEW_325,
                              450,
                              2
);

pros::Imu imu(6);
pros::Rotation vertical_encoder(-17);
pros::Rotation horizontal_encoder(16);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.5);
    
lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            &horizontal_tracking_wheel,
                            nullptr,
                            &imu
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
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
