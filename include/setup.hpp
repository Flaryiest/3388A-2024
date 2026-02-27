#ifndef _SETUP_HPP_
#define _SETUP_HPP_

#include "lemlib/api.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

// External reference to the cat image
extern "C" {
    extern const lv_img_dsc_t cat_img;
}

// Controller
extern pros::Controller controller;

// Drive motors
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

// Intake motors
extern pros::Motor leftIntakeBottom;
extern pros::Motor leftIntakeTop;
extern pros::Motor rightIntakeBottom;

// Sensors
extern pros::Optical color_sensor;
extern pros::Distance left_distance;
extern pros::Imu imu;
extern pros::Rotation vertical_encoder;
extern pros::Rotation horizontal_encoder;

// Pneumatics
extern pros::adi::DigitalOut hood;
extern pros::adi::DigitalOut doinker;
extern pros::adi::DigitalOut expansion;
extern pros::adi::DigitalOut wings;
extern pros::adi::DigitalOut middleDescore;
extern pros::adi::DigitalOut odomLift;

// LemLib config
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::Drivetrain drivetrain;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;

#endif // _SETUP_HPP_
