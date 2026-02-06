#ifndef _AUTONS_HPP_
#define _AUTONS_HPP_

// Intake/motor helper functions
void scoreHigh();
void scoreMiddleHigh();
void reverseMotors();
void reverseMotorsSlow();
void stopAllMotors();
void reverseMotorsButTop();
void moveDistance(float distance, int timeout, float maxSpeed = 60);

// Autonomous routines
void left_auton();
void right_auton();
void testing_auton();
void lateralTestingAuton();
void skillsAutonomous();
void giveAWP();
void soloAWP();
void newSAWP();
void left_split();

#endif // _AUTONS_HPP_
