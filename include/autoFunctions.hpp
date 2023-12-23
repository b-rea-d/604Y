#ifndef _AUTO_FUNCTIONS_HPP_
#define _AUTO_FUNCTIONS_HPP_

void powerDrive(double forwardPower, double turningPower);
void resetDriveEncoders();

void moveForward(double targetDistance);
void moveRight(double targetDistance);
void moveBackward(double targetDistance);
void moveLeft(double targetDistance);
void wings(bool wingPosition);
void intakeOn(int power);
void intakeOff(int power);
void moveF (double targetDistance, double Kp);
void moveB (double targetDistance, double Kp);
void moveR (double targetDistance, double Kp);
void moveL (double targetDistance, double Kp);
void turn(double degree, double kp);
#endif