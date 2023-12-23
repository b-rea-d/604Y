#include "main.h"

void powerDrive(double forwardPower, double turningPower){
	Lf = forwardPower *.98 + turningPower;
	Lf2 = forwardPower *.98 + turningPower;
	Lb = forwardPower *.98 + turningPower;
	Rf = forwardPower  - turningPower;
	Rf2 = forwardPower - turningPower;
	Rb = forwardPower  - turningPower;
}

void resetDriveEncoders(){
	Lb.tare_position(); 
}
void moveForward(double targetDistance){
	
	resetDriveEncoders();
 	while(Lb.get_position() <= targetDistance){
		powerDrive(100,0);
	}
	powerDrive (0,0);
}
void moveLeft(double targetDistance){
	resetDriveEncoders();
	while(-Lb.get_position() <= targetDistance){
		powerDrive(0,-50);
	}
	powerDrive(0,0);
}

void moveRight(double targetDistance){
	resetDriveEncoders();
	while(Lb.get_position() <= targetDistance){
		powerDrive(0,50);
	}
	powerDrive(0,0);
}

void moveBackward(double targetDistance){
	resetDriveEncoders();
	while(Lb.get_position() >= -targetDistance){
		powerDrive(-100,0);
	}                                                                                                                                                                                                                                              
	powerDrive(0,0);
}

void wings(bool wingPosition){
	
		Wing.set_value(wingPosition);
} 

void intakeOn(int power){
	Intake1 = power;
} 
void intakeOff(){
	Intake1 = 0;
}


void moveF (double targetDistance, double Kp){
	Lb.tare_position();
	double error = targetDistance - Lb.get_position();
	double power;

	while (fabs(error) >= 50){
		pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", error);

	error = targetDistance - Lb.get_position();
	power = Kp * error; 
	if(power>= 90){
		powerDrive (90,0);
	}
	else{
		powerDrive (power,0);
	}

	} 
	powerDrive (0,0);
}

void moveB (double targetDistance, double Kp){
	Lb.tare_position();
	double error = targetDistance + Lb.get_position();
	double power;

	while (abs(error) > 5){
	error = targetDistance + Lb.get_position();
	power = Kp * error; 
	powerDrive (-power,0);
	} 
	powerDrive (0,0);
}

void moveR (double targetDistance, double Kp){
	Lb.tare_position();
	double error = targetDistance - Lb.get_position();
	double power;

	while (abs(error) > 0){
	error = targetDistance - Lb.get_position();
	power = Kp * error; 
	powerDrive (0, power);
	} 
	powerDrive (0,0);
}
void turn(double degree, double kp){
    imu.tare_rotation(); 
    double error = degree - imu.get_rotation();
    double power; 
    while(abs(error) >= 5){
        error = degree - imu.get_rotation(); 
        power = error*kp;
        powerDrive(0, power);
    }
    powerDrive(0,0);
}
/*
void (double)

	double error = targetDistance - Lb.get_position();
	double power;

	while (abs(error) > 0){
	error = targetDistance - Lb.get_position();
	power = Kp * error; 
	powerDrive (0, power);
	} 
	powerDrive (0,0);
*/
