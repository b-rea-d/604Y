#include "main.h"

void powerDrive(double forwardPower, double turningPower){
	Lf = forwardPower *.98 + turningPower;
	Lf2 = forwardPower *.98 + turningPower;
	Lb = forwardPower *.98 + turningPower;
	Rf = forwardPower  - turningPower;
	Rf2 = forwardPower - turningPower;
	Rb = forwardPower  - turningPower;
}

void powerDriveSide(double leftSide, double rightSide){
	Lf = -leftSide; 
	Lf2 = -leftSide; 
	Lb = -leftSide;
	Rf = -rightSide; 
	Rf2 = -rightSide;
	Rb = -rightSide;
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


void moveF (double targetDistance, double Kp, double limit, int time){
	startTimer(1);
	Lb.tare_position();
	double error = targetDistance - Lb.get_position();
	double power;

	while (fabs(error) >= 50 && time >= getTime(1)){
		pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", error);

	error = targetDistance - Lb.get_position();
	power = Kp * error; 
	if(power>= limit){
		powerDrive (limit,0);
	}
	else{
		powerDrive (power,0);
	}

	} 
	powerDrive (0,0);
}

void swingleft(double targetDistance, double Kp, double limit,double time){ 
	startTimer(1);
	Rf.tare_position();
	double error = targetDistance - Rf.get_position();
	double power;

	while (fabs(error) >= 50 && time >= getTime(1)){
		pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", error);

	error = targetDistance - Rf.get_position();
	power = Kp * error; 
		if(abs(power)>= limit){
		powerDriveSide((limit*.3), limit);
	}
	else{

		powerDriveSide((power*.3), power);
	}

	

	} 
	powerDrive (0,0);
}
void circle_turn_right(double targetDistance, double Kp, double limit,double time){ 
	startTimer(1);
	Lf.tare_position();
	double error = targetDistance - Lf.get_position();
	double power;

	while (fabs(error) >= 50 && time >= getTime(1)){
		pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", error);

	error = targetDistance - Lf.get_position();
	power = Kp * error; 
		if(abs(power)>= limit){
		powerDriveSide(limit,(limit*.41));
	}
	else{

		powerDriveSide(power,(power*.41));
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
//inches to ticks

double inchestoticks (double inches){ 
	double internal = (double)300, //tickets per revolution
	external = (double)4/50, //gear ratio
	
		diameter = 2.75, //diamter of wheels
	pi =3.141; // whaterver pi is

	return ((inches/pi/diameter/external*internal));
}

void PIDdrive (double inches, double kP, double kI, double kD, double limit, int Time){
	startTimer(1);
	
	int power, intergral, past_error;
	double derivative, error, target;

//declare variables for turning
int r_target,r_power;
double r_kP= 0.2, r_error;


//calculating error distance 
Rf.tare_position();
target = inchestoticks(inches);
error = target-Rf.get_position();

//set error for rotation
r_target=imu.get_rotation();


//ensure function runs until robot position in within 2 encoder ticks of the target
// || means or cuz people are weird
//timer thing
while (((fabs(error) > 2)||(fabs(r_error) >= 5 ))&& (Time > getTime(1))) {
	pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", error);
	//driving forward
	//calculating the derivative
	past_error = error;
	error = target - Rf.get_position();
	derivative = error - past_error;
	//onlu using intergral if within 10 encoder ticks  of target
	if(fabs(error)<10){
		intergral += error;
	}
	past_error = error;
	//calculate motor power and assigning power to all motors
	power = error*kP +intergral*kI + derivative*kD;

	// //rotation thingy
	// r_error = r_target - imu.get_rotation();
	// r_power = r_error*r_kP;
	// if(fabs(error)<10){
	// 	intergral += error;
	// }
	// powerDrive(power,r_power);

//limit
	error = target - Rf.get_position();
	if(power>= limit){
		powerDrive (limit,0);
	}
	else{
		powerDrive (power,0);
	}

	} 


	powerDrive(0,0);
	}



void PIDturn (double degree, double kP, double kI, double kD, double limit, int Time){
	startTimer(1);
	imu.tare_rotation();
	//declare variables for turning
	int r_target,r_power, r_intergral, r_past_error;
	double r_error, r_derivative;
	r_target=degree - imu.get_rotation();
	r_error = r_target - imu.get_rotation();

	//set error for rotation
	r_target=degree - imu.get_rotation();
	pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", r_error);
	while (((fabs(r_error) >= 5 ))) {
	pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", r_error);
	//(Time > getTime(1))
	//rotation thingy
	r_error = degree - imu.get_rotation();
	r_power = r_error*kP+r_intergral*kI + r_derivative*kD; 
	if(fabs(r_error)<10){
		r_intergral += r_error;
	}
	r_error = r_derivative;
	//limit 
	if(r_power>= limit){
		powerDrive(0,limit);
	}
	else if(r_power<= limit){
		powerDrive(0, limit);
	}
	else{
		powerDrive (0, r_power);
	}


	} 



	powerDrive(0,0);
	}






	




	




