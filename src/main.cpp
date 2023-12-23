#include "main.h"

Motor Lf(-13);
Motor Lf2(2);
Motor Lb (-12);
Motor Rf(18);
Motor Rf2 (-3);
Motor Rb (19);

Motor Cata (17);

Motor Intake1 (14);
Imu imu (6);

ADIDigitalOut Wing (4);


ADIDigitalOut Hang (3);


Controller master(E_CONTROLLER_MASTER);




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.reset();
	delay(1000);
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, follwing either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	
	//turn(-90, 1.2);
	
		///THIS IS OffENSIVE AUTO 


/*Intake1 = 90;
	moveF (3100, 1.76);
	delay(1000);
	turn (95, 1.8);
	delay(1000);
	Intake1 = -127;
	Wing.set_value(1);
	moveF (1300, 1.76);
	Intake1 = 127;
	Intake1 = 0;
	delay (1000);
	moveF(-700, 1.76);
	delay (1000);
	moveF (1400, 1.76);
	delay (1000);
	moveF (-700,1.76);
*/


	/*
	moveB (200, .14);
	moveF (220, .16);
	Wings (0);

	moveR ();
	moveF ();
	moveB ();
	moveL ();
	moveF ();
	moveR ();
	moveF ();
	Wings (1);
	moveL ();
	Wings (0);
	moveF ();
	moveL ();
	MoveF ();
	// could require more back and forths
	wings (0);
	moveR ();
	moveF ();
	Intake1 (-);
	moveR(-);
	moveF ();
	moveR (-);
	moveF ();
	Intake1 (-);
*/

		////THIS IS THE RIGHT SIDE AUTO 
//

	cata = 85;
	cata = 0;
	moveF (1500,.9);
	delay (750);
	moveF (-450, .9);
	delay (750);
	wings (1);
	turn (-100, 1.6);
	delay (750);
	turn (-ata70, 1.7);
	delay (1000);
	wings (0);
	delay (750);
	delay (750);
	moveF (500, .9);
	delay (750);
	turn (-40, 1.6);
	moveF (1400, .9);
	

	

}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	bool wingPosition = false;
	bool HangPosition = false; 
	while (true) {
	int Forward = master.get_analog(ANALOG_RIGHT_Y);
	int Turn = master.get_analog(ANALOG_RIGHT_X) * .80;
	powerDrive(Forward,Turn);

	if (master.get_digital(DIGITAL_B)){
		Cata = 85;
	} 

	else{
		Cata = 0;
	}
	
	if (master.get_digital(DIGITAL_R1)){
		Intake1 = -127;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	}
	else if (master.get_digital(DIGITAL_R2)){
		Intake1 = 127;

	}
	else{
		Intake1 = 0; 

	
}
	if (master.get_digital_new_press(DIGITAL_L2)){
		wingPosition = !wingPosition;
		Wing.set_value(wingPosition);

	}
	if (master.get_digital_new_press(DIGITAL_L1)){
		HangPosition = !HangPosition;
		Hang.set_value(HangPosition);

	}

	
}

}