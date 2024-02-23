#include "main.h"
#include <algorithm>
#include <cstddef>

Motor Lf(-3, pros::E_MOTOR_GEARSET_06); //number (06) is the cartridge
Motor Lf2(-4, pros::E_MOTOR_GEARSET_06);
Motor Lb (-5, E_MOTOR_GEARSET_06);
Motor Rf(8, E_MOTOR_GEARSET_06);
Motor Rf2 (9, E_MOTOR_GEARSET_06);
Motor Rb (10, E_MOTOR_GEARSET_06);

Motor Cata (-20, E_MOTOR_GEARSET_36);

Motor Intake1 (2, E_MOTOR_GEARSET_06);


ADIDigitalOut Wing (5);

ADIDigitalOut Hang (4);

ADIDigitalOut AWP (6);

Controller master(E_CONTROLLER_MASTER);
//sensor ports
Rotation odomVerticalPod (6, false);
Imu imu (21);
//motor groups
pros::MotorGroup left_side_motors({Lf, Lf2, Lb});
pros::MotorGroup right_side_motors({Rf, Rf2, Rb});

//Lemlib drivetrain  struct

lemlib::Drivetrain drivetrain{
	&left_side_motors, //left drrivetrain motors
	&right_side_motors, //right train motors
	10, //track width in INCHES
	2.75, //wheel diameter
	450, //wheel rpm
	8 //tune this value later : )

};

//define odom pod
lemlib::TrackingWheel vertical_tracking_wheel(&odomVerticalPod, 2.75, 4.3, 1); 

//odom struct
lemlib::OdomSensors sensors{
	&vertical_tracking_wheel,
	nullptr,
	nullptr,
	nullptr,
	&imu

};

// forward/backward PID
lemlib::ControllerSettings lateralController {
    10, // kP
	0, //KI
    4, // kD
	0, //windup
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    250, // largeErrorTimeout
    5// slew rate
};
 
// turning PID
lemlib::ControllerSettings angularController {

    1.6, // kP
	0,
    1, // kD
	0,
    2, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    250, // largeErrorTimeout
    5 // slew rate
};
 
 
// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

ASSET(skills1_txt);
ASSET(skills2_txt);
ASSET(skills3_txt);
ASSET (skills4_txt);
ASSET (skills5_txt);
ASSET (skills6_txt);
ASSET (skills7_txt);
ASSET (skills8_txt);
ASSET (skills9_txt);
ASSET (awp1_txt);
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


// void initialize() {
  	// Lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  	// Lf2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  	// Lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  	// Rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Rf2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// chassis.calibrate();
	// pros::lcd::initialize();
	// imu.reset();
	// delay(1000);
	// pros::lcd::set_text(1, "Hello PROS User!");

// 	pros::lcd::register_btn1_cb(on_center_button);



// pros::Task screenTask ([&](){ void screen
// 	lemlib::Pose pose(0,0,0);
// 	while (true) {
//         lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
//         pros::lcd::print(0, "x: %f", pose.x); // print the x position
//         pros::lcd::print(1, "y: %f", pose.y); // print the y position
//         pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
//         pros::delay(10);
//     });


void initialize() {
	/*Lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  	Lf2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  	Lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  	Rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Rf2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);*/
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	chassis.setPose(0, 0, 0); //starting position

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
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
	// chassis.setPose(0, 0, 0);
	// chassis.moveToPose(6,60,20,2000,{true},true);
	// chassis.waitUntil(20);
	// Intake1 = 127;
	// chassis.waitUntil(60);
	// Intake1 = 0;
	// chassis.waitUntilDone();
	// chassis.moveToPoint(0, 0, 2000, false);
// 	//move from starting position to targeted position (x,y)
// 	chassis.moveToPose (25, 25, 90, 2000); //move to point 25, 25, facing 90 degrees at the end, mo
// //kP = 1.9, kI = 0.0003, kD = 2
// 	//move to a point (x,y) but doesnt care what direction
// 	chassis.moveToPoint (43, 7, 1000, true, 80);
// //do something in theh middle of function
// 	chassis.waitUntil(4);
// 	Intake1 = 127;
// 	chassis.waitUntil(8);
// 	Intake1 = 0;
// 	chassis.waitUntilDone(); // wait until the first move to pose function is completed


// // 			///THIS IS OffENSIVE AUTO 

// chassis.setPose(37,-56.5,0);
//  chassis.moveToPose(42.6,-0.4, -1.8,1500,{.forwards=true, .lead=.2}); //go for middle triball
// Intake1 = 100;
// chassis.waitUntil(74);
//  chassis.moveToPose(42.6, -0.4, 125.9, 2000, {false, 100}); //turn
// chassis.moveToPoint(85,-0.4, 2000, {true}); //push triiballs in
// // Intake1 = -127;
//  wings (1);
//  chassis.moveToPoint(20,-4, 700);
//  Intake1 = 0;
//  wings (0);
// chassis.moveToPose(31.5,-41.3, 1.6, 2000, {false, .2 }); //move it back

// delay (100);
//  chassis.moveToPose(51.9,-61.6, -56.6, 2000, {false, 13, .6, 127, 100}); //turn it so it aligns with post
// delay (100);
//  chassis.moveToPose (63.98, -61.6, -177.2, 2000, {false, 13, .6, 127, 100}); //descore
//   AWP.set_value(1);
//  chassis.moveToPose (65.6, -61.6, -175.7, 2000, {false, 13, .6, 127, 100}); //descore
// delay (1000);
// chassis.moveToPose (74.8, -61.6, -138.5, 2000,{true, 13., 6, 127, 100}); //trun to align for push
//  AWP.set_value(0);
// chassis.moveToPoint (74.8, -37.1, 2000, false, 127); //push




//  Intake1 = -127;
//  chassis.waitUntil(1);
//  wings(1);
//  chassis.waitUntil(10);
//  Intake1= 0;
//  chassis.moveToPoint(30,-4, 1000, false);
//  chassis.waitUntil(4);
//  wings(0);
//  chassis.moveToPose(30,-4, 232, 1200);
//  chassis.moveToPose(5,-24, 270, 1500,{.forwards=true, .lead=.25});
//  Intake1 = -127;
//  chassis.waitUntil(40);
//  Intake1 = 0;
//  chassis.moveToPose(14,-12, -215, 1500,{.forwards=false, .lead=.2, .maxSpeed=60});
//  chassis.moveToPose(29,-57, 181, 1700,{.forwards=true, .lead=.4});
//  chassis.moveToPose(29,-57, 270, 1000);
//  chassis.moveToPose(50,-49, 219.4, 1500, {.forwards=false, .maxSpeed=60});
//  chassis.waitUntil(5);
//  AWP.set_value(1);
//  chassis.moveToPose(48,-49, 180, 1500, {false});
//  chassis.waitUntil(2);
//  AWP.set_value(0);
//  chassis.moveToPose(43,-51, 48, 600);
//  chassis.moveToPose(56,-28, 0, 1500, {.forwards=true, .lead=.4});
//  Intake1 = -127;
//  chassis.waitUntil(10);
//  Intake1 = 0;

		////THIS IS THE WINPOINT AUTO 

// chassis.setPose(52, 48, -20); //start position
// AWP.set_value(1);
// chassis.moveToPose(47.7, 42.2, -82.2, 2000, {false, 10, .6, 100, 80}); //descore tribal
// chassis.waitUntil(4);
// AWP.set_value(0);
// chassis.waitUntilDone();
// chassis.moveToPose (61.4, 48, -267, 500, {true, 15, .3, 127, 100}); //turn so intake faces front
// chassis.moveToPose (80, 48, -267, 2000, {true, 15, .3, 127, 100}); //touch pole
// chassis.moveToPose (94, 48, 86.2, 2000);
// Intake1 = -127;









//  chassis.moveToPose (3.7, -5.8, -59, 2000, {false}, true);
//  chassis.moveToPose (14.1, -6.5, -135.1, 2000, {false}, true);
// chassis.moveToPose (46.7, 31.6, -138.5, 2000, {false}, true);
//  chassis.moveToPose (43.5,31.33, -70.5, 2000, {false}, true);
 
//  chassis.moveToPose (60.4, 24.01, -66.7, 2000, {false}, true);

// chassis.moveToPose (38.4, 33.4, -70.1, 2000, {true}, true);
// chassis.moveToPose (44.97, 34.38, -140.5, 2000);
// chassis.moveToPose(21.1, -7.64, -159.4, 2000);
// chassis.moveToPose (22.06, -4.43, 243.54, 2000);
// chassis.moveToPose (38.51, -11.81, -245.76,  2000);

//   //this is skills auto


// chassis.setPose(-49.5, -58, 152); //set the position 
// chassis.follow(skills1_txt, 20, 1500, false); //score the first 2 triballs 
// chassis.moveToPose(-58.7, -40, 157.5, 1500, {true, 20}, true);//go back


//  chassis.moveToPose(-82, -40, 80,500, {.forwards = false,.chasePower = 25, .lead = .6}); //going to cata position
// Cata = 127;
// delay (28300);
// Cata = 127;
// chassis.moveToPose (-56, -60, -22, 2000, {false, 25});// turn to align with bar
// chassis.moveToPose (-54.6, -65, -66.4, 1000, {false});//go back to descore
// // chassis.waitUntil(8);
// // AWP.set_value(1);
// // chassis.waitUntil(14);
// // AWP.set_value (0);
// // chassis.waitUntilDone();
//  chassis.moveToPose (15.2, -100, -67.2, 2000, {false}); //go to the other side 105.9
// chassis.moveToPose (59, -68, -154, 2000,{false, 25, .3 });//aligning with matchload
//  Cata = 0;
// Intake1=-127;

// // wings (1);
// chassis.moveToPose (45, -87, -158, 1000, {true, 25}); //go back
// // wings (0);
// chassis.moveToPoint(59, -70, 1000, false, 127 ); // push
// chassis.moveToPose (45, -87, -158, 1000, {true, 25}); //go back
// chassis.moveToPose (34.5, -81.9,  -29, 1000, {true, 25}); //turn so that it can go forward
// Intake1 = 0;
// chassis.moveToPose (8, -46.1, -29, 1000, {true, 25}); //move forward
// chassis.moveToPose (9.3, -47.1, 372.7, 500, {true, 25}); //turn
// chassis.moveToPose (-2.5, -47, -42, 500, {false, 25}); //move forward
//  chassis.moveToPose (13.2, -47, 103.8, 500, {true, 25}); //turn
// //wings (1);
// Intake1 = -127;
// chassis.moveToPose (56,-47, 106, 1700, {true, 25, .6, 127, 100}); //push
// chassis.moveToPose (9.5, -30.98, 107.6, 1000, {false, 25});//go back
// //wings (0); 
// Intake1 = 0;
// chassis.moveToPose (7.07, -32.5, 20.3, 500, {true, 25});//turn **********
// chassis.moveToPose (7.07, -24.5, 18.2, 500, {true, 25});//move forward
// chassis.moveToPose (7.07, -21.8, 108.2, 500, {true, 25});//turn
// Intake1 = -127;
// chassis.moveToPose (50, -31.7, 108.2, 1700, {true, 25, .6, 127, 100});//push
// wings (1);
// Intake1 = 0;
// chassis.moveToPose (13.5, -22.4, 108.2, 1000, {false, 25});//go back
// wings (0);
// chassis.moveToPose (11.5, -23.1, 17.97, 500, {true, 25});//turn ****
//  chassis.moveToPose (13.4, -16.8, 17.97, 500, {true, 25});//move forward
//  chassis.moveToPose (17.9, -12.6, 108.8, 500, {true, 25});//turn
// chassis.moveToPose (50, -22.9, 110.6, 2000, {true, 25, .6, 127, 100});//push
//  wings (1);
// chassis.moveToPose (32.9, -9, 110.4, 1000, {false, 25});//moveback
// wings (0);
// chassis.moveToPose (32.9, -9, -475.6, 500, {false, 25});//turnso back is 
// chassis.moveToPose (53.9  , 4.6, 215, 1000, {false, 25, .6, 127, 100});//move it forward so touches matchload bar
// chassis.moveToPose (61, 19.3, 215, 1000, {false, 25});//touch mathcload
// //  wings (1);
// chassis.moveToPose (54.22, 12.5, 325, 100, {false, 25});//turn to touch matchload
// // wings (0);
// chassis.moveToPose (70, 0.1, 325 , 100, {false, 25});//go forward on matchload bar
//  chassis.moveToPose (70, -3.6, 370.8, 500, {.forwards = false, .chasePower = 25, .lead = 0.2});//turn to push
//  chassis.moveToPose (70,  -25,373.9, 1500, {false, 25, .6, 127, 100}); //push
//  chassis.moveToPose (60.3,  2 ,373.9, 1500, {true, 25, .6, 127, 100}); //go back
//    chassis.moveToPose (60.3,  -25,373.9, 1500, {false, 25, .6, 127, 100}); //push
//     chassis.moveToPose (60.3,  2 ,373.9, 1500, {true, 25, .6, 127, 100}); //go back
//    chassis.moveToPose (60.3,  -25,373.9, 1500, {false, 25, .6, 127, 100}); //push
//     chassis.moveToPose (60.3,  2 ,373.9, 1500, {true, 25, .6, 127, 100}); //go back
// chassis.moveToPose (68, -5.47, 355.3, 2000, {false, 20, .6, 127, 100}); //push
// chassis.moveToPose (70,4.6, 7, 500, {true, 25, .6, 127, 100}); //go back
// chassis.moveToPose (70, -5.47, 7, 2000, {false, 25, .6, 127, 100}); //push
// chassis.moveToPose (70,4.6, 7, 1000, {true, 25, .6, 127, 100}); //go back





// // wings (1);
// chassis.moveToPose (11.04, -44.9, 422.2, 2000, {true, 20, .6, 127, 100}); //turn to push
// delay (1000);
// chassis.moveToPose (39.6, -33.8, 439.99, 2000, {true, 20, .6, 127, 100}); //push
// delay (500);
// chassis.moveToPose (23.4, -24.97, 461.99, 2000, {false}); //move it back
// delay (500);
// chassis.moveToPose (46.7, -28.98, 459.87, 2000, {true, 20, .6, 127, 100}); //push #2
// chassis.moveToPose (19.6, -24.4, 440, 2000, {false}); //move it back
// chassis.moveToPose (48.0, -19.6, 444.3, 2000 {true, 20, .6, 127, 100}); //thirs push
// chassis.moveToPoint (27.9, -35.9, 2000, false, 2000); //go back
// delay (500);
// chassis.moveToPose (23.3, -31.1, 433.9, 2000);//turn to align for second push
// chassis.moveToPose (47.4, -25.6, 461.9, 2000, {true, 20, .6, 127, 100});//go push
// //wings (0);


// chassis.moveToPose (14.6, -12.6, 787.02, 2000, {false, 15, .6, 80, 100}, true); //go back
// chassis.moveToPose (41.2, -1.8, 87.5, 2000, {true, 20, .6, 127, 100}, true);//push
// delay (500);
// chassis.moveToPose (10.2, -.1, 86.6, 2000, {false, 10, .6, 80, 100}, true); //go back
// wings (0);
// chassis.moveToPose (11.8, -.2, 173.8, 2000, {false, 100, .6, 80, 100}, true); //turn 90 degrees
// chassis.moveToPose (4.6, 27.6, 170.9, 2000, {false, 15, .6, 100, 80}, true);//move back to edge
// chassis.moveToPose (3.6, 25.2, 259, 2000, {false, 15, .6, 100, 80}, true);//turn 90 degrees
// chassis.moveToPose(42, 32.2, 257, 2000, {false, 15, .6, 100, 80}, true);//go back to net
// chassis.moveToPose (42.6, 32.9, 352.5, 2000, {false, 15, .6, 100, 80, true});//turn to push
// chassis.moveToPose (45.6, 23.6, 350.3, 2000, {false, 20, .6, 127, 100}, true); //push/



// chassis.setPose(25, -62, 90);
//chassis.follow(skills3_txt, 10, 2000);
//chassis.moveToPose(-36.022, -48.417, 79.538, 2000);
/*chassis.moveToPose(3.60, 23.48, 27.15, 2000, {true,8,0.6,100,100}, true);
chassis.moveToPose (-1.955, 8.227, 22.799, 2000, {false}, true);
chassis.moveToPose (0.991, 9.152, 86.078, 2000,{false}, true );
//matchload
chassis.moveToPose(14.0, 12.97,88.99,2000);
chassis.moveToPose(22.613, 7.0, 181.943, 2000);
delay (100);
chassis.moveToPose (16.579, -9.417, 203.763, 2000);
delay(100);
chassis.moveToPose (16.834, -3.375, 109.370,2000);*/


	//PUSH ONE TRIBALL IN

	// moveForward(50000);


//Offensive 
Intake1 = 127;
delay (100);
Intake1 = 0;
wings (1);
delay (500);
wings (0);
chassis.setPose(55, -40, -28.5);
chassis.moveToPose (29.5, 17, -19.4,1000, {true, 25, .3});//go for middle triball
Intake1 = 127;
chassis.moveToPose (29.5, 17, 95.2, 700);//turn
chassis.moveToPose (70, 17, 95.2, 2000, {true, 25, .6, 127, 100});//push hmiddle triball and the other one
wings (1);
Intake1 = 0;
chassis.moveToPose (64.6, 17, 248.9, 700, {true, 25});//turn right
wings(0);
Intake1 = 127;
chassis.moveToPose (35.8, -3, 248.9, 1000, {true, 20});//move forwardd to get acorn

chassis.moveToPose (40, -3.9, 105, 700, {true, 20});//turn left 
chassis.moveToPose (48, -3.9, 105, 700 ,{true, 20});//move forward and outtake
Intake1 = -100;
chassis.moveToPose (48, -15, -39, 700, {false, 20});// chassis.moveToPose (40, -8.9, -54.6, 2000); //spin so back is facing back
Intake1 = 0;
chassis.moveToPose (61.9, -42.1, -36.3, 1500, {false, 20});//go back to touch matchload
chassis.moveToPose (56, -44, 225.5, 500, {false});//turn for matchload
chassis.moveToPose (56, -44, -125, 700, {false, 10, .6, 100, 80});//move back
AWP.set_value(1);
chassis.moveToPose(75,-32.8, -149.9, 1000, {false, 7, .4, 100, 80});// descore
  chassis.moveToPose (85, -16.4, -172, 2000, {false, 20, .6, 100, 80}); //push
 AWP.set_value (0);
 chassis.moveToPoint (85, -25, 2000, true, 127); //go back
   chassis.moveToPoint (85, -16.4, 2000, false, 127); //push
 chassis.moveToPoint (85, -25, 2000, true, 127); //go back
  
  

	
	

 }








// /* * Runs the operator control code. This function will be started in its own task
//  * with the default priority and stack size whenever the robot is enabled via
//  * the Field Management System or the VEX Competition Switch in the operator
//  * control mode.
//  *
//  * If no competition control is connected, this function will run immediately
//  * following initialize().
//  *
//  * If the robot is disabled or communications is lost, the
//  * operator control task will be stopped. Re-enabling the robot will restart the
//  * task, not resume it from where it left off.
//  */
void opcontrol() {
	bool wingPosition = false;
	bool HangPosition = false;
	bool AWPPosition = false;


// chassis.setPose(-49.5, -58, 152); //set the position 
// chassis.follow(skills1_txt, 20, 1500, false); //score the first 2 triballs 
// chassis.moveToPose(-58.7, -40, 157.5, 2000, {true}, true);//go back
//  chassis.moveToPose(-67, -39, 90,2000, {.forwards = false, .lead = .6}); //going to cata positio
// 	 chassis.moveToPose(-67, -39, 90,2000, {.forwards = false, .lead = .6}); //going to cata positio
// Cata = 127;
// delay (28600);

	
	while (true) {

	int Forward = master.get_analog(ANALOG_RIGHT_Y);
	int Turn = master.get_analog(ANALOG_RIGHT_X) * .80;
	powerDrive(Forward,Turn);
	
	
	if (master.get_digital(DIGITAL_B)){
		Cata = 127;


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
	if (master.get_digital_new_press(DIGITAL_A)){
		AWPPosition = !AWPPosition;
		AWP.set_value(AWPPosition);
	}
	
}

}
