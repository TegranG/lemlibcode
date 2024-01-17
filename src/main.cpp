#include "lemlib/api.hpp"
#include "main.h"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

//object definitions

//auton selector def
bool auto_started = false;

int current_auton_selection = 0;

//drivetrain

//indisual drivetrain motors

pros::Controller master(pros::E_CONTROLLER_MASTER);


pros::Motor leftBack(19,pros::E_MOTOR_GEARSET_06, true); //left back motor(port,gearbox,reversed?)
pros::Motor leftFront(18,pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftTop(16,pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightBack(12,pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightTop(20,pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFront(13,pros::E_MOTOR_GEARSET_06, false);

//drivetrain motor groups

pros::MotorGroup leftSide({leftBack,leftFront,leftTop});
pros::MotorGroup rightSide({rightBack,rightFront,rightTop});

//flywheel

pros::Motor flywheel(17,pros::E_MOTOR_GEARSET_06,false);

//Inertial sensor

//horizantel odom pod

pros::ADIEncoder enc('A', 'B', true); // ports A and B, reversed

lemlib::TrackingWheel horizontal(&enc,2.75,6,1);

pros::Imu intSensor(15);

//tracking wheel

//pros::Rotation rot(1, false); // port 1, not reversed

//lemlib drivetrain 

lemlib::Drivetrain drivetrain (&leftSide, &rightSide, 14,lemlib::Omniwheel::NEW_325, 360,2);

//PID Controllers

//pid tuning

// Move the robot 10 inches forward using the chassis.moveTo() function
// increase kP until the robot starts oscillating
// increase kD until the oscillation stops
// record kP and kD values
// repeat steps 2-4 until you can't stop the oscillation. At this point, use the last kP and kD values you recorded.

//forward/backward controller

lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// turning PID

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// odometry struct

lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    &horizontal, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &intSensor // inertial sensor
};

//lemlib chassisdriveTrain

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		//pros::lcd::set_text(2, "I was pressed!");
	} else {
		//pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void screen() {
    // loop forever
    while (true) {
        //debugg code
		
		// pros::lcd::print(0, "x: %f", pose.x); // print the x position
        // pros::lcd::print(1, "y: %f", pose.y); // print the y position
        // pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
	
		//auton selector code

		//selector::init();

		pros::delay(10);
    }
}
 
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
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
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
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
void competition_initialize() { //preauton
	// while(auto_started == false){            //Changing the names below will only change their names on the
  //   switch(current_auton_selection){       //Tap the brain screen to cycle through autons.
  //     case 0:
  //       pros::lcd::print(0, "5 ball"); //50, 50, "farside 5 ball"
  //       break;
  //     case 1:
  //       //Brain.Screen.printAt(50, 50, "farside 5 ball");
  //       break;
  //   //   case 2:
  //   //     pros::lcd::print(0, "farside1ball");
  //   //     break;
  //   //   case 3:
  //   //     pros::lcd::print(0, "wpcloseside");
  //   //     break;
  //   //   case 4:
  //   //     pros::lcd::print(0, "skillsauton");
  //   //     break;
  //   //   case 5:
  //   //     pros::lcd::print(0, "elimFarSide");
  //   //     break;
  //   //   case 6:
  //   //     pros::lcd::print(0, "WPUPDATED");
  //   //     break;
  //   //   case 7:
  //   // 	pros::lcd::print(0, "Holonomic Odom Test");
  //   // 	break;
  //   }
  //   if(pros::lcd::read_buttons()  == LCD_BTN_LEFT || LCD_BTN_CENTER || LCD_BTN_RIGHT){ // if brain was pressed at all
  //     while(pros::lcd::read_buttons()  == LCD_BTN_LEFT  || LCD_BTN_CENTER || LCD_BTN_RIGHT) {}
  //     current_auton_selection ++;
  //   } else if (current_auton_selection == 8){
  //     current_auton_selection = 0;
  //   }
	// }
}

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
    // example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    // chassis.moveToPose(20, 15, 90, 4000);
    // pros::lcd::print(4, "pure pursuit finished!");
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
	

        // get joystick positions
 //while (true) {
        // while(1) {
        // // Retrieve the necessary joystick values
        // int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // // Move the left side of the robot
        // leftSide.move(leftY + rightX);
        
        // // Move the right side of the robot 
        // rightSide.move(leftY - rightX);
        // pros::delay(20);
    //}
}


