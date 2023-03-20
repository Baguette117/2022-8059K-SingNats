#include "main.h"
#include "globals.hpp"
#include "pros/adi.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Motor leftFM(leftFMPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor leftBack(leftBackPort, E_MOTOR_GEAR_GREEN, false, E_MOTOR_ENCODER_DEGREES);
	Motor rightFM(rightFMPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor rightBack(rightBackPort, E_MOTOR_GEAR_GREEN, true, E_MOTOR_ENCODER_DEGREES);
	Motor feedLeft(feedLeftPort,  E_MOTOR_GEAR_BLUE, false, E_MOTOR_ENCODER_DEGREES);
	Motor feedRight(feedRightPort,  E_MOTOR_GEAR_BLUE, true, E_MOTOR_ENCODER_DEGREES);
	ADIEncoder leftEnc(leftEncT, leftEncB, false);
	ADIEncoder rightEnc(rightEncT, rightEncB, true);
	ADIDigitalOut expansion (expansionPort, LOW);
	Controller master (CONTROLLER_MASTER);

	Task catapultPIDTask (catapultPID, (void*)"Casey kinda sus", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "catapultPIDTask");
	// Task autonPIDTask (autonPID, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "autonPIDTask");
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
	// near();
	// far();
	full();
	// calibration();
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
	Motor leftFM(leftFMPort, false);
	Motor leftBack(leftBackPort, false);
	Motor rightFM(rightFMPort, true);
	Motor rightBack(rightBackPort, true);
	Motor feedLeft(feedLeftPort,  false);
	Motor feedRight(feedRightPort, true);
	ADIDigitalOut expansion (expansionPort, LOW);
	Controller master (CONTROLLER_MASTER);

	double left, right;
	bool inverted = false, precision = false, feedToggle = false;
	while (true) {
		if (master.get_digital_new_press(DIGITAL_A)){
			inverted = !inverted;
			master.clear();
		}

		if (master.get_digital_new_press(DIGITAL_Y)){
			precision = !precision;
			master.clear();
		}
		
		if (master.get_digital_new_press(DIGITAL_R2)){ //Feed
			feedToggle = !feedToggle;
		}


		left = master.get_analog(ANALOG_LEFT_Y);
		right = master.get_analog(ANALOG_RIGHT_Y);
		
		if (precision) {
			left *= .5;
			right *= .5;
			master.print(1, 0, "Slow");
		} else {
			left *= .9;
			right *= .9;
			master.print(1, 0, "Fast");
		}

		if (inverted){
			leftFM.move(-right);
			leftBack.move(-right);
			rightFM.move(-left);
			rightBack.move(-left);
			master.print(1, 6, "Inverted");
		} else {
			leftFM.move(left);
			leftBack.move(left);
			rightFM.move(right);
			rightBack.move(right);
			master.print(2, 0, "Normal");
		}

		if (master.get_digital(DIGITAL_R1)){ //Feed
			feedLeft.move(127);
			feedRight.move(127);
		} else if (master.get_digital(DIGITAL_R2)){
			feedLeft.move(intakeSpeed);
			feedRight.move(intakeSpeed);
		} else {
			feedLeft.move(0);
			feedRight.move(0);
		}

		if (master.get_digital_new_press(DIGITAL_L2)){ //Catapult
			catapultShoot();
			printf("Shot\n");
		}

		if (master.get_digital(DIGITAL_UP)){ //Expansion
			expansion.set_value(HIGH);
		} else if (master.get_digital(DIGITAL_DOWN)){
			expansion.set_value(LOW);
		}
		delay(20);
	}
}
