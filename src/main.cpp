#include "main.h"
#include "subsystems.hpp"

void initialize() {
	sensor.calibrate();
	imu.tare_heading();
	dunker.set_brake_mode(MOTOR_BRAKE_HOLD);	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	while (imu.is_calibrating())
		pros::delay(10);
	
	drive->setMaxVelocity(100);
	
	// turn towards MG1
	turnPID(-27.25, 0);

	// drive towards MG1
	drive->moveRawAsync(-1050);
	pros::delay(2250);
	clamp.set_value(true);
	intake.move_voltage(12000);

	// turn towards RING1
	turnPID(180.00, 0.5);

	// drive towards RING1
	drive->moveRawAsync(650);
	pros::delay(2500);

	// turn towards RING2
	turnPID(90);

	// drive towards RING2
	drive->moveRawAsync(300);
	pros::delay(1800);
	drive->moveRaw(-250);

	// turn towards RING3
	turnPID(180);

	// drive towards RING3
	drive->moveRawAsync(650);
	pros::delay(3500);

	// sweep blue ring away
	intake.move_voltage(0);
	turnPID(270);
	sweeper.set_value(true);
	pros::delay(500);
	turnPID(180);
	sweeper.set_value(false);
	intake.move_voltage(12000);

	// align towards RING4
	turnPID(215, 0.5);

	// drive towards RING4
	drive->moveRawAsync(1000);
	pros::delay(3500);
	intake.move_voltage(-12000);
	drive->moveRawAsync(100);
	pros::delay(750);

	// turn towards RING5
	turnPID(90, 1);

	intake.move_voltage(12000);

	// drive towards RING5
	drive->setMaxVelocity(150);
	drive->moveRawAsync(700);
	pros::delay(2000);
	intake.move_voltage(0);
	
	// turn to corner
	turnPID(-50);
	intake.move_voltage(-12000);
	pros::delay(2000);
	intake.move_voltage(0);

	// move to corner
	drive->moveRawAsync(-425);
	pros::delay(750);
	clamp.set_value(false);
	pros::delay(1000);

	// move back from corner
	drive->moveRaw(395);

	// turn to MG2
	turnPID(89, 0);

	// outtake extra blues
	intake.move_voltage(-12000);

	// drive to MG2
	drive->moveRawAsync(-1450);
	pros::delay(2150);
	clamp.set_value(true);
	intake.move_voltage(0);
	pros::delay(1000);

	// turn to RING6
	turnPID(270);
	intake.move_voltage(12000);

	// drive to RING6
	drive->moveRaw(550);
	pros::delay(1500);
	intake.move_voltage(0);
	clamp.set_value(true);
	pros::delay(500);

	// turn to corner
	turnPID(70);
	clamp.set_value(false);
	pros::delay(500);

	// drive to corner
	drive->moveRawAsync(-1400);
	pros::delay(5000);
	clamp.set_value(false);
	drive->moveRaw(100);
	turnPID(15);
	drive->moveRawAsync(400);
	pros::delay(500);
	intake.move_voltage(6000);
	pros::delay(1000);
	intake.move_voltage(0);
	pros::delay(500);

	// turn to MG3
	turnPID(-130); //either -140 or -130, 135 was a little offset

	// outtake extra blues
	intake.move_voltage(-12000);

	// drive to MG3
	drive->moveRawAsync(-1400);
	pros::delay(1000);
	intake.move_voltage(0);
	clamp.set_value(true);
	pros::delay(1000);
	intake.move_voltage(12000);
	pros::delay(2000);
	intake.move_voltage(0);
}

void opcontrol() {

	bool clampToggle = false, clampLatch = false, sweeperToggle = false, sweeperLatch = false;
	double leftY, rightY, rightX;

	pros::lcd::initialize();

	while (true) {

		// tank drive with reduced speeds
		leftY = master.getAnalog(ControllerAnalog::leftY) * 0.75;
		rightY = master.getAnalog(ControllerAnalog::rightY) * 0.75;

		// slight dampener?
		if(abs(leftY) <= 0.1 || abs(rightY) <= 0.1)
			drive->getModel()->tank(leftY * 0.5, rightY * 0.5);
		
		else 
			drive->getModel()->tank(leftY, rightY);

		// auton testing
		if(master.getDigital(ControllerDigital::left)) {
			autonomous();
			//turnPID(90);
			pros::lcd::set_text(2 , "Auto finished!");
		}

		// intake
		if(master.getDigital(ControllerDigital::R1))
			intake.move(127);
		else if(master.getDigital(ControllerDigital::R2))
			intake.move(-127);
		else 
			intake.move(0);

		// clamp toggle
		if (clampToggle)
			clamp.set_value(true); // turns clamp solenoid on
		else
			clamp.set_value(false); // turns clamp solenoid off
		if (master.getDigital(ControllerDigital::L1)) {
			if(!clampLatch){ // if latch is false, flip toggle one time and set latch to true
				clampToggle = !clampToggle;
				clampLatch = true;
			}
		}
		else
			clampLatch = false; //once button is released then release the latch too

		// sweeper toggle
		if (sweeperToggle)
			sweeper.set_value(true); // turns clamp solenoid on
		else
			sweeper.set_value(false); // turns clamp solenoid off
		if (master.getDigital(ControllerDigital::L2)) {
			if(!sweeperLatch){ // if latch is false, flip toggle one time and set latch to true
				sweeperToggle = !sweeperToggle;
				sweeperLatch = true;
			}
		}
		else
			sweeperLatch = false; //once button is released then release the latch too

		// fast dunker
		if(master.getDigital(ControllerDigital::X))
			dunker.move(120);
		else if(master.getDigital(ControllerDigital::B))
			dunker.move(-120);

		// slow dunker
		else if(master.getDigital(ControllerDigital::up))
			dunker.move(127 * 0.3);
		else if(master.getDigital(ControllerDigital::down))
			dunker.move(-127 * 0.3);
		else
			dunker.brake();
		
		pros::delay(20);
	}
}