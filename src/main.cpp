#include "main.h"
#include "subsystems.hpp"

void initialize() {
	sensor.calibrate();
	imu.tare_heading();
	dunker.set_brake_mode(MOTOR_BRAKE_HOLD);	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

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

		// intake
		if(master.getDigital(ControllerDigital::L2))
			intake.move(127);
		else if(master.getDigital(ControllerDigital::L1))
			intake.move(-127);
		else 
			intake.move(0);

		// clamp toggle
		if (clampToggle)
			clamp.set_value(true); // turns clamp solenoid on
		else
			clamp.set_value(false); // turns clamp solenoid off
		if (master.getDigital(ControllerDigital::R1)) {
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
		if (master.getDigital(ControllerDigital::R2)) {
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