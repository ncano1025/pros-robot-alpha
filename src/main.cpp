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

double getYawQuaternion() {
	pros::quaternion_s_t qt = imu.get_quaternion();

	//error fetching quat, retry
	if (qt.w == PROS_ERR_F) {
		qt = imu.get_quaternion();
		if (qt.w == PROS_ERR_F) {
			pros::lcd::set_text(1, "ERROR: IMU Quaternion Fetch Failed");
			return -1.0;
		}
	}

	//convert quat to yaw
	double yaw = atan2(2 * ((qt.w * qt.z) + (qt.x * qt.y)), 1 - (2 * ((qt.y * qt.y) + (qt.z * qt.z)))); //yaw formula = atan2(2(wz + xy), 1 - 2(y^2 + z^2))

	//returns yaw converted from rad to deg; angle is returned from -180 to 180 (+ 180 for [0, 360])
	return ((yaw * (180 / M_PI)) + 180);
}