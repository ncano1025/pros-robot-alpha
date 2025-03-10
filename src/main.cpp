#include "main.h"
#include "subsystems.hpp"

// drive PID variables
double drive_kP = 0.0;
double drive_kI = 0.0;
double drive_kD = 0.0;

// turn PID variables
bool autonomousPID = false;

double turn_kP = 1.0;
double turn_kI = 0.0;
double turn_kD = 0.0;

double heading, target;

double turn_error, turn_prev_error = 0.0, turn_total_error = 0.0;
double turn_derivative;

double optimized_angle, temp_angle, turn_PID;

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

void turnPID(double target, double weightAdjustment) {
	int correctCount = 0;
	heading = getYawQuaternion();
	pros::lcd::print(1, "Heading = %lf, Target = %lf", heading, target);
	
	while (correctCount <= 10) {
		
		heading = getYawQuaternion() - 180;
		optimized_angle = target - heading;
			
		pros::lcd::print(1, "Heading = %lf, Target = %lf", heading, target);

		if(target > heading){
			temp_angle = ((360 - target) + heading) * -1;
			if(abs(optimized_angle) > abs(temp_angle))
				optimized_angle = temp_angle;
		}
		else {
			temp_angle = (360 - heading) + target;
			if(abs(optimized_angle) > abs(temp_angle))
				optimized_angle = temp_angle;
		}

		// proportion
		turn_error = optimized_angle;

		pros::lcd::print(3, "Optimized = %lf", optimized_angle);

		// integral
		turn_total_error += turn_error;

		// derivative
		turn_derivative = turn_error - turn_prev_error;
		
		// get prev error for next instance
		turn_prev_error = turn_error;

		turn_PID = ((turn_kP * turn_error) / 360);
		turn_PID += copysign(0.12 + (weightAdjustment * 0.05), turn_PID);

		pros::lcd::print(2, "PID = %lf", turn_PID);

		if(abs(turn_PID) >= 0.5)
			drive->getModel()->tank(copysign(0.5, turn_PID), copysign(0.5, -turn_PID));
		
		else if(abs(optimized_angle) > 0.15)
			drive->getModel()->tank(turn_PID, -turn_PID);
		
		else
			drive->getModel()->tank(0, 0);
		
		if(abs(optimized_angle) <= 0.2)
			correctCount++;
		
		pros::delay(10);
	}

	drive->getModel()->tank(0, 0);

	pros::delay(250);
	
}