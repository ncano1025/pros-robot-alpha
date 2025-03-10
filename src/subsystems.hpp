#include "main.h"

extern Controller master;
extern pros::MotorGroup left_mg;    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
extern pros::MotorGroup right_mg;  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
extern pros::MotorGroup chassis;

extern pros::Imu imu;
extern pros::Rotation rotation;
extern pros::MotorGroup intake;

extern pros::Motor dunker;

extern std::shared_ptr<ChassisController> drive;

extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics sweeper;
extern pros::ADIPotentiometer sensor;