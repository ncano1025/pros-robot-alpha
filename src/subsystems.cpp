#include "main.h"

Controller master;
pros::MotorGroup left_mg({-1, 4, 17, -18});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({7, -8, 15, -21});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::MotorGroup chassis({-1, 4, 17, -18, 7, -8, 15, -21});

pros::MotorGroup intake({-6, 13});
pros::Motor dunker(-10);

pros::Imu imu(12);
pros::Rotation rotation(5);
pros::ADIPotentiometer sensor('A', pros::E_ADI_POT_EDR);

pros::adi::Pneumatics clamp('H', false);
pros::adi::Pneumatics sweeper('G', false);

std::shared_ptr<ChassisController> drive =
    ChassisControllerBuilder()
        .withMotors({-1, 4, 17, -18}, {7, -8, 15, -21})
        .withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 12_in}, imev5BlueTPR})
        .withOdometry()
        /*
        .withGains(
            {0.0025, 0, 0.00000}, // Distance controller gains (kP, kI, kD)
            {0.00115, 0, 0}, // Turn controller gains (kP, kI, kD)
            {0, 0, 0}  // Angle controller gains for path following (kP, kI, kD)
        )
        */
        .build();


