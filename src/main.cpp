#include "main.h"
#include "lemlib/api.hpp"
#include "definitions.hpp"





/*ASSET Defintions*/

	/*Skills*/
	ASSET(skills1_txt);
	ASSET(skills2_txt);


/*End of ASSET Defintions*/

/*Variable Definitions*/

    /*Controller Variables*/
    bool lastKnownButtonR1State;
	bool lastKnownButtonBState;
	bool lastKnownButtonYState;
	bool lastKnownButtonRightState;
	bool lastKnownButtonDownState;
	bool wingToggle = false;
	bool blockerToggle = false;
	bool hangToggle = false;

	/*Kicker Variables*/
	bool slapperFireToggle = true;
	bool kickerSet = false;




/*End of Variable Definitions*/


/*Device Initilization*/

	/*Drivetrain Initilizations*/

	pros::Motor lD1(LD1, SPEEDBOX, true);
	pros::Motor lD2(LD2, SPEEDBOX, true);
	pros::Motor lD3(LD3, SPEEDBOX, true);
	pros::Motor rD1(RD1, SPEEDBOX, false);
	pros::Motor rD2(RD2, SPEEDBOX, false);
	pros::Motor rD3(RD3, SPEEDBOX, false);

	pros::MotorGroup lDrive({lD1, lD2, lD3});
	pros::MotorGroup rDrive({rD1, rD2, rD3});

	pros::Imu imu(IMU_PORT);

	pros::Rotation odomRot(ODOM_ROT, false);

	lemlib::TrackingWheel odomWheel(&odomRot, 2.75, 0, 1);

	/*End of Drivetrain Initializations*/


	/*Non-DT Initializations*/

	pros::Motor slapperMotor(SLAP_PORT, TORQUEBOX, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor intakeMotor(INTAKE_PORT, SPEEDBOX, false);

	pros::Distance kickerDistance(DISTANCE_PORT);

	pros::ADIDigitalOut vertWingPnuem(VERTW_ADIDO);
	pros::ADIDigitalOut horiWingPnuem(HORIW_ADIDO);
	pros::ADIDigitalOut blockerPnuem(BLOCKER_ADIDO);
	pros::ADIDigitalOut hangPnuem(HANG_ADIDO);



	/*End of Non-DT Initializations*/

	/*Controller Initialization*/

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	/*End of Controller Initilization*/


/*End of Device Initilization*/


/*LemLib Chassis Initializations*/

	/*LemLib Drivetrain Initilization*/
	lemlib::Drivetrain drivetrain
	{
		&lDrive, /*Pointer to the left drive channel*/
		&rDrive, /*Pointer to the right drive channel*/
		10.5, /*Track Width*/
		3.25, /*Wheel Diameter*/
		450, /*Wheel RPM*/
		8 /*Chase Power*/
	};
	/*End of LemLib Drivetrain Initilization*/


	/*LemLib Odometry Initilization*/
	lemlib::OdomSensors odomSensors
	{
		&odomWheel, /*Center Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		&imu /*Inertial Sensor*/
	};
	/*End of LemLib Odometery Sensors Initilization*/


	/*Lateral (Forwards/Backwards) PID Initilization*/
	lemlib::ControllerSettings lateralController
	(
		8,  //16, // kP
        0, //3 // kI
		32, //80, // kD
        0, //4 // Windup Range
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
    );
	/*End of Lateral (Forwards/Backwards) PID Initilization*/


	/*Angular (Turning) PID Initilization*/
	lemlib::ControllerSettings angularController(
		4,  //7 // kP
        0, // kI
		40, //60 // kD
        0, // Windup Range
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
    );
	/*End of Angular (Turning) PID Initilization*/


	/*LemLib Chassis Initilization*/
	lemlib::Chassis drive(drivetrain, lateralController, angularController, odomSensors);
	/*End of LemLib Chassis Initilization*/


/*End of LemLib Chassis Initializations*/








/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	drive.calibrate();
	intakeMotor.set_brake_mode(HOLD);
	slapperMotor.set_brake_mode(COAST);	
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
void autonomous() 
{
	lDrive.set_brake_modes(HOLD);
	rDrive.set_brake_modes(HOLD);

	drive.setPose(-44, -56, 135);

	drive.follow(skills1_txt, 15, 2000, false);
	//drive.moveToPoint(poseX, poseY - 8, 500);
	//drive.moveToPoint(poseX, poseY + 10, 500, {.forwards = false, .minSpeed = 127});
	drive.moveToPoint(-62, -39, 500);
	drive.turnTo(62, -5, 1000, {.forwards = false});
	drive.waitUntilDone();
	/*horiWings Out Here*/
	/*wait while Shooting Here*/
	/*horiWings In Here*/
	drive.moveToPoint(-31, -59, 1000, {.forwards = false});
	drive.moveToPoint(31, -59, 2000, {.forwards = false});
	drive.follow(skills2_txt, 15, 2000, false);
	vertWingPnuem.set_value(1);
	drive.waitUntil(26);
	vertWingPnuem.set_value(0);
	


	
	
	
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
void opcontrol() 
{
	lDrive.set_brake_modes(COAST);
	rDrive.set_brake_modes(COAST);

	pros::Task task([]()
	{
		while(true)
		{
			while(slapperFireToggle)
			{
				if (kickerDistance.get() <= 5 || slapperMotor.get_position() <= 85)
				{
					slapperMotor.move(127);
				}
				else
				{
					pros::delay(100);
					slapperMotor.move(0);
				}
				pros::delay(10);
			}
			pros::delay(10);
		}
	});
	
	while(true)
	{
		
		std::cout << "Distance: " << kickerDistance.get() << " Motor Position: " << slapperMotor.get_position() << std::endl;

		drive.tank(masterLeftY, masterRightY, 5);

		


		if (masterL1)
		{
			intakeMotor.move(-127);
		}
		else if (masterL2) 
		{
			intakeMotor.move(127);
		}
		else
		{
			intakeMotor.move(0);
		}


		if (masterR1 != lastKnownButtonR1State)
		{
			lastKnownButtonR1State = masterR1;
			if (masterR1)
			{
				slapperFireToggle = !slapperFireToggle;
			}
		}

		

		if (masterR2)
		{
			horiWingPnuem.set_value(1);
		}
		else
		{
			horiWingPnuem.set_value(0);
		}

		// Vertical Wing Pneumatics Toggle
		if(masterY != lastKnownButtonYState)
		{
			lastKnownButtonYState = masterY;
			if(masterY)
			{
				wingToggle = !wingToggle;
				vertWingPnuem.set_value(wingToggle);
			}
		}


		if(masterRight != lastKnownButtonRightState)
		{
			lastKnownButtonRightState = masterRight;
			if(masterRight)
			{
				blockerToggle = !blockerToggle;
				slapperFireToggle = !slapperFireToggle;
				blockerPnuem.set_value(blockerToggle);
			}
		}

		if(masterDown != lastKnownButtonDownState)
		{
			lastKnownButtonDownState = masterDown;
			if(masterDown && blockerToggle == false)
			{
				blockerToggle = !blockerToggle;
				slapperFireToggle = !slapperFireToggle;
				blockerPnuem.set_value(blockerToggle);
			}
			else if(masterDown && blockerToggle == true)
			{
				hangToggle = !hangToggle;
				hangPnuem.set_value(hangToggle);
				blockerToggle = !blockerToggle;
				blockerPnuem.set_value(blockerToggle);
				
			}
			
		}



		





	}
	
}