#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	CANJaguar frontLeft;
	CANJaguar frontRight;
	CANJaguar backRight;
	CANJaguar backLeft;
	Joystick stick; // only joystick

public:
	RobotDemo(void):
		
		frontLeft(14),
		backLeft(15),
		frontRight(11),
		backRight(10),
		myRobot(frontLeft, backLeft, frontRight, backRight),	// these must be initialized in the same order
		stick(1)		// as they are declared above.
	{
		myRobot.SetExpiration(0.1);
		
		backLeft.ConfigEncoderCodesPerRev(250);
		backLeft.SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		backRight.ConfigEncoderCodesPerRev(250);
		backRight.SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

