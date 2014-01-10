#include "WPILib.h"
#include "PID.h"
#include "Timer.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	Timer autonTimer;
	CANJaguar frontLeft;
	CANJaguar frontRight;
	CANJaguar backRight;
	CANJaguar backLeft;
	RobotDrive myRobot; // robot drive system
	
	Joystick stick; // only joystick
	
	PID driveLeftPID;
	PID driveRightPID;

public:
	RobotDemo(void):
		
		stick(1),
		frontLeft(14),
		backLeft(15),
		frontRight(11),
		backRight(10),
		myRobot(frontLeft, backLeft, frontRight, backRight),
		driveLeftPID(0,0,0,100,100),
		driveRightPID(0,0,0,100,100)
		
	{
		/*
		backLeft.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		backRight.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		*/
		backRight.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		backLeft.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		
		backLeft.ConfigEncoderCodesPerRev(360);
		backRight.ConfigEncoderCodesPerRev(360);
		
		myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		autonTimer.Start();
		// bool isHot = true;
		
		double distanceTravelled = 0;
		while (autonTimer.Get() < 5)
		{
			myRobot.Drive(-0.5,0.0);
			distanceTravelled = 18.85 * ((backRight.GetPosition()+backLeft.GetPosition())/2.0);
			SmartDashboard::PutNumber("Distance",distanceTravelled); 
		}
		myRobot.Drive(0.0,0.0);
		
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
			
			distanceTravelled = 18.85 * ((backRight.GetPosition()+backLeft.GetPosition())/2.0);
			SmartDashboard::PutNumber("Distance",distanceTravelled);
			SmartDashboard::PutNumber("Distance R",distanceTravelled);
			SmartDashboard::PutNumber("Distance L",distanceTravelled);
			
			
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

