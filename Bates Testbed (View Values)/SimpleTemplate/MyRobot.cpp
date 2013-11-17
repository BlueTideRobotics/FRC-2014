// Nicholas Currault, 11/8/13

#include "WPILib.h"
#include <Ultrasonic.h>

class RobotDemo : public SimpleRobot
{	
	DigitalInput irSensor;
	DigitalInput limitSwitch;

	Ultrasonic ultraSonic;

public: 
	RobotDemo(void):
		irSensor(5),
		limitSwitch(10),
		
		ultraSonic(1,3)
	{
			//leave this here
	}
	
	void Autonomous(void)
	{
	}

	void OperatorControl(void)
	{
		
		while (IsOperatorControl())
		{
			ultraSonic.Ping();
			SmartDashboard::PutBoolean("IR Sensor",irSensor.Get());
			SmartDashboard::PutBoolean("Limit Switch", !limitSwitch.Get ());
			SmartDashboard::PutNumber("Ultra Sonic", ultraSonic.GetRangeInches());
			SmartDashboard::PutBoolean("wubz",ultraSonic.IsRangeValid());
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

