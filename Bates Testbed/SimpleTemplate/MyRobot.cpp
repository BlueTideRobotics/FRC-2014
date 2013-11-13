// Nicholas Currault, 11/8/13

#include "WPILib.h"
#include "Solenoid.h"

class RobotDemo : public SimpleRobot
{
	Joystick stick;
	DigitalInput pressureInCylinder;
	Solenoid driveA;
	Solenoid driveB;
	Relay *pneumaRelay;

public:
	RobotDemo(void):
		stick(1),
		
		pressureInCylinder(1),
		driveA(1),
		driveB(8),

	{
		pneumaRelay = new Relay(1);//, Relay::kForwardOnly
		//leave this here
	}

	void Autonomous(void)
	{
	}

	void OperatorControl(void)
	{
		bool onePressed=false;
		bool twoPressed=false;
		bool relaySet=false;
		
		
		while (IsOperatorControl())
		{
			// Update Smart Dashboard values
			SmartDashboard::PutBoolean("onePressed",onePressed);
			SmartDashboard::PutBoolean("relaySet",relaySet);
			SmartDashboard::PutBoolean("Pressa",pressureInCylinder.Get());
			SmartDashboard::PutBoolean("A",driveA.Get());
			SmartDashboard::PutBoolean("B",driveB.Get());
			
			// Adjusts relaySet based on button 1
			if(stick.GetRawButton(1)&&!onePressed)
			{
				onePressed=true;
				relaySet=!relaySet;
			}
			else if(!stick.GetRawButton(1))
			{
				onePressed=false;
			}
			
			// Controls relay with regard to the 'relaySet' boolean and Pressa.
			if (relaySet&&!pressureInCylinder.Get())
			{
				pneumaRelay->Set(Relay::kForward);
			}
			else
			{
				pneumaRelay->Set(Relay::kOff);
			}
			
			if(stick.GetRawButton(4))
			{
				driveA.Set(true);
				driveB.Set(false);
			}
			else if(stick.GetRawButton(5))
			{
				driveA.Set(false);
				driveB.Set(true);
			}
			else if (stick.GetRawButton(2)&&!twoPressed) 
			{
				driveA.Set(!driveA.Get());
				driveB.Set(!driveB.Get());
				twoPressed=true;
			}
			else if (!stick.GetRawButton(2))
			{
				twoPressed=false;
			}
			
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

