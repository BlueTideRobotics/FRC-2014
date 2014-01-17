// Nicholas Currault, 11/8/13

#include "WPILib.h"
#include "Solenoid.h"

class RobotDemo : public SimpleRobot
{
	Joystick stick;
	
	Solenoid liftUp;
	Solenoid liftDown;
	Solenoid gripperOpen;
	Solenoid gripperClose;
	
	DigitalInput pressureInCylinder;
	Relay pneumaRelay;
public:
	RobotDemo(void):
		stick(1),
		
		liftUp(1),
		liftDown(3),
		gripperOpen(6),
		gripperClose(8),
		
		pressureInCylinder(1),
		
		pneumaRelay(8)
	{
		
	}

	void Autonomous(void)
	{
	}

	void OperatorControl(void)
	{
		/*
		 * Control Guide
		 * 
		 * Button 1: toggle relaySet
		 * Button 2: move lift down
		 * Button 3: move lift up
		 * Button 4: close gripper
		 * Button 5: open gripper
		 */
		
		bool onePressed=false;
		bool relaySet=false;
		
		// Initializing lift down
		/*
		liftUp.Set(false);
		liftDown.Set(true);
		
		//Initializing gripper open
		gripperOpen.Set(true);
		gripperClose.Set(false);
		*/
		
		while (IsOperatorControl())
		{
			// Update Smart Dashboard values
			SmartDashboard::PutBoolean("relaySet",relaySet);
			SmartDashboard::PutBoolean("Relay on?",pneumaRelay.Get());
			SmartDashboard::PutBoolean("Pressa",pressureInCylinder.Get());
			SmartDashboard::PutBoolean("Lift Up",liftUp.Get());
			SmartDashboard::PutBoolean("Lift Down",liftDown.Get());
			SmartDashboard::PutBoolean("Gripper Open",gripperOpen.Get());
			SmartDashboard::PutBoolean("Gripper Close",gripperClose.Get());
			
			// Adjust relaySet based on button 1
			if(stick.GetRawButton(1)&&!onePressed)
			{
				onePressed=true;
				relaySet=!relaySet;
			}
			else if(!stick.GetRawButton(1))
			{
				onePressed=false;
			}
			
			// Control relay using 'relaySet' and the pressure sensor.
			if (relaySet&&!pressureInCylinder.Get())
			{
				pneumaRelay.Set(Relay::kOn);
			}
			else
			{
				pneumaRelay.Set(Relay::kOff);
			}
			
			// Makes stuff move with solenoids
			/*
			if(stick.GetRawButton(4)) // Close
			{
				gripperClose.Set(true);
				gripperOpen.Set(false);
			}
			else if(stick.GetRawButton(5)) // Open
			{
				gripperClose.Set(false);
				gripperOpen.Set(true);
			}
			else if (stick.GetRawButton(2)) // Down
			{
				liftUp.Set(false);
				liftDown.Set(true);
			}
			else if (stick.GetRawButton(3)) // Up
			{
				liftUp.Set(true);
				liftDown.Set(false);
			}
			*/
			if (stick.GetRawButton(6))
			{
				liftUp.Set(true);
				liftDown.Set(true);
				gripperOpen.Set(false);
				gripperClose.Set(false);
			}
			else if (stick.GetRawButton(7))
			{
				liftUp.Set(false);
				liftDown.Set(false);
				gripperOpen.Set(true);
				gripperClose.Set(true);
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

