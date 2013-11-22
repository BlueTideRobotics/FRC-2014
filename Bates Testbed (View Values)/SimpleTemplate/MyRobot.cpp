// Nicholas Currault, 11/8/13

#include "WPILib.h"
#include <Ultrasonic.h>

float potentiometerVoltCorrection(float volts)
{
	if (volts < 0)
	{
		return 0.0;
	}
	else if (volts > 4.894)
	{
		return 5.0;
	}
	else
	{
		return volts;
	}
}

float deadZone (float joystickVal)
{
	if(joystickVal<0.05&&joystickVal>-0.05)
	{
		return 0;
	}
	else
	{
		return joystickVal;
	}
}

class RobotDemo : public SimpleRobot
{	
	Joystick stick;
	DigitalInput irSensor;
	DigitalInput limitSwitch;

	Ultrasonic ultraSonic;
	
	AnalogChannel potentiometer;
	
	Servo servo;

public: 
	RobotDemo(void):
		stick(1),
		
		irSensor(5),
		limitSwitch(10),
		
		ultraSonic(1,3),
		
		potentiometer(1),
		
		servo(10)
	{
			//leave this here
	}
	
	void Autonomous(void)
	{
	}

	void OperatorControl(void)
	{
		float servoSetVal=0;
		float voltz;
		while (IsOperatorControl())
		{
			// Infrared sensor
			SmartDashboard::PutBoolean("IR Sensor",irSensor.Get());
			
			// Limit switch
			SmartDashboard::PutBoolean("Limit Switch", !limitSwitch.Get ());
			
			// Ultrasonic doesn't work
			ultraSonic.Ping();
			SmartDashboard::PutNumber("Ultra Sonic", ultraSonic.GetRangeInches());
			SmartDashboard::PutBoolean("wubz",ultraSonic.IsRangeValid());
			
			// Poteniometer is slightly inaccurate towards high and low end
			voltz = potentiometerVoltCorrection(potentiometer.GetAverageVoltage());
			SmartDashboard::PutNumber("Potentiometer Voltage", voltz);
			SmartDashboard::PutNumber("Actual Volts", potentiometer.GetAverageVoltage());
			SmartDashboard::PutNumber("Potentiometer Angle", (voltz/5)*315);
			
			// Servo
			if (stick.GetRawButton(1))
			{
				servoSetVal=0;
			}
			else if (stick.GetRawButton(2))
			{
				servoSetVal=1;
			}
			servoSetVal+=(deadZone(stick.GetX())/300.0);
			if(servoSetVal>1)
			{
				servoSetVal=1;
			}
			else if(servoSetVal<0)
			{
				servoSetVal=0;
			}
			servo.Set(servoSetVal);
			SmartDashboard::PutNumber("Servo",servoSetVal);
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

