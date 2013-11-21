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
		float goodJoystick=0;
		float servoSetVal=0;
		while (IsOperatorControl())
		{
			
			ultraSonic.Ping();
			SmartDashboard::PutBoolean("IR Sensor",irSensor.Get());
			SmartDashboard::PutBoolean("Limit Switch", !limitSwitch.Get ());
			SmartDashboard::PutNumber("Ultra Sonic", ultraSonic.GetRangeInches());
			SmartDashboard::PutBoolean("wubz",ultraSonic.IsRangeValid());
			
			float voltz = potentiometerVoltCorrection(potentiometer.GetAverageVoltage());
			float angleDegrees = (voltz/5)*315;
			SmartDashboard::PutNumber("Potentiometer Voltage", voltz);
			SmartDashboard::PutNumber("Actual Volts", potentiometer.GetAverageVoltage());
			SmartDashboard::PutNumber("Potentiometer Angle", angleDegrees);
			SmartDashboard::PutNumber("joystickVal",stick.GetX());
			if(stick.GetX()<0.05&&stick.GetX()>-0.05)
				goodJoystick=0;
			else
				goodJoystick=stick.GetX();
			if (stick.GetRawButton(1))
			{
				servoSetVal=0;
			}
			else if (stick.GetRawButton(2))
			{
				servoSetVal=1;
			}
			servoSetVal+=(goodJoystick/300.0);
			if(servoSetVal>1)
				servoSetVal=1;
			if(servoSetVal<0)
				servoSetVal=0;
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

