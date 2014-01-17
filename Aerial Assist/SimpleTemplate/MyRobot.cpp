/*

frontLeft(14),
		backLeft(15),
		frontRight(11),
		backRight(10),
*/

// Sage Thorin Druch
// Douglas Nicholas Currault III
#include "WPILib.h"
//#include "includeFunctions.h"
#include "PID.h"
#include "Timer.h"
double Map(double value,double high, double low)
{
	return ((value+fabs(low))/((high+fabs(low))/2))-1;
}
double trueMap(double val,double valHigh,double valLow,double newHigh, double newLow)
{
	double midVal=((valHigh-valLow)/2)+valLow;
	double newMidVal=((newHigh-newLow)/2)+newLow;
	double ratio=(newHigh-newLow)/(valHigh-valLow);
	return (((val-midVal)*ratio)+newMidVal);
}
double inverseMap(double value, double high, double low)
{
	return(((value+1)*((high+fabs(low))/2)))+low;
}
void constrain(double *value,double highConstraint,double lowConstraint)
{
	if(*value>highConstraint)
		*value=highConstraint;
	if(*value<lowConstraint)
		*value=lowConstraint;
}
float deadZone(float value,float deadzone)
{
	if(value>deadzone||value<-deadzone)
		return value;
	else
		return 0.0;
}
int negative(double value)
{
	if(value<0.0)
		return -1;
	else
		return 1;
}

class RobotDemo : public SimpleRobot
{
	CANJaguar frontLeft;
	CANJaguar frontRight;
	CANJaguar backRight;
	CANJaguar backLeft;
	Joystick stick;
	RobotDrive myRobot; // robot drive system
	DriverStationLCD *dslcd;
	PID driveLeftPID;
	PID driveRightPID;
	DigitalOutput strobe1;
	DigitalOutput strobe2;
	Timer timer;
	Timer turnTime;
	Servo cameraPivotAxis,cameraTiltAxis;
	Joystick cameraStick;
	Solenoid shifterIn, shifterOut;
	
	CANJaguar ballGrab;
	
	CANJaguar ballLift;
	
	DigitalInput liftLowerLimit;
	
	Joystick PIDStick;
	PID gyroPID;
	Gyro gyro;
	
	Timer autonTimer;

public:
	RobotDemo(void):
		frontLeft(14),
		frontRight(11),
		backRight(10),
		backLeft(15),
		stick(1),
		myRobot(frontLeft, backLeft, frontRight, backRight),	// these must be initialized in the same order

		driveLeftPID(0,0,0,100,100),
		driveRightPID(0,0,0,100,100),
		strobe1(1),
		strobe2(2),
		timer(),
		cameraPivotAxis(2),
		cameraTiltAxis(3),
		cameraStick(2),
		shifterIn(5),
		shifterOut(4),
		
		ballGrab(13),
		
		ballLift(16), // May be 17? Idk
		
		liftLowerLimit(14),
		
		gyro(8),
		
		PIDStick(3),
		gyroPID(0,0,0,360,360)
	{
		timer.Start();
		/*backLeft.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		backRight.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		*/backRight.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		backLeft.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		backLeft.ConfigEncoderCodesPerRev(360);
		backRight.ConfigEncoderCodesPerRev(360);
		//shooter.ChangeControlMode(CANJaguar::kVoltage);
		dslcd=DriverStationLCD::GetInstance();
		myRobot.SetExpiration(0.1);
		//try to use autonomous by calling it from within here
		//stuff I care about
		gyro.Reset();
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		
		// Semi-sketchy way
		bool isHot = true;
		double distance = 0;
		double origPos = backLeft.GetPosition();
		
		double sketchyTimer = 0.0;
		
		bool turning = false;
		bool left = false;
		
		autonTimer.Start();
		
		while(IsAutonomous())
		{
			SmartDashboard::PutNumber("Sketchy Timer",sketchyTimer);
			SmartDashboard::PutNumber("Real timer", autonTimer.Get());
			SmartDashboard::PutNumber("Distance", distance);
			SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
			
			if (distance < 60) // Drive forward
			{
				myRobot.Drive(-0.5,0.0);
			}
			else
			{
				myRobot.Drive(0.0,0.0);
				turning = true;
				gyro.Reset();
			}
			
			if (turning)
			{
				if (fabs(gyro.GetAngle()) < 90)
				{
					if (left) 
					{
						myRobot.Drive(0.3,-1.0);
					}
					else
					{
						myRobot.Drive(0.3,1.0);
					}
				}
				else
				{
					turning = false;
					myRobot.Drive(0.0,0.0);
				}
			}
			
			Wait(0.005);
			
			// Update time and distance
			sketchyTimer = sketchyTimer + 0.005;
			distance = (50.5/3.462) * fabs(backLeft.GetPosition() - origPos);
		}
					
		
	}


	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		SmartDashboard::PutNumber("P",55);
		SmartDashboard::PutNumber("I",2000000);
		SmartDashboard::PutNumber("D",0.000);
		SmartDashboard::PutNumber("rightS",0);
		SmartDashboard::PutNumber("leftS",0);
		SmartDashboard::PutBoolean("Drive Fool!",true);
		SmartDashboard::PutNumber("rightServo",0.5);
		SmartDashboard::PutNumber("leftServo",0.5);
		double P,I,D,leftS,rightS,outputLeft,outputRight;
		bool drive=true;
		bool turnPressed=false;
		bool turnReleased=true;
		float cameraPivot=0.677;
		float cameraTilt=0.677;
		bool shiftPos=true;
		bool shiftPosDown=false;
		double liftMotorsSet = 0.5;
		
		// Initial pneumatic states
		shifterIn.Set(true);
		shifterOut.Set(false);
		
		//Stuff I care about
		SmartDashboard::PutNumber("RealP",55);
		SmartDashboard::PutNumber("RealI",2000000);
		SmartDashboard::PutNumber("RealD",0);
		SmartDashboard::PutNumber("RealE",0.1);
		
		double realP,realI,realD,realE;
		bool buttonHeld;
		while (IsOperatorControl())
		{	
			//Stuff I care about
			
			realP=SmartDashboard::GetNumber("RealP");
			realI=SmartDashboard::GetNumber("RealI");
			realD=SmartDashboard::GetNumber("RealD");
			realE=SmartDashboard::GetNumber("RealE");
			
			gyroPID.SetVal(RealP,RealI,RealD);
			if(stick.GetRawButton(2))
				gyro.Reset();
			
			if(stick.GetRawButton(1))
			{
				if(!buttonHeld)
					gyroPID.ResetI();
				buttonHeld=true;
				gyroPID.Enable();
				float gyroValSet=gyro.GetAngle()%360;
				if(gyroValSet>180)
					gyroValSet=gyro.GetAngle()+(360-gyroValSet);
				else
					gyroValSet=gyro.GetAngle()-gyroValSet;
				
				gyroPID.SetPID(realP,realI,realD);
				gyroPID.SetEpsilon(realE);
				gyroPID.SetMaxChangeSetpoint(0.01);
				
				gyroPID.SetSetpoint(gyroValSet);
				
				double output=gyroPID.GetOutput(gyro.GetAngle(),NULL,NULL,NULL);
				constrain(output,-1,1);
				SmartDashboard::PutNumber("PIDOutputGyro",output);
				drive=false;//this part applies to lower code
				myRobot.ArcadeDrive(output,0);
				
			}
			else
			{
				drive=true;//hat 
				buttonHeld=false;
				gyroPID.Disable();
			}
			
			
			//Stuff I don't care about
			// Ball lifter
			
			SmartDashboard::PutNumber("Throttle",stick.GetThrottle());
			SmartDashboard::PutNumber("Lifter", ballLift.Get());
			SmartDashboard::PutBoolean("Limit Switch",liftLowerLimit.Get());
			
			
			if (stick.GetRawButton(7))
			{
				liftMotorsSet = stick.GetThrottle();
			}
			else if (stick.GetRawButton(6))
			{
				liftMotorsSet = -stick.GetThrottle();
			}
			else
			{
				liftMotorsSet = 0.0;
			}
			
			if (liftLowerLimit.Get()&&!stick.GetRawButton(7))
			{
				liftMotorsSet = 0.0;
			}
			
			ballLiftL.Set(liftMotorsSet);
			ballLiftR.Set(-liftMotorsSet);
			
			
			
			// Ball grabber
			if (stick.GetRawButton(3))
			{
				ballGrab.Set(0.5);
			}
			else if (stick.GetRawButton(4))
			{
				ballGrab.Set(-0.5);
			}
			else
			{
				ballGrab.Set(0.0);
			}
			SmartDashboard::PutNumber("armz", ballGrab.Get());
			
			/*rightEncoder=fabs(backRight.GetSpeed())*-negative(backRight.Get());
						leftEncoder=backLeft.GetSpeed();*/
						/*positionRight=backRight.GetPosition();
						positionLeft=backLeft.GetPosition();*/
			
			
			if(stick.GetRawButton(2)&&!shiftPosDown)
			{
				shiftPosDown=true;
				shiftPos=!shiftPos;
				if(shiftPos)
				{
					shifterIn.Set(true);
					shifterOut.Set(false);
				}
				else
				{
					shifterOut.Set(true);
					shifterIn.Set(false);
				}
			}
			else if(!stick.GetRawButton(2))
			{
				shiftPosDown=false;
			}
			SmartDashboard::PutBoolean("shifters in", shiftPos);
			
			cameraPivot=cameraPivot+(deadZone(cameraStick.GetY(),0.07)/70);
			if(cameraPivot>1.0)cameraPivot=1.0;
			if(cameraPivot<0.0)cameraPivot=0.0;
			cameraTilt=cameraTilt-(deadZone(cameraStick.GetX(),0.12)/70);
			if(cameraTilt>1.0)cameraTilt=1.0;
			if(cameraTilt<0.0)cameraTilt=0.0;
			if(cameraStick.GetRawButton(4))
			{
				cameraTilt=0.28;
				cameraPivot=0.7;
			}
			if(cameraStick.GetRawButton(3))
			{
				cameraTilt=0.677;
				cameraPivot=0.677;
			}
			if(cameraStick.GetRawButton(5))
			{
				cameraTilt=1;
				cameraPivot=0.685;
			}
			
			SmartDashboard::PutNumber("cameraTilt", cameraTilt);
			SmartDashboard::PutNumber("cameraPivot", cameraPivot);
			cameraPivotAxis.Set(cameraPivot);
			cameraTiltAxis.Set(cameraTilt);

			if(stick.GetRawButton(1)&&drive&&!turnReleased)
			{
				rightS=backRight.GetPosition()+3;
				leftS=backLeft.GetPosition()-3;
				turnReleased=false;
				turnPressed=true;
				drive=false;
			}
			if(!stick.GetRawButton(1))
				turnReleased=true;
			if(stick.GetRawButton(1)&&!drive&&!turnReleased)
			{
				turnReleased=false;
				turnPressed=false;
				drive=true;
			}
			drive=SmartDashboard::GetBoolean("Drive Fool!");
			if(int((((int(timer.Get()*100))%100)/((stick.GetY()+0.1)*100))+0.5)==0)
			{
				strobe1.Set(1);
				strobe2.Set(1);
			}
			else
			{
				strobe1.Set(0);
				strobe2.Set(0);
			}
			if(drive)
			{
				myRobot.ArcadeDrive(-stick.GetY(),(stick.GetX()/1.5));
				driveLeftPID.Disable();
				driveRightPID.Disable();
			}
			/*else//PID shit
			{
				driveLeftPID.Enable();
				driveRightPID.Enable();
				P=SmartDashboard::GetNumber("P");
				I=SmartDashboard::GetNumber("I");
				D=SmartDashboard::GetNumber("D");
				if(!turnPressed)
				{
					leftS=SmartDashboard::GetNumber("leftS");
					rightS=SmartDashboard::GetNumber("rightS");
				}
				driveLeftPID.SetPID(P,I,D);
				driveRightPID.SetPID(P,I,D);
				//driveLeftPID.SetEpsilon(0.03);
				driveLeftPID.SetSetpoint(leftS);
				driveRightPID.SetSetpoint(rightS);
				outputLeft=driveLeftPID.GetOutput(-backLeft.GetPosition());
				outputRight=driveRightPID.GetOutput(backRight.GetPosition());
				frontLeft.Set(-outputLeft);
				backLeft.Set(-outputLeft);
				frontRight.Set(outputRight);
				backRight.Set(outputRight);
			}*/
			SmartDashboard::PutBoolean("drive",drive);
			SmartDashboard::PutNumber("OutLeft",outputLeft);
			SmartDashboard::PutNumber("right",backRight.GetPosition());
			SmartDashboard::PutNumber("left",-backLeft.GetPosition());
			dslcd->PrintfLine(DriverStationLCD::kUser_Line3,"%f||%f",frontRight.GetOutputVoltage(),frontLeft.GetOutputVoltage());
			Wait(0.003);				// wait for a motor update time
			dslcd->UpdateLCD();
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

