#include "WPILib.h"

using std::string;

class Angel: public IterativeRobot
{
	private:

	//Drivebase option select
	SendableChooser *driveChooser;
	const string tankDrive = "Tank Drive";
	const string arcadeDrive = "Arcade Drive";
	const string climber = "Climber";
	string driveSelected;

	//Drive Motors
	CANTalon *leftCim, *leftMini, *rightCim, *rightMini;
	//Shooter Motors
	CANTalon *intake1, *intake2, *shooter, *intakeArm, *winch;

	float shooterError, shooterTarget, shooterSpeed, pOut, iOut, output;bool isAtSpeed = false;

	//Controllers
	Joystick *driver, *oper;

	//Solenoids
	Solenoid *gripA, *gripB;
	Solenoid *liftA, *liftB;

	//Banner Sensor for ball detection
	DigitalInput *banner;

	//NetworkTables and Vision variables
	std::shared_ptr<NetworkTable> table;
	float cogArea = 0;
	float cogx = 0;
	float cogy = 0;
	float cumcogPIDError = 0;
	float cogPIDError = 0;

	//x PID
	float cogxP = 0.09;
	float cogxI = 0.05;//.05
	float cogxTar = 180;

	//normPID/Y
	float PIDError = 0;
	float cumPIDError = 0;

	//shooting state machine
	enum shootState
	{
		intake,// Move around and accept a ball
		// Starts 2.5 sec after ball is fired. Ends if ball hits banner sensor
		reving,// Spin firing talon until its fast enough to throw a ball into a goal
		// Starts after ball hits banner, ends if aborted or at 200000 speed
		firing// Ball is sent to the firing talon to fly through the air
	// Starts when reving is at full speed and ends after 2.5 seconds
	};
	shootState currShootState;

	Timer *ResetFiringState;

	// Printf intel
	Timer *ElapsedRuntime;// Use in printf to tell how long the code has been running

	//livestream
	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;

	void RobotInit()
	{

		//Initializing chooser and options
		driveChooser = new SendableChooser();
		driveChooser->AddDefault(arcadeDrive, (void*) &arcadeDrive);
		driveChooser->AddObject(tankDrive, (void*) &tankDrive);
		driveChooser->AddObject(climber, (void*) &climber);

		driver = new Joystick(0);
		oper = new Joystick(1);

		leftCim = new CANTalon(10);
		leftMini = new CANTalon(14);
		rightCim = new CANTalon(9);
		rightMini = new CANTalon(11);

		intake1 = new CANTalon(1);
		intake2 = new CANTalon(0);
		shooter = new CANTalon(4);

		intakeArm = new CANTalon(13);
		winch = new CANTalon(12);

		gripA = new Solenoid(4);//set(1)=fire
		gripB = new Solenoid(5);//set(1)=lock
		liftA = new Solenoid(0);//set(1)=drop
		liftB = new Solenoid(2);//set(1)=pickup

		//NetworkTables
		table = NetworkTable::GetTable("SmartDashboard");

		banner = new DigitalInput(0);// 0 means go; 1 means stop

		// Shooting states
		ResetFiringState = new Timer();
		ElapsedRuntime = new Timer();

		// create an image
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam2", IMAQdxCameraControlModeController, &session);
		imaqError = IMAQdxConfigureGrab(session);
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		currShootState = intake;
		ElapsedRuntime->Reset();
		ElapsedRuntime->Start();

		IMAQdxStartAcquisition(session);
	}

	void TeleopPeriodic()
	{
		printEverything();

		Drivebase();
		Shooter();
		Gripper();

		// acquire images
		IMAQdxStartAcquisition(session);
		IMAQdxGrab(session, frame, true, NULL);
		imaqDrawShapeOnImage(frame, frame, { 200, 200, 100, 1 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_RECT, 0.0f);
		CameraServer::GetInstance()->SetImage(frame);

	}

	void TestPeriodic()
	{

	}

	//Function for operating drivebase. Allows for tank/Arcade option select
	void Drivebase()
	{

		if (driver->GetRawButton(7))
		{
			liftA->Set(1);
			liftB->Set(0);
		}
		if (driver->GetRawButton(8))
		{
			liftA->Set(0);
			liftB->Set(1);
		}

		//x rotation alignment via vision
		if (driver->GetRawButton(14))//Center square board
		{
			visionXAlign();
		}

		//Places chooser onto SmartDashboard and records option select
		SmartDashboard::PutData("Drive Modes", driveChooser);
		driveSelected = *((string*) driveChooser->GetSelected());

		//Defaults to arcade drive
		if (driveSelected == arcadeDrive)
		{
			double speed = deadband(-driver->GetY());
			double turn = deadband(driver->GetZ());

			setLeftSpeed(speed + turn);
			setRightSpeed(-speed + turn);
		}
		else
			if (driveSelected == tankDrive)
			{
				double leftSpeed = deadband(-driver->GetY());
				double rightSpeed = deadband(driver->GetRawAxis(5));

				setLeftSpeed(leftSpeed);
				setRightSpeed(rightSpeed);

			}
			else
			{
				double speed = deadband(-driver->GetY());
				setSpeed(speed);
			}

	}

	//Function for operating shooter
	void Shooter()
	{
		//left trigger
		double intakeForward = (int) ((oper->GetRawAxis(3) + 1) / 2);
		//left bumper
		double intakeReverse = -oper->GetRawButton(5);
		//left stick
		double intakeArmSpeed = deadband(oper->GetY());
		//square
		double shooterForward = -oper->GetRawButton(1);
		//X
		double shooterReverse = oper->GetRawButton(2);
		//Right Trigger
		double winchUp = -(int) ((oper->GetRawAxis(4) + 1) / 2);
		double winchDown = oper->GetRawButton(6);

		bool ballHit = banner->Get();

		/* This is an enum state machine to go between intake, reving, and firing
		 Here's how the states work:

		 intake,  Move around and accept a ball
		 Starts 2.5 sec after ball is fired. Ends if ball hits banner sensor
		 reving,  Spin firing talon until its fast enough to throw a ball into a goal
		 Starts after ball hits banner, ends if aborted or at 20000 speed
		 firing,  Ball is sent to the firing talon to fly through the air
		 Starts when reving is at full speed and ends after 2.5 seconds
		 Note, the transition between firing and intake is jenky
		 It currently is button operated, and later might be a 2nd banner*/
		switch (currShootState)
		{
			case intake:
				shooter->Set(0);
				isAtSpeed = false;

				// In this mode, the robot is waiting to obtain a ball
				if (intakeForward != 0)
				{
					setIntake(intakeForward);
					// If the intake button is pressed, pull a ball in
				}
				else
				{
					setIntake(intakeReverse);
					// If the regurgitation button is pressed, spit the ball out
				}
				if (ballHit == 1)
				{
					// switch states because we've fully loaded a ball
					setIntake(0);
					currShootState = reving;
					printf("Transition to reving at time %f\n", ElapsedRuntime->Get());
				}
				break;
			case reving:
				// In this mode, the robot is waiting for the shooter to reach 20K speed
				if (intakeForward == 0)
				{
					setIntake(intakeReverse);
					currShootState = intake;
					printf("Regurgitation to intake at time %f\n", ElapsedRuntime->Get());
					// Regurgitate, and go to intake because we're spitting out the ball
				}
				if (oper->GetRawButton(4))
				{
					shooterPID();
					// This runs the rev up process, PID'ing the shooter to 20000.
				}
				else
					if (shooterForward != 0)
					{
						// just like intakeForward... but for the shooter
						shooter->Set(shooterForward);//.80 - .88 for parabolic
					}
					else//reverse shooter
					{
						shooter->Set(shooterReverse * .2);
					}
				if (isAtSpeed)
				{
					// switch states because we are completely reved up
					currShootState = firing;
					printf("Transition to firing at time %f\n", ElapsedRuntime->Get());

					ResetFiringState->Reset();
					ResetFiringState->Start();
				}
				break;
			case firing:
				if (ballHit == 1)
				{
					// If the ball is still at the banner sensor, push it into the shooter
					intake1->Set(intakeForward);
				}
				if (ResetFiringState->Get() >= 1)
				{
					// "patch" - this is how you switch back to intake, at least for now
					// TODO: fix the long standing bug and make the state switcher smarter
					// (timer, second banner, flags, etc.)
					currShootState = intake;
					printf("Transition to intake at time %f\n", ElapsedRuntime->Get());
				}
				break;
		}
		//reset winch when its all the way down
		if (oper->GetRawButton(13))
		{
			winch->SetEncPosition(0);
		}
		//move winch up
		if (driver->GetRawButton(5))
		{
			visionYAlign();
		}
		else
		{
			if (winchUp != 0)
			{
				winch->Set(winchUp * .75);
			}
			else//move winch down
			{
				winch->Set(winchDown * .75);
			}
		}

		//move intake arm up or down
		intakeArm->Set(0.5 * intakeArmSpeed);

		if (oper->GetPOV() == 0)
		{
			winch->Set(normPID(0, winch->GetEncPosition(), .0057, .003));
		}
		if (oper->GetPOV() == 90)
		{
			winch->Set(normPID(270, winch->GetEncPosition(), .0057, .003));
		}
		if (oper->GetPOV() == 180)
		{
			winch->Set(normPID(425, winch->GetEncPosition(), .0057, .003));
		}
		if (oper->GetPOV() == 270)
		{
			winch->Set(normPID(590, winch->GetEncPosition(), .0057, .003));
		}

		//upload values to SmartDashboard for display
		SmartDashboard::PutNumber("Shooter Speed", -shooter->GetSpeed());
		SmartDashboard::PutBoolean("Ball in place", ballHit);
		SmartDashboard::PutNumber("Hood Position", winch->GetEncPosition());
	}

	//Sets speeds of both drive motors to for/back
	void setSpeed(double speed)
	{
		leftCim->Set(speed);
		leftMini->Set(speed);
		rightCim->Set(-speed);
		rightMini->Set(-speed);
	}

	//Sets speeds of left side drive motors
	void setLeftSpeed(double speed)
	{
		leftCim->Set(speed);
		leftMini->Set(speed);
	}

	//Sets speeds of right side drive motors
	void setRightSpeed(double speed)
	{
		rightCim->Set(speed);
		rightMini->Set(speed);
	}

	void setIntake(double speed)
	{
		intake1->Set(speed);
		intake2->Set(-speed * .75);
	}

	//Solenoids for firing/locking the climbing mechanism
	void Gripper()
	{
		if (oper->GetRawButton(3))//releases climber to attach to bar
		{
			gripA->Set(1);
			gripB->Set(0);
		}
		if (oper->GetRawButton(4))//locks climber back in place after manually being dragged down
		{
			gripA->Set(0);
			gripB->Set(1);
		}
	}

	//Removes idle stick input at 7%
	double deadband(double input)
	{

		if (absVal(input) < .07)
		{
			return 0;
		}
		return input;
	}

	//C++ absolute value function sucks
	double absVal(float input)
	{
		if (input < 0)
			return -input;
		return input;
	}

	//PID for anything
	float normPID(float myTar, float myPos, float myP, float myI)
	{
		PIDError = myTar - myPos;
		cumPIDError = PIDError;
		float PIDPout = PIDError * myP;
		float PIDIout = PIDError * myI;
		float PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDError) < 5)
		{
			PIDSpeed = 0;
		}
		printf("YSpeed: %f\n", PIDSpeed);
		printf("YError: %f\n", PIDError);
		return (PIDSpeed);
	}

	void shooterPID()
	{
		shooterSpeed = -shooter->GetSpeed();
		shooterTarget = 20000;
		shooterError = shooterTarget - shooterSpeed;

		float errorMargin = 1000;

		if (absVal(shooterError) <= errorMargin)
			isAtSpeed = true;
		else
			isAtSpeed = false;

		float kP = 0.0005;
		float kI = kP / 600;

		pOut = kP * shooterError;
		iOut += kI * shooterError;

		shooter->Set(-(pOut + iOut));

	}
	void printEverything()
	{
		//Vision Printf
		//printf("Area: %f\nX: %f\nY: %f\n", table->GetNumber("COG_AREA", 217), table->GetNumber("COG_X", 217), table->GetNumber("COG_Y", 217));
		//printf("Distance: %f\n", visionCalcDist());
		//visionPID(cogxTar, cogx, cogxP, cogxI);//prints speed of rotation within function

		//winch, shooter, and banner sensor printf
		//printf("winch: %i shooter: %f voltage: %f\n", winch->GetEncPosition(),shooter->GetSpeed(),shooter->GetOutputVoltage());
		//printf("shooter: %i\n", shooter->GetEncPosition());
		//printf("banner: %i\n", banner->Get());
		//printf("Left cim: %f left mini: %f right cim: %f right mini: %f\n", leftCim->GetOutputVoltage(),
		//leftMini->GetOutputVoltage(), rightCim->GetOutputVoltage(), rightMini->GetOutputVoltage());
		//printf("%d\n", (oper->GetRawButton(4) && absVal(shooterError) <= 1500));
		//printf("intake: %d\n", (int) ((oper->GetRawAxis(3) + 1) / 2));
		//printf("ShootError: %f\n", shooterError);
		//printf("isAtSpeed: %d\n", isAtSpeed);
	}

	//PID for vision component
	float visionPID(float cogTar, float cogPos, float cogP, float cogI)
	{
		cogPIDError = cogTar - cogPos;
		if (absVal(cogPIDError) < 20)
		{
			cogP = .12;
			cogI = .05;
		}
		cumcogPIDError += cogPIDError;
		float cogPIDPout = cogPIDError * cogP;
		float cogPIDIout = cogPIDError * cogI;
		float cogPIDSpeed = (cogPIDPout + cogPIDIout) / 6;

		if (absVal(cogPIDError) < 3)
		{
			cogPIDSpeed = 0;
		}
		printf("Error: %f\n", cogPIDError);
		return (cogPIDSpeed);
	}

	//x axis vision alignment: rotate bot
	void visionXAlign()
	{
		cogx = table->GetNumber("COG_X", 217);
		leftCim->Set(-visionPID(cogxTar, cogx, cogxP, cogxI));
		leftMini->Set(-visionPID(cogxTar, cogx, cogxP, cogxI));
		rightMini->Set(-visionPID(cogxTar, cogx, cogxP, cogxI));
		rightCim->Set(-visionPID(cogxTar, cogx, cogxP, cogxI));
		visionPID(cogxTar, cogx, cogxP, cogxI);
	}

	//y axis vision alignment
	void visionYAlign()
	{
		cogy = table->GetNumber("COG_Y", 217);
		if (cogy < 60)//backed up to defenses at courtyard
		{
			winch->Set(normPID(270, winch->GetEncPosition(), .005, .002));
		}
		else
			if (cogy < 80)
			{
				winch->Set(normPID(320, winch->GetEncPosition(), .005, .002));
			}
			else
				if (cogy < 100)
				{
					winch->Set(normPID(360, winch->GetEncPosition(), .005, .002));
				}
				else
					if (cogy < 120)
					{
						winch->Set(normPID(390, winch->GetEncPosition(), .005, .002));
					}
					else
						if (cogy < 140)
						{
							winch->Set(normPID(425, winch->GetEncPosition(), .005, .002));
						}
						else
							if (cogy < 160)
							{
								winch->Set(normPID(470, winch->GetEncPosition(), .005, .002));
							}
							else
								if (cogy < 180)
								{
									winch->Set(normPID(525, winch->GetEncPosition(), .005, .002));
								}
								else
									if (cogy < 200)//on lip of tower's base
									{
										winch->Set(normPID(595, winch->GetEncPosition(), .005, .002));
									}
	}
}
;

START_ROBOT_CLASS(Angel)
