#include "WPILib.h"
#include <string>
using std::string;

class BunnyOmni: public IterativeRobot {
private:
	//Drivebase option select
	SendableChooser *driveChooser;
	const string tankDrive = "Tank Drive";
	const string arcadeDrive = "Arcade Drive";
	const string climber = "Climber";
	string driveSelected;

	//Spy Bot Select
	SendableChooser *spyBotChooser;
	const string yesSpyBot = "Yes";
	const string noSpyBot = "No";
	string spyBotSelected;

	//Defense Option Select
	SendableChooser *defenseChooser;
	const string moat = "Moat";
	const string ramparts = "Ramparts";
	const string sallyPort = "Sally Port";
	const string drawbridge = "Drawbridge";
	const string rockWall = "Rock Wall";
	const string roughTerrain = "Rough Terrain";
	string defenseSelected;

	//Position Option Select
	SendableChooser *positionChooser;
	const string firstPosition = "First Position";
	const string secondPosition = "Second Position";
	const string thirdPosition = "Third Position";
	const string fourthPosition = "Fourth Position";
	string positionSelected;

	double intakeArmSpeed = 0;
	//Drive Motors
	CANTalon *leftFrontCim, *leftBackCim, *rightFrontCim, *rightBackCim;

	//Shooter Motors
	CANTalon *intakeInner, *intakeOuter, *shooter1, *shooter2, *arm, *hood;

	//Shooter PID
	float shooterError, shooterTarget, shooterSpeed, shooterAvg, pOut, iOut,
			output;bool isAtSpeed = false, ohShoot = false, stateShoot = false;

	double throttle = 0;
	double armkP, armkI;

	//Controllers
	Joystick *driver, *oper;

	//Solenoids
	DoubleSolenoid *rightPTO, *leftPTO;
	Solenoid *fingers;
	Solenoid *climberAngle;
	Solenoid *climbLock;

	//Gyros
	AnalogGyro *horzGyro, *vertGyro;

	bool climbIsReleased = false; //climber is locked
	bool ptoEngaged = false;bool angleSet = false;bool pidArm = false;

	//Banner Sensor for ball detection
	DigitalInput *bannerInner, *zeroHood;bool ballHit = false;

	//NetworkTables and Vision variables
	std::shared_ptr<NetworkTable> table;
	float cogx = 0;
	float cogy = 0;
	float cumcogPIDError = 0;
	float cogPIDError = 0;

	//x PID
	float cogxP = 0.005;
	float cogxI = 0.0009; //.05
	float cogxMult = 1;
	float cogxTar = 160;
	float tempcogxP = 0;
	float tempcogxI = 0;

	//normPID/Y
	float PIDError = 0;
	float cumPIDError = 0;

	//shooting state machine
	enum shootState {
		intake, // Move around and accept a ball
		// Starts 2.5 sec after ball is fired. Ends if ball hits banner sensor
		reving,	// Spin firing talon until its fast enough to throw a ball into a goal
		// Starts after ball hits banner, ends if aborted or at 200000 speed
		firing		// Ball is sent to the firing talon to fly through the air
	// Starts when reving is at full speed and ends after 2.5 seconds
	};
	shootState currShootState;

	enum climbState {
		standby, angle, release, drive
	} currClimbState;

	Timer *resetFiringState;
	Timer *runtime;

	//Live Stream
	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;

	//Auton Backup Timers
	Timer *approachTimer;

	//One Ball Auton
	enum OneBallNoSpy {
		cross, align, shoot
	} OneBallNoSpy;

	enum OneBallSpy {
		shootBall, driveToDefense, crossDefense
	} OneBallSpy;

	enum moat {
		moatDriveOver, moatRealign
	} Moat;

	enum ramparts {
		rampartsDriveOver, rampartsRealign
	} Ramparts;

	enum sallyPort {
		sallyDriveThrough, sallyPortRealign
	} SallyPort;

	enum drawbridge {
		drawThrough, drawBridgeRealign

	} Drawbridge;

	enum rockWall {
		rockDriveForward, rockWallRealign
	} RockWall;

	enum roughTerrain {
		roughDriveForward, roughTerrainRealign
	} RoughTerrain;

	//Auton Variables
	float leftFrontCimEncTar = 217;
	float leftBackCimEncTar = 217;
	float rightFrontCimEncTar = 217;
	float rightBackCimEncTar = 217;

	float leftFrontCimEncPos;
	float leftBackCimEncPos;
	float rightFrontCimEncPos;
	float rightBackCimEncPos;

	int defense, position;bool autonTimerFlag = true;

	enum autonShootState {
		autonReving, // Spin firing talon until its fast enough to throw a ball into a goal
		// Starts after ball hits banner, ends if aborted or at 200000 speed
		autonFiring	// Ball is sent to the firing talon to fly through the air
	// Starts when reving is at full speed and ends after 2.5 seconds
	} autonShootState;

	void RobotInit() {
		//Initializing chooser and options
		driveChooser = new SendableChooser();
		driveChooser->AddDefault(arcadeDrive, (void*) &arcadeDrive);
		driveChooser->AddObject(tankDrive, (void*) &tankDrive);
		driveChooser->AddObject(climber, (void*) &climber);

		spyBotChooser = new SendableChooser();
		spyBotChooser->AddDefault(noSpyBot, (void*) &noSpyBot);
		spyBotChooser->AddObject(yesSpyBot, (void*) &yesSpyBot);

		defenseChooser = new SendableChooser();
		defenseChooser->AddObject(moat, (void*) &moat);
		defenseChooser->AddObject(ramparts, (void*) &ramparts);
		defenseChooser->AddObject(sallyPort, (void*) &sallyPort);
		defenseChooser->AddObject(drawbridge, (void*) &drawbridge);
		defenseChooser->AddObject(rockWall, (void*) &rockWall);
		defenseChooser->AddDefault(roughTerrain, (void*) &roughTerrain);

		positionChooser = new SendableChooser();
		positionChooser->AddDefault(firstPosition, (void*) &firstPosition);
		positionChooser->AddObject(secondPosition, (void*) &secondPosition);
		positionChooser->AddObject(thirdPosition, (void*) &thirdPosition);
		positionChooser->AddObject(fourthPosition, (void*) &fourthPosition);

		//Gyros
		horzGyro = new AnalogGyro(0);
		//vertGyro = new AnalogGyro(1); not currently on bot

		//Timers
		approachTimer = new Timer();

		//Initializing joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);

		//Drive motors
		leftFrontCim = new CANTalon(14);
		leftBackCim = new CANTalon(15);	//2
		rightFrontCim = new CANTalon(12);
		rightBackCim = new CANTalon(13);

		//Intake, shooter motors
		intakeInner = new CANTalon(3);
		intakeOuter = new CANTalon(6);
		shooter1 = new CANTalon(0);
		shooter2 = new CANTalon(1);	//1
		arm = new CANTalon(2);	//2
		hood = new CANTalon(7);

		//Initializing Solenoids
		rightPTO = new DoubleSolenoid(0, 1);
		leftPTO = new DoubleSolenoid(2, 3);
		climberAngle = new Solenoid(6);
		climbLock = new Solenoid(7);

		//NetworkTables
		table = NetworkTable::GetTable("SmartDashboard");

		//Ball recog sensors
		bannerInner = new DigitalInput(0);	// 0 means go; 1 means stop
		zeroHood = new DigitalInput(1);	// 1 means not pressed; 0 means pressed

		// Shooting states
		resetFiringState = new Timer();
		runtime = new Timer();

		// create an image
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController,
				&session);
		imaqError = IMAQdxConfigureGrab(session);

		SmartDash();
	}

	void AutonomousInit() {
		SmartDash();
		leftFrontCim->SetEncPosition(0);
		leftBackCim->SetEncPosition(0);
		rightFrontCim->SetEncPosition(0);
		rightBackCim->SetEncPosition(0);

		leftFrontCimEncPos = leftFrontCim->GetEncPosition();
		leftBackCimEncPos = leftBackCim->GetEncPosition();
		rightFrontCimEncPos = rightFrontCim->GetEncPosition();
		rightBackCimEncPos = rightBackCim->GetEncPosition();

		approachTimer->Reset();
		autonTimerFlag = true;
		autonShootState = autonReving;

		defenseChoosing();
		positionChoosing();
		spyBotChoosing();
	}

	void AutonomousPeriodic() {
		//One Ball Auton when we are NOT the Spy Bot
		SmartDash();
		switch (OneBallNoSpy) {
		case cross:
			pidArm = true;
			ballHit = bannerInner->Get();

			if (ballHit == 1) {

				setIntake(0);
				armPID(0);
				pidArm = false;
				crossingDefense();

					if (approachTimer->Get() >= 4.3) { //Rock Wall in the 3rd Position (Middle). This number is the final time in rock wall +.1 if changes are made to this time
						OneBallNoSpy = align;
					}

			} else {
				setIntake(1);
			}
			break;

		case align:
			positionDefense();

			break;

		case shoot:
			visionXAlign();
			visionYAlign();

			switch (autonShootState) {
			case autonReving:
				// In this mode, the robot is waiting for the shooter to reach 20K speed
				printf("reving\n");

				shooterPID();
				// This runs the rev up process, PID'ing the shooter to 20000.
				if (isAtSpeed)	//isAtSpeed
				{
					printf("WE MADE IT\n");
					// switch states because we are completely reved up
					resetFiringState->Reset();
					resetFiringState->Start();
					autonShootState = autonFiring;

				}
				break;
			case autonFiring:
				printf("firing");

				// If the ball is still at the banner sensor, push it into the shooter
				setIntake(1);
				//intakeInner->Set(1);//fix this shit TODO:

				if (resetFiringState->Get() >= 1) {
					setIntake(0);
					setShooter(0);
					iOut = 0;
				}
				break;
			}

			break;
		}

		//One Ball Auton Where We ARE Spy Bot
		switch (OneBallSpy) {
		case shootBall:
			//				switch (autonShootState)
			//				{
			//					case autonReving:
			//						// In this mode, the robot is waiting for the shooter to reach 20K speed
			//						printf("reving\n");
			//
			//							shooterPID();
			//							// This runs the rev up process, PID'ing the shooter to 20000.
			//						if (isAtSpeed)//isAtSpeed
			//						{
			//							// switch states because we are completely reved up
			//							currShootState = firing;
			//
			//							resetFiringState->Reset();
			//							resetFiringState->Start();
			//						}
			//						break;
			//					case autonFiring:
			//						printf("firing");
			//						if (ballHit == 1)
			//						{
			//							// If the ball is still at the banner sensor, push it into the shooter
			//							setIntake(1);
			//							//intakeInner->Set(1);//fix this shit TODO:
			//						}
			//						if (resetFiringState->Get() >= 1)
			//						{
			//							currShootState = intake;
			//							iOut = 0;
			//						}
			//						break;
			//				}
			break;

		case driveToDefense:
			break;

		case crossDefense:
			crossingDefense();
			break;
		}
	}

	void TeleopInit() {
		currShootState = intake;
		currClimbState = standby;

		//livestream
		IMAQdxStartAcquisition(session);

	}

	void TeleopPeriodic() {

		shooterSpeed = shooter1->GetSpeed();
		SmartDash();
		if (driver->GetRawButton(14))	//Center square board
				{
			visionXAlign();
		} else {
			Drivebase();
		}

		Shooter();
		Arm();
		Hood();
		Climber();
		LiveStream();

	}

	void TestPeriodic() {
//		printf("banner: %i\n", bannerInner->Get());
//		printf("ShootError: %f\n", shooterError);
//		printf("isAtSpeed: %d\n", isAtSpeed);
//		printf("shooter1: %i\n", shooter1->GetSpeed());
//		printf("shooter2: %i\n", shooter2->GetSpeed());
//		printf("zeroHood: %i\n",zeroHood->Get());
	}
	void SmartDash() {
		//Places chooser onto SmartDashboard and records option select
		SmartDashboard::PutData("Drive Modes", driveChooser);
		driveSelected = *((string*) driveChooser->GetSelected());

		SmartDashboard::PutData("What defense are we crossing?",
				defenseChooser);
		defenseSelected = *((string*) defenseChooser->GetSelected());

		SmartDashboard::PutData("What position are we in?", positionChooser);
		positionSelected = *((string*) positionChooser->GetSelected());

		SmartDashboard::PutData("Are we the Spy Bot?", spyBotChooser);
		spyBotSelected = *((string*) spyBotChooser->GetSelected());
		SmartDashboard::PutNumber("One Ball No Spy", OneBallNoSpy);
		SmartDashboard::PutNumber("Auton Shoot State", autonShootState);
		SmartDashboard::PutNumber("Position", position);
		SmartDashboard::PutBoolean("isAtSpeed", isAtSpeed);
		SmartDashboard::PutNumber("Shooter Speed", shooter1->GetSpeed());
		SmartDashboard::PutNumber("Timer", approachTimer->Get());
		SmartDashboard::PutNumber("Hood Speed", hood->GetSpeed());

		//upload values to SmartDashboard for display
//		SmartDashboard::PutNumber("Shooter Speed", shooterSpeed);
		SmartDashboard::PutBoolean("Ball in place", ballHit);
		SmartDashboard::PutNumber("Hood Position", hood->GetEncPosition());
		cogx = table->GetNumber("COG_X", 217);
		std::string cogxVal = std::to_string(cogx);

//		SmartDashboard::PutString("DB/String 0", cogxVal);
		SmartDashboard::PutNumber("Arm position", arm->GetEncPosition());
		//tempcogxP = SmartDashboard::GetNumber("DB/Slider 0", 0);
		//tempcogxI = SmartDashboard::GetNumber("DB/Slider 1", 0);
//		cogxMult = SmartDashboard::GetNumber("DB/Slider 2", 1);
	}

	//Live stream
	void LiveStream() {
		// acquire images
		IMAQdxStartAcquisition(session);
		IMAQdxGrab(session, frame, true, NULL);
		//imaqDrawShapeOnImage(frame, frame, { 200, 200, 100, 1 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_RECT, 0.0f);
		CameraServer::GetInstance()->SetImage(frame);
	}
	//Function for operating drivebase. Allows for tank/Arcade option select
	void Drivebase() {
		//Defaults to arcade drive
		if (driveSelected == arcadeDrive) {
			double speed = deadband(-driver->GetY());
			double turn = deadband(driver->GetZ());

			setLeftSpeed(speed + turn);
			setRightSpeed(-speed + turn);
		} else if (driveSelected == tankDrive) {
			double leftSpeed = deadband(-driver->GetY());
			double rightSpeed = deadband(driver->GetRawAxis(5));

			setLeftSpeed(leftSpeed);
			setRightSpeed(rightSpeed);

		} else	//climber
		{
			double speed = deadband(-driver->GetY());
			setSpeed(speed);
		}
	}
	//Function for operating shooter
	void Shooter() {
		ballHit = bannerInner->Get();

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

		switch (currShootState) {

		case intake:
			printf("intake\n");
			setShooter(0);
			isAtSpeed = false;

			// In this mode, the robot is waiting to obtain a ball
			if (oper->GetRawButton(5)) {
				setIntake(-1);

				// If the regurgitation button is pressed, pull a ball in
			} else if (oper->GetRawButton(7)) {
				setIntake(1);
				pidArm = true;
				armPID(-1500);
				// If the intake button is pressed, spit the ball out
			} else {
				setIntake(0);
				pidArm = false;
			}

			if (ballHit == 1) {
				// switch states because we've fully loaded a ball
				setIntake(0);
				currShootState = reving;
			}

			break;
		case reving:
			// In this mode, the robot is waiting for the shooter to reach 20K speed
			printf("reving\n");
			if (oper->GetRawButton(7)) {
				armPID(0);
				pidArm = true;
			} else {
				pidArm = false;
			}

			if (oper->GetRawButton(5)) {
				setIntake(-1);
				currShootState = intake;
				// Regurgitate, and go to intake because we're spitting out the ball
			}
			if (oper->GetRawButton(4)) {
				shooterPID();
				// This runs the rev up process, PID'ing the shooter to 20000.
			} else {
				setShooter(0);
			}

			if (isAtSpeed)	//isAtSpeed
			{
				// switch states because we are completely reved up
				currShootState = firing;

				resetFiringState->Reset();
				resetFiringState->Start();
			}
			break;
		case firing:
			printf("firing");
			if (ballHit == 1) {
				// If the ball is still at the banner sensor, push it into the shooter
				setIntake(1);
				//intakeInner->Set(1);//fix this shit TODO:
			}
			if (resetFiringState->Get() >= 1) {
				currShootState = intake;
				iOut = 0;
			}
			break;
		}

	}

	void Arm() {
		//left stick
		intakeArmSpeed = deadband(oper->GetY());
		printf("armSpeed %f current %f\n", intakeArmSpeed,
				arm->GetOutputCurrent());

		if (oper->GetRawButton(13)) {
			arm->SetEncPosition(0);

		}

		//move intake arm up or down
		if (intakeArmSpeed < 0) {
			arm->Set(intakeArmSpeed);
			pidArm = false;

		} else if (intakeArmSpeed > 0) {
			arm->Set(intakeArmSpeed * 0.5);
			pidArm = false;
		} else if (!pidArm) {
			arm->Set(0);
		}
	}

	void Hood() {

		//Right Trigger
		double hoodUp = -(int) ((oper->GetRawAxis(4) + 1) / 2);
		double hoodDown = oper->GetRawButton(6);

		//reset hood when its all the way down
		if (zeroHood->Get() == 0) {
			if (oper->GetRawButton(6) == 1) {
				hood->Set(0);
			}
			hood->SetEncPosition(0);
			hood->Set(normPID(50, hood->GetEncPosition(), .0057, .003));
		} else if (driver->GetRawButton(5))	//move hood up
				{
			visionYAlign();
		} else {

			if (hoodUp != 0) {
				hood->Set(hoodUp * .75);
			} else	//move hood down
			{
				hood->Set(hoodDown * .75);
			}
		}

		if (oper->GetPOV() == 0) {
			hood->Set(normPID(0, hood->GetEncPosition(), .0057, .003));
		}
		if (oper->GetPOV() == 90) {
			hood->Set(normPID(610, hood->GetEncPosition(), .0057, .003));//610
		}

		if (oper->GetPOV() == 180) {
			hood->Set(normPID(690, hood->GetEncPosition(), .0057, .003));//690
		}

		if (oper->GetPOV() == 270) {
			hood->Set(normPID(1000, hood->GetEncPosition(), .0057, .003));//1000
		}

	}

//Sets speeds of both drive motors to for/back
	void setSpeed(double speed) {
		leftFrontCim->Set(speed);
		leftBackCim->Set(speed);
		rightFrontCim->Set(-speed);
		rightBackCim->Set(-speed);
	}

//Sets speeds of left side drive motors
	void setLeftSpeed(double speed) {
		leftFrontCim->Set(speed);
		leftBackCim->Set(speed);
	}

//Sets speeds of right side drive motors
	void setRightSpeed(double speed) {
		rightFrontCim->Set(speed);
		rightBackCim->Set(speed);
	}

//Sets speeds of shooter motors
	void setShooter(double speed) {
		shooter1->Set(speed);
		shooter2->Set(speed);
	}

//Sets speed of intake motors
	void setIntake(double speed) {
		intakeInner->Set(speed);
		intakeOuter->Set(-speed * .75);
	}

//Solenoids for firing/locking the climbing mechanism
	void Climber() {
		switch (currClimbState) {
		case standby:
			if (driver->GetRawButton(6) && !angleSet) {
				currClimbState = angle;
			}
			break;
		case angle:
			climberAngle->Set(1);
			angleSet = true;
			if (driver->GetRawButton(6) && angleSet) {
				angleSet = false;
				currClimbState = standby;
			} else if (driver->GetRawButton(13)) {
				currClimbState = release;
			}
			break;
		case release:
			climbLock->Set(1);
			currClimbState = drive;
			break;
		case drive:
			leftPTO->Set(DoubleSolenoid::kForward);
			rightPTO->Set(DoubleSolenoid::kReverse);
		}
	}

//Removes idle stick input at 7%
	double deadband(double input) {

		if (absVal(input) < .07) {
			return 0;
		}
		return input;
	}

//C++ absolute value function sucks
	double absVal(float input) {
		if (input < 0)
			return -input;
		return input;
	}

//PID for anything
	float normPID(float myTar, float myPos, float myP, float myI) {
		PIDError = myTar + myPos;
		cumPIDError = PIDError;
		printf("hoodError: %f\n", PIDError);
		float PIDPout = PIDError * myP;
		float PIDIout = PIDError * myI;
		float PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDError) < 5) {
			PIDSpeed = 0;
		}
		return (PIDSpeed);
	}

//Velocity control for shooter
	void shooterPID() {
		printf("Shooting\n");
		float kP = 0.00020;	//.000029
		float kI = kP / 1000;	//kp/450

		shooterSpeed = shooter1->GetSpeed();
		shooterTarget = 24000;
		shooterError = shooterTarget - shooterSpeed;
		//printf("shooterSpeed: %f\n", shooterSpeed);
		//printf("shooterError: %f\n", shooterError);
		float errorMargin = 700;

		if ((absVal(shooterError) - 2000) <= errorMargin)
			isAtSpeed = true;
		else
			isAtSpeed = false;

		pOut = kP * shooterError;
		iOut += kI * shooterError;
		//printf("iOut: %f\n", iOut);

		setShooter(pOut + iOut);

	}

//PID for vision component
	float visionPID(float cogTar, float cogPos) {
		cogPIDError = cogTar - cogPos;
		float cogPIDSpeed;

		if (cogPIDError > 40) {
			cogPIDSpeed = .37;
		} else if (cogPIDError > 5) {
			cogPIDSpeed = .3;
		} else if (cogPIDError < -40) {
			cogPIDSpeed = -.37;
		} else if (cogPIDError < -5) {
			cogPIDSpeed = -.3;
		} else {
			cogPIDSpeed = 0;
		}
		printf("cogPIDError: %f\n", cogPIDError);
		printf("cogPIDSeed: %f\n", cogPIDSpeed);
		return (cogPIDSpeed);
	}

//x axis vision alignment: rotate bot
	void visionXAlign() {
		cogx = table->GetNumber("COG_X", 217);
		printf("cogx: %f", cogx);
		leftFrontCim->Set(-visionPID(cogxTar, cogx));
		leftBackCim->Set(-visionPID(cogxTar, cogx));
		rightFrontCim->Set(-visionPID(cogxTar, cogx));
		rightBackCim->Set(-visionPID(cogxTar, cogx));
	}

//y axis vision alignment
	void visionYAlign() {
		cogy = table->GetNumber("COG_Y", 217);
		if (cogy < 28)		//backed up to defenses at courtyard
				{
			hood->Set(normPID(580, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 55) {
			hood->Set(normPID(615, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 82) {
			hood->Set(normPID(735, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 109) {
			hood->Set(normPID(685, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 159) {
			hood->Set(normPID(792, hood->GetEncPosition(), .005, .0025));
		} else
			hood->Set(normPID(1006, hood->GetEncPosition(), .005, .0025));

	}
	void armPID(float armTar) {
		float armCur = arm->GetEncPosition();
		float armSpeed = -normPID(armTar, -armCur, .0007, .00003);
		if (armSpeed > .5) {
			armSpeed = .5;
		}
		if (armSpeed < -.5) {
			armSpeed = -.5;
		}
		arm->Set(armSpeed);

	}
	//Chooses defense that we will be crossing
	void defenseChoosing() {
		//sets the defense that the robot is crossing over
		if (defenseSelected == moat) {
			defense = 1;
		} else {
			if (defenseSelected == ramparts) {
				defense = 2;
			} else {
				if (defenseSelected == sallyPort) {
					defense = 3;
				} else {
					if (defenseSelected == drawbridge) {
						defense = 4;
					} else {
						if (defenseSelected == rockWall) {
							defense = 5;
						} else {
							if (defenseSelected == roughTerrain) {
								defense = 6;
							}
						}
					}
				}
			}
		}
	}

	//Crossing each defense we can in auton
	void crossingDefense() {
		switch (position) {
		//crossing the defenses when in position next to low bar
		case 1:
			switch (defense) {
			case 1:
				switch (Moat) {
				case moatDriveOver:
					setSpeed(.8);
					approachTimer->Start();
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Moat = moatRealign;
					}
					break;
				case moatRealign:
					break;
				}
				break;
			case 2:
				switch (Ramparts) {
				case rampartsDriveOver:
					approachTimer->Reset();
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Ramparts = rampartsRealign;
					}
					break;
				case rampartsRealign:
					setSpeed(0);
					break;
				}
				break;
			case 3:
				switch (SallyPort) {
				case sallyDriveThrough:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						SallyPort = sallyPortRealign;
					}
					break;
				case sallyPortRealign:
					break;
				}
				break;
			case 4:
				switch (Drawbridge) {
				case drawThrough:
					approachTimer->Start();
					printf("%f", approachTimer->Get());
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						Drawbridge = drawBridgeRealign;
					}
					break;
				case drawBridgeRealign:
					break;
				}
				break;
			case 5:
				RockWall = rockDriveForward;
				switch (RockWall) {
				case rockDriveForward:
					approachTimer->Start();

					if (approachTimer->Get() < 1) {
						setSpeed(.8);
					}
					if (approachTimer->Get() >= 1
							&& approachTimer->Get() < 3.7) {
						setSpeed(.8);
					}
					if (approachTimer->Get() >= 3.7) {
						setSpeed(0);
						RockWall = rockWallRealign;
					}
					break;
				case rockWallRealign:
					break;
				}
				break;
			case 6:
				switch (RoughTerrain) {
				case roughDriveForward:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						RoughTerrain = roughTerrainRealign;
					}
					break;
				case roughTerrainRealign:
					break;
				}
				break;
			}
			break;

			//crossing the defenses when in middle position
		case 2:
			switch (defense) {
			case 1:
				switch (Moat) {
				case moatDriveOver:
					setSpeed(.8);
					approachTimer->Start();
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Moat = moatRealign;
					}
					break;
				case moatRealign:
					break;
				}
				break;
			case 2:
				switch (Ramparts) {
				case rampartsDriveOver:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Ramparts = rampartsRealign;
					}
					break;
				case rampartsRealign:
					setSpeed(0);
					break;
				}
				break;
			case 3:
				switch (SallyPort) {
				case sallyDriveThrough:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						SallyPort = sallyPortRealign;
					}
					break;
				case sallyPortRealign:
					break;
				}
				break;
			case 4:
				switch (Drawbridge) {
				case drawThrough:
					approachTimer->Start();
					printf("%f", approachTimer->Get());
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						Drawbridge = drawBridgeRealign;
					}
					break;
				case drawBridgeRealign:
					break;
				}
				break;
			case 5:
				RockWall = rockDriveForward;
				switch (RockWall) {
				case rockDriveForward:
					if (autonTimerFlag) {
						approachTimer->Reset();
						autonTimerFlag = false;
					}
					approachTimer->Start();
					//printf("%d \n", OneBallNoSpy);
					if (approachTimer->Get() < 1) {
						setSpeed(.7);
					}
					if (approachTimer->Get() >= 1
							&& approachTimer->Get() < 4.2) {
						setSpeed(.7);\
					}
					if (approachTimer->Get() >= 4.2) /*4.3*/{
						setSpeed(0);
						RockWall = rockWallRealign;
					}
					break;
				case rockWallRealign:

					break;
				}
				break;
			case 6:
				switch (RoughTerrain) {
				case roughDriveForward:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						RoughTerrain = roughTerrainRealign;
					}
					break;
				case roughTerrainRealign:
					break;
				}
				break;
			}
			break;

			//crossing in position 3
		case 3:
			switch (defense) {
			case 1:
				switch (Moat) {
				case moatDriveOver:
					setSpeed(.8);
					approachTimer->Start();
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Moat = moatRealign;
					}
					break;
				case moatRealign:
					break;
				}
				break;
			case 2:
				switch (Ramparts) {
				case rampartsDriveOver:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Ramparts = rampartsRealign;
					}
					break;
				case rampartsRealign:
					setSpeed(0);
					break;
				}
				break;
			case 3:
				switch (SallyPort) {
				case sallyDriveThrough:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						SallyPort = sallyPortRealign;
					}
					break;
				case sallyPortRealign:
					break;
				}
				break;
			case 4:
				switch (Drawbridge) {
				case drawThrough:
					approachTimer->Start();
					printf("%f", approachTimer->Get());
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						Drawbridge = drawBridgeRealign;
					}
					break;
				case drawBridgeRealign:
					break;
				}
				break;
			case 5:
				RockWall = rockDriveForward;
				switch (RockWall) {
				case rockDriveForward:
					approachTimer->Start();

					if (autonTimerFlag) {
						approachTimer->Reset();
						autonTimerFlag = false;
					}
					approachTimer->Start();
					//printf("%d \n", OneBallNoSpy);
					if (approachTimer->Get() < 1) {
						setSpeed(.7);
					}
					if (approachTimer->Get() >= 1
							&& approachTimer->Get() < 4.1) {
						setSpeed(.7);\
					}
					if (approachTimer->Get() >= 4.1) {
						setSpeed(0);
						RockWall = rockWallRealign;
					}
					break;

				case rockWallRealign:
					break;
				}
				break;
			case 6:
				switch (RoughTerrain) {
				case roughDriveForward:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						RoughTerrain = roughTerrainRealign;
					}
					break;
				case roughTerrainRealign:
					break;
				}
				break;
			}
			break;
			//crossing in last position farthest from low bar
		case 4:
			switch (defense) {
			case 1:
				switch (Moat) {
				case moatDriveOver:
					setSpeed(.8);
					approachTimer->Start();
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Moat = moatRealign;
					}
					break;
				case moatRealign:
					break;
				}
				break;
			case 2:
				switch (Ramparts) {
				case rampartsDriveOver:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						Ramparts = rampartsRealign;
					}
					break;
				case rampartsRealign:
					setSpeed(0);
					break;
				}
				break;
			case 3:
				switch (SallyPort) {
				case sallyDriveThrough:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						SallyPort = sallyPortRealign;
					}
					break;
				case sallyPortRealign:
					break;
				}
				break;
			case 4:
				switch (Drawbridge) {
				case drawThrough:
					approachTimer->Start();
					printf("%f", approachTimer->Get());
					setSpeed(.8);
					if (approachTimer->Get() >= 2) {
						setSpeed(0);
						Drawbridge = drawBridgeRealign;
					}
					break;
				case drawBridgeRealign:
					break;
				}
				break;
			case 5:
				RockWall = rockDriveForward;
				switch (RockWall) {
				case rockDriveForward:
					approachTimer->Start();

					if (approachTimer->Get() < 1) {
						setSpeed(.8);
					}
					if (approachTimer->Get() >= 1
							&& approachTimer->Get() < 3.6) {
						setSpeed(.8);
					}
					if (approachTimer->Get() >= 3.6) {
						setSpeed(0);
						RockWall = rockWallRealign;
					}
					break;
				case rockWallRealign:
					break;
				}
				break;
			case 6:
				switch (RoughTerrain) {
				case roughDriveForward:
					approachTimer->Start();
					setSpeed(.8);
					if (approachTimer->Get() >= 3) {
						setSpeed(0);
						RoughTerrain = roughTerrainRealign;
					}
					break;
				case roughTerrainRealign:
					break;
				}
				break;
			}
			break;
		}
	}

	//selects the position that we are in
	void positionChoosing() {
		//sets the position that the robot is in
		if (positionSelected == firstPosition) {
			position = 1;
		} else {
			if (positionSelected == secondPosition) {
				position = 2;
			} else {
				if (positionSelected == thirdPosition) {
					position = 3;
				} else {
					if (positionSelected == fourthPosition) {
						position = 4;
					}
				}
			}
		}
	}

	//Aligns robot to shoot based on the position it is in
	void positionDefense() {

		switch (position) {
		case 1:
			//use gyros to turn right
			visionXAlign();
			visionYAlign();
			if (cogx > 155 && cogx < 165 && cogy == 950) {
				OneBallNoSpy = shoot;
			}
			break;
		case 2:
			visionXAlign();
			visionYAlign();

			if (absVal(-visionPID(cogxTar, cogx)) <= 0.0875
					&& hood->GetSpeed() == 0) {
				printf("WE MADE IT \n");
				OneBallNoSpy = shoot;
			}
			break;
		case 3:
			visionXAlign();
			visionYAlign();
			break;
		case 4:
			//use gyros to turn left
			visionXAlign();
			visionYAlign();
			break;
		}
	}

	//Method for choosing the correct auton based on if we are spy bot or not
	void spyBotChoosing() {
		if (spyBotSelected == yesSpyBot) {
			OneBallSpy = shootBall;
		} else {
			if (spyBotSelected == noSpyBot) {
				OneBallNoSpy = cross;
			}
		}
	}

};

START_ROBOT_CLASS(BunnyOmni)
