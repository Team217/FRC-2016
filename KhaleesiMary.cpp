#include "WPILib.h"

using std::string;

class Khaleesi: public IterativeRobot {
private:
	//Defense Option Select
	int defenseSelected;

	//Position Option Select
	int positionSelected;

	//Shooter PID
	float shooterError, shooterTarget, shooterSpeed, shooterAvg, pOut, iOut,
			output;

	bool isAtSpeed = false;

	double intakeArmSpeed = 0;
	double armkP, armkI;

	//Drive Motors
	CANTalon *left, *left2, *right, *right2;

	//Shooter Motors
	CANTalon *intakeInner, *intakeOuter, *shooter1, *shooter2, *arm, *hood;

	//Controllers
	Joystick *driver, *oper;

	//Solenoids
	DoubleSolenoid *rightPTO, *leftPTO;
	Solenoid *climberAngle;
	Solenoid *climbLock;

	//Relay
	Relay *ledSpike;

	//LED bool
	bool ledBool = false;

	//Gyros
	AnalogGyro *horzGyro;
	ADXRS450_Gyro *vertGyro;
	double xAngle = 0, yAngle = 0;
	double target = 0;

	bool climbIsReleased = false; //climber is locked
	bool ptoEngaged = false;
	bool angleSet = false;
	bool pidArm = false;
	bool angleReleased = true;

	//Toggle variables for climber
	bool stateAngle = false, releasedAngle = false;
	bool stateLock = false, releasedLock = false;
	bool statePTO = false, releasedPTO = true;

	//Banner Sensor for ball detection and Hood Limit Switch
	DigitalInput *bannerInner, *zeroHood;
	bool ballHit = false;
	DigitalInput *climberSwitch;

	//NetworkTables and Vision variables
	std::shared_ptr<NetworkTable> table;
	float cogx = 0;
	float cogy = 0;
	float cumcogPIDError = 0;
	float cogPIDError = 0;

	//x PID
	float cogxP = 0.005;
	float cogxI = 0.0009; //.05
	float cogxTar = 180;

	//normPID/Y
	float PIDError = 0;
	float cumPIDError = 0;

	//shooting state machine
	enum shootState {
		intake, // Move around and accept a ball
		// Starts 2.5 sec after ball is fired. Ends if ball hits banner sensor
		reving,	// Spin firing talon until its fast enough to throw a ball into a goal
		// Starts after ball hits banner, ends if aborted or at 24000 speed
		firing		// Ball is sent to the firing talon to fly through the air
	// Starts when reving is at full speed and ends after 2.5 seconds
	};
	shootState currShootState;

	enum climbState {
		angle,	//Toggle for climber angle. Begins in upright position.
		//Must be retoggled during climb to go up straight
		release,		//Engages climber to grab onto tower
		drive		//Disengages PTOs to climb tower
	} currClimbState;

	//Timers
	Timer *approachTimer;		//auton
	Timer *resetFiringState;		//teleop shooterPID

	//Auton Enums
	enum moatEnum {
		moatIntake, moatFirstBump, moatSecondBump, moatShoot
	} moatState;

	enum rockWallEnum {
		rockWallIntake,
		rockWallApproach,
		rockWallCross,
		rockWallDrive,
		rockWallShoot
	} rockWallState;
	enum roughTerrainEnum {
		roughTerrainIntake, roughTerrainDrive, roughTerrainShoot
	} roughTerrainState;

	enum autonShootState {
		autonAligning,		//Robot turns left or right in order to see goal
		autonAligning2,		//Robot aligns according to X and Y vision
		autonReving,		//Shooter begins reving up to fire
		autonFiring		//Shooter fires
	} autonShootState;

	//Auton SmartDash Variables
	int defense, position;
	bool autonTimerFlag = true;
	//Auton Variables
	float autonTime = 0;		//chosen via pos
	bool turnRight = false;		//rotate until vision targets

	//makes auton do defenseChoosing() once in periodic
	bool onetimepls = 1;

	//Live Stream-Intermediate
	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;
	//double maxY=0;

	void RobotInit() {
		//Gyros
		horzGyro = new AnalogGyro(0);		//need to init still
		vertGyro = new ADXRS450_Gyro();

		//Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);

		//Drive motors
		left = new CANTalon(14);
		left2 = new CANTalon(15);
		left2->SetControlMode(CANSpeedController::kFollower);
		left2->Set(14);

		right = new CANTalon(12);
		right2 = new CANTalon(13);
		right2->SetControlMode(CANSpeedController::kFollower);
		right2->Set(12);

		//Intake, shooter motors
		intakeInner = new CANTalon(6);
		intakeOuter = new CANTalon(3);
		shooter1 = new CANTalon(0);
		shooter2 = new CANTalon(1);
		arm = new CANTalon(2);
		hood = new CANTalon(7);

		//Relay
		ledSpike = new Relay(0, Relay::kForwardOnly);

		//Solenoids
		rightPTO = new DoubleSolenoid(0, 3);
		leftPTO = new DoubleSolenoid(1, 2);
		climberAngle = new Solenoid(5);
		climbLock = new Solenoid(4);

		//NetworkTables
		table = NetworkTable::GetTable("SmartDashboard");

		//Ball Recog Sensors
		bannerInner = new DigitalInput(0);		// 0 means go; 1 means stop

		//Hood Limit Switch
		zeroHood = new DigitalInput(1);	// 1 means not pressed; 0 means pressed

		//Climber drive stop
		climberSwitch = new DigitalInput(9);

		//Timer
		approachTimer = new Timer();
		resetFiringState = new Timer();

		//Default auton shoot state
		autonShootState = autonAligning;

		//create an image-Live Stream
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		//the camera name (ex "cam0") can be found through the roborio web dashboard
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController,
				&session);
		imaqError = IMAQdxConfigureGrab(session);
		SmartDash();
	}

	void AutonomousInit() {
		ledBool = false;
		//maxY=0;
		//auton selection
		onetimepls = 1;
		leftPTO->Set(DoubleSolenoid::kForward);
		rightPTO->Set(DoubleSolenoid::kReverse);

		string testString = SmartDashboard::GetString("DB/String 7", "2");
		defenseSelected = std::stoi(testString);		//what defense

		string testmyString = SmartDashboard::GetString("DB/String 8", "2");
		positionSelected = std::stoi(testmyString);		//what position

		SmartDash();

		//Auton Timer Reset
		approachTimer->Reset();

		//Select Defense and Position
		defenseChoosing();
		positionChoosing();

		autonShootState = autonAligning;

		//Reset Gyros for Auton Use
		vertGyro->Reset();
		horzGyro->InitGyro();
		horzGyro->Reset();

		//Reset autonTime
		autonTime = 0;
	}

	void AutonomousPeriodic() {
//		if(vertGyro->GetAngle() > maxY)
//		{
//		maxY=vertGyro->GetAngle();
//		printf("%f",maxY);
//		}

		//get values from PC Dashboard for auton
		string testString = SmartDashboard::GetString("DB/String 7", "2");
		defenseSelected = std::stoi(testString);		//what defense

		string testmyString = SmartDashboard::GetString("DB/String 8", "2");
		positionSelected = std::stoi(testmyString);		//what position

		//runs defense choosing once
		if (onetimepls) {
			defenseChoosing();
			onetimepls = 0;
		}
		positionChoosing();

		switch (defense) {
		case 1:
			Moat(position);
			break;
		case 2:
			break;
		case 3:
			RockWall(position);
			break;
		case 4:
			RoughTerrain(position);
			break;
		}
	}

	void Moat(int pos) {
		switch (pos) {
		case 1:
			autonTime = 0;
			break;
		case 2:
			autonTime = 3.1;
			break;
		case 3:
			autonTime = 3.0;
			break;
		case 4:
			autonTime = 3.0;
			break;
		case 5:
			autonTime = 3.1;		//2.5 if just cross
			break;
		}
		switch (moatState) {
		case moatIntake:
			ballHit = bannerInner->Get();
			setIntake(1);
			armPID(0);
			if (ballHit == 1) {
				setIntake(0);
				moatState = moatFirstBump;
			}
			break;
		case moatFirstBump:
			armPID(0);
			left->Set(0.5);
			right->Set(-0.5);
			if (vertGyro->GetRate() >= 25) {
				left->Set(0);
				right->Set(0);
				Wait(.75);
				approachTimer->Reset();
				approachTimer->Start();
				moatState = moatSecondBump;
			}
			break;
		case moatSecondBump:
			armPID(0);
			left->Set(1);
			right->Set(-1);
			if (approachTimer->Get() >= autonTime) {
				left->Set(0);
				right->Set(0);
				moatState = moatShoot;
			}
			break;
		case moatShoot:
			autonShoot();
			break;
		}

	}

	void RockWall(int pos) {
		switch (pos) {
		case 1:
			autonTime = 0;
			break;
		case 2:
			autonTime = 1.5;
			break;
		case 3:
			autonTime = 1.15;
			break;
		case 4:
			autonTime = 1.0;
			break;
		case 5:
			autonTime = 1.5;
			break;
		}
		SmartDash();
		switch (rockWallState) {
		case rockWallIntake:
			setIntake(1);
			ballHit = bannerInner->Get();
			armPID(0);
			if (ballHit == 1) {
				setIntake(0);
				rockWallState = rockWallApproach;
			}
			break;
		case rockWallApproach:
			armPID(0);
			setSpeed(.59);
			if (vertGyro->GetRate() <= -60) {
				Wait(.1);
				vertGyro->GetRate();
				rockWallState = rockWallCross;
			}
			break;
		case rockWallCross:
			//TODO: implement vertGyro control for timer
			setSpeed(.59);
			armPID(0);
			if (vertGyro->GetRate() <= -85) {
				left->Set(0);
				right->Set(0);
				Wait(1);
				approachTimer->Reset();
				approachTimer->Start();
				rockWallState = rockWallDrive;
			}
			break;
		case rockWallDrive:
			setSpeed(.63);
			if (approachTimer->Get() >= autonTime) {
				setSpeed(0);
				rockWallState = rockWallShoot;
			}
			break;
		case rockWallShoot:
			autonShoot();
			break;
		}
	}

	void RoughTerrain(int pos) {
		switch (pos) {
		//All times reduced 1.2 s 3/26/2016
		case 1:
			autonTime = 0;
			break;
		case 2:
			autonTime = 4.28;
			break;
		case 3:
			autonTime = 4.0;
			break;
		case 4:
			autonTime = 4.0;
			break;
		case 5:
			autonTime = 4.15;
			break;
		}

		switch (roughTerrainState) {
		case roughTerrainIntake:
			approachTimer->Reset();
			approachTimer->Start();
			setIntake(1);
			ballHit = bannerInner->Get();
			armPID(0);
			if (ballHit == 1) {
				setIntake(0);
				roughTerrainState = roughTerrainDrive;
			}
			break;
		case roughTerrainDrive:
			setSpeed(.55);
			if (approachTimer->Get() >= autonTime) {
				setSpeed(0);
				roughTerrainState = roughTerrainShoot;
			}
			break;
		case roughTerrainShoot:
			autonShoot();
			break;
		}
	}

	//State machine for aligning and firing in auton
	void autonShoot() {
		cogx = table->GetNumber("COG_X", 1000);
		cogy = table->GetNumber("COG_Y", 1000);
		float hoodSpeed = hood->GetSpeed();

		switch (autonShootState) {
		case autonAligning:
			switch (position) {
			case 1:
			case 2:
			case 3:
				turnRight = true;
				break;
			case 4:
			case 5:
				turnRight = false;
				break;
			default:
				break;
			}
			if (turnRight && cogx == 0) {
				left->Set(.5);
				right->Set(.5);
			} else if (!turnRight && cogx == 0) {
				left->Set(-.5);
				right->Set(-.5);
			} else {
				left->Set(0);
				right->Set(0);
				autonShootState = autonAligning2;
			}
			break;
		case autonAligning2:
			visionXAlign();
			//stops hood from going past point
			if (hood->GetEncPosition() < -1450) {
				hood->Set(normPID(-1300, hood->GetEncPosition(), .005, .0025));
			} else
				visionYAlign();
			if (cogx < 175 && cogx > 165 && hoodSpeed == 0) {
				autonShootState = autonReving;
			}
			break;
		case autonReving:
			visionXAlign();
			//stops hood from going past point
			if (hood->GetEncPosition() < -1450) {
				hood->Set(normPID(-1300, hood->GetEncPosition(), .005, .0025));
			} else
				visionYAlign();
			// In this mode, the robot is waiting for the shooter to reach 24K speed
			shooterPID();
			// This runs the rev up process, PID'ing the shooter to 24000.
			if (isAtSpeed)		//isAtSpeed
			{
				// switch states because we are completely reved up
				resetFiringState->Reset();
				resetFiringState->Start();
				autonShootState = autonFiring;

			}
			break;
		case autonFiring:
			if (resetFiringState->Get() < 1) {
				setIntake(1);
			} else {
				setIntake(0);
				setShooter(0);
				iOut = 0;
			}
			// If the ball is still at the banner sensor, push it into the shooter

			break;
		}
	}

	void TeleopInit() {

		SmartDash();

		//Live Stream
		IMAQdxStartAcquisition(session);

		//ensure solenoids are disengaged
		climberAngle->Set(0);
		climbLock->Set(0);

		leftPTO->Set(DoubleSolenoid::kForward);
		rightPTO->Set(DoubleSolenoid::kReverse);

	}

	void TeleopPeriodic() {

		SmartDash();

		shooterSpeed = shooter1->GetSpeed();

		if (driver->GetRawButton(14))		//Center square board
				{
			visionXAlign();
		} else {
			Drivebase();
		}
		if (oper->GetRawButton(10)) {
			setShooter(1);
			if (oper->GetRawButton(14)) {
				setIntake(1);
			}
		} else {
			Shooter();
		}
		Gyro();
		Arm();
		//stops hood from going past point
		if (hood->GetEncPosition() < -1450) {
			hood->Set(normPID(-1300, hood->GetEncPosition(), .005, .0025));
		} else
			Hood();
		Climber();
		LiveStream();
	}
	void DisabledInit() {
		ledSpike->Set(Relay::kOff);
		ledBool = false;
	}

	void TestPeriodic() {

	}

	//Function to capture images and send to DashBoard
	void LiveStream() {
		IMAQdxStartAcquisition(session);
		//Acquire Images
		IMAQdxGrab(session, frame, true, NULL);

		//Send Image to DashBoard
		CameraServer::GetInstance()->SetImage(frame);
	}

	//Function to display various data to SmartDashboard
	void SmartDash() {
		string HoodString = "Hood: " + std::to_string(hood->GetEncPosition());
		SmartDashboard::PutString("DB/String 5", HoodString);

		string armString = "arm: " + std::to_string(arm->GetEncPosition());
		SmartDashboard::PutString("DB/String 6", armString);
		string cogxString = "COG_X "
				+ std::to_string(table->GetNumber("COG_X", 1000));
		SmartDashboard::PutString("DB/String 0", cogxString);

		string cogyString = "COG_Y "
				+ std::to_string(table->GetNumber("COG_Y", 1000));
		SmartDashboard::PutString("DB/String 1", cogyString);

		string shooterString = "Shooter: "
				+ std::to_string(shooter1->GetSpeed());
		SmartDashboard::PutString("DB/String 2", shooterString);

		string bann;
		if (bannerInner->Get())
			bann = "true";
		if (!bannerInner->Get())
			bann = "false";
		string ballHitString = "BallHit: " + bann;
		SmartDashboard::PutString("DB/String 3", ballHitString);

		string pto;
		if (ptoEngaged == 1)
			pto = "true";
		if (ptoEngaged == 0)
			pto = "false";
		string PTOString = "PTO: " + pto;
		int RockWallString = rockWallState;
		SmartDashboard::PutNumber("DB/String 4", rockWallState);

		string gyroString = "VertGyro: " + std::to_string(vertGyro->GetAngle());
		SmartDashboard::PutString("DB/String 9", gyroString);
	}

	//Function for operating drivebase. Allows for tank/Arcade option select
	//Includes gyro correction. Must be tested
	void Drivebase() {
		double speed = deadband(-driver->GetY());
		double turn = deadband(driver->GetZ());

		if (!driver->GetRawButton(8)) {
			target = xAngle;
		}

		if (climberSwitch->Get() == 0 && !ptoEngaged) {
			left->Set(0);
			right->Set(0);
		}

		else {
			left->Set(speed + turn + gyroPID(target));
			right->Set(-speed + turn + gyroPID(target));
		}
	}
	double gyroPID(double target) {
		double gyroError;
		double gyroKP = 0.03;

		gyroError = target - xAngle;
		return gyroError * gyroKP;
	}
	//Updates angle variables, resets x gyro when full rotation is completed
	void Gyro() {
		xAngle = horzGyro->GetAngle();
		yAngle = vertGyro->GetAngle();
		if (xAngle == 360 || xAngle == -360) {
			xAngle = 0;
		}
		if (driver->GetRawButton(9)) {
			horzGyro->Reset();
			vertGyro->Reset();
		}
	}

	//Function for operating shooter
	void Hood() {

		//stops hood from going past point
		if (hood->GetEncPosition() < -1450) {
			hood->Set(normPID(-1300, hood->GetEncPosition(), .005, .0025));
		} else {
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
				if (oper->GetRawButton(6)) {
					hood->Set(.75);
				} else if (oper->GetRawButton(8)) {
					hood->Set(-.75);
				} else
					hood->Set(0);
			}

			if (oper->GetPOV() == 0) {
				hood->Set(normPID(0, hood->GetEncPosition(), .0048, .0025));
			}
			if (oper->GetPOV() == 90) {
				hood->Set(normPID(640, hood->GetEncPosition(), .0048, .0025));//610
			}

			if (oper->GetPOV() == 180) {
				hood->Set(normPID(660, hood->GetEncPosition(), .0048, .0025));//690
			}

			if (oper->GetPOV() == 270) {
				hood->Set(normPID(915, hood->GetEncPosition(), .0048, .0025));//1000
			}
		}

	}

	//state machine for operating shooter
	void Shooter() {
		/*
		 * 1. Operator uses intake to claim a ball and can spit out as well.
		 * 	Arm comes down automatically.
		 * 2. Banner sensor is hit to stop ball. Ball can be spit out.
		 * 	Arm comes all the way up.
		 * 3. Operator revs up shooter and holds intake, intake is not activated
		 * 	until shooter is up to speed.
		 */
		ballHit = bannerInner->Get();
		if (ballHit || ledBool) {
			ledSpike->Set(Relay::kOn);
		} else {
			ledSpike->Set(Relay::kOff);
		}
		switch (currShootState) {

		case intake:
			setShooter(0);
			isAtSpeed = false;

			// In this mode, the robot is waiting to obtain a ball
			if (oper->GetRawButton(5)) {
				setIntake(-1);

				// If the regurgitation button is pressed, pull a ball in
			} else if (oper->GetRawButton(7)) {
				setIntake(1);
				pidArm = true;
				armPID(-1300);
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
			// In this mode, the robot is waiting for the shooter to reach 24K speed
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

			} else if (oper->GetRawButton(3)) {
				intakeOuter->Set(-1);
			} else if (!oper->GetRawButton(3)) {
				intakeOuter->Set(0);
			}
			if (oper->GetRawButton(4)) {
				shooterPID();
				// This runs the rev up process, PID'ing the shooter to 24000.
			} else {
				setShooter(0);
			}

			if (isAtSpeed && oper->GetRawButton(7))	//isAtSpeed
					{
				// switch states because we are completely reved up
				currShootState = firing;

				resetFiringState->Reset();
				resetFiringState->Start();
			}
			break;
		case firing:
			if (ballHit == 1) {
				// If the ball is still at the banner sensor, push it into the shooter
				intakeInner->Set(-1);
			}
			if (resetFiringState->Get() >= 1.5) {
				currShootState = intake;
				iOut = 0;
			}
			break;
		}

	}

	//function for operating arm
	void Arm() {
		/*
		 * Arm comes up full speed, down half speed (0.5)
		 *
		 * Square - arm comes up to 0
		 * X - arm becomes level with floor
		 */

		//left stick
		intakeArmSpeed = deadband(oper->GetY());
		if (arm->GetEncPosition() > -125 && intakeArmSpeed < 0) {
			intakeArmSpeed = 0;
		}
		if (oper->GetRawButton(13)) {
			arm->SetEncPosition(0);

		}

		//move intake arm up or down
		if (intakeArmSpeed < 0) {
			arm->Set(intakeArmSpeed * .5);
			pidArm = false;
		} else if (intakeArmSpeed > 0) {
			arm->Set(intakeArmSpeed * 0.3);
			pidArm = false;
		} else if (oper->GetRawButton(1)) {
			armPID(0);
		} else if (oper->GetRawButton(2)) {
			armPID(-1100);
		} else if (!pidArm) {
			arm->Set(0);
		}
	}

	//Sets speeds of both drive motors to for/back
	void Climber() {

		/*
		 * Square - Toggle for angle
		 * X - Toggle for release
		 * Circle - Toggle for PTOs
		 */

		//climber angle Toggle
		if (driver->GetRawButton(1) && !stateAngle)	//after second push
				{
			releasedAngle = false;
			climberAngle->Set(0);	//becomes 0 degrees

		} else if (driver->GetRawButton(1) && stateAngle)	//first push
				{
			releasedAngle = false;
			climberAngle->Set(1);	//becomes 30 degrees

		} else if (!driver->GetRawButton(1) && !releasedAngle)	//happens first
				{
			stateAngle = !stateAngle;
			releasedAngle = true;
		}

		//climber release Toggle
		if (driver->GetRawButton(2) && !stateLock)	//after second push
				{
			releasedLock = false;
			climbLock->Set(0);	//re-engages lock

		} else if (driver->GetRawButton(2) && stateLock)	//first push
				{
			releasedLock = false;
			climbLock->Set(1);	//releases lock
			ledBool = true;
		} else if (!driver->GetRawButton(2) && !releasedLock)	//happens first
				{
			stateLock = !stateLock;
			releasedLock = true;
		}

		//climberPTO Toggle
		if (driver->GetRawButton(3) && !statePTO)	//after second push
				{
			releasedPTO = false;
			ptoEngaged = false;
			leftPTO->Set(DoubleSolenoid::kReverse);
			rightPTO->Set(DoubleSolenoid::kForward);	//driving mode

		} else if (driver->GetRawButton(3) && statePTO)	//first push
				{
			releasedPTO = false;
			ptoEngaged = true;
			leftPTO->Set(DoubleSolenoid::kForward);
			rightPTO->Set(DoubleSolenoid::kReverse);	//climbing mode

		} else if (!driver->GetRawButton(3) && !releasedPTO)	//happens first
				{
			statePTO = !statePTO;
			releasedPTO = true;
		}

	}

	//PID for x-axis alignment
	void visionXAlign() {
		cogx = table->GetNumber("COG_X", 1000);
		left->Set(-visionPID(cogxTar, cogx));
		right->Set(-visionPID(cogxTar, cogx));
	}

	//PID for hood alignment
	void visionYAlign() {
		cogy = table->GetNumber("COG_Y", 217);
		if (cogy < 65)	//backed up to defenses at courtyard
				{
			hood->Set(normPID(640, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 86) {
			hood->Set(normPID(660, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 107) {
			hood->Set(normPID(660, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 128) {
			hood->Set(normPID(660, hood->GetEncPosition(), .005, .0025));
		} else if (cogy < 163) {
			hood->Set(normPID(660, hood->GetEncPosition(), .005, .0025));
		} else
			//close to tower
			hood->Set(normPID(915, hood->GetEncPosition(), .005, .0025));

	}

	//PID for x-axis vision alignment use
	float visionPID(float cogTar, float cogPos) {
		cogPIDError = cogTar - cogPos;
		float cogPIDSpeed;

		if (cogPIDError > 40) {
			cogPIDSpeed = .33;	//was .37
		} else if (cogPIDError > 5) {
			cogPIDSpeed = .29;	//was .27
		} else if (cogPIDError < -40) {
			cogPIDSpeed = -.33;	//was -.37
		} else if (cogPIDError < -5) {
			cogPIDSpeed = -.29;	//-.27
		} else {
			cogPIDSpeed = 0;
		}
		return (cogPIDSpeed);
	}

	//PID for arm
	void armPID(float armTar) {
		float armCur = arm->GetEncPosition();
		float armSpeed = -normPID(armTar, -armCur, .001, .0003);
		if (armSpeed > .5) {
			armSpeed = .5;
		}
		if (armSpeed < -.5) {
			armSpeed = -.5;
		}
		arm->Set(armSpeed);

	}

	//PID for shooter
	void shooterPID() {
		float kP = 0.00020;	//.000029
		float kI = kP / 1000;	//kp/450

		shooterSpeed = shooter1->GetSpeed();
		shooterTarget = 24000;
		shooterError = shooterTarget - shooterSpeed;
		float errorMargin = 700;

		if ((absVal(shooterError) - 2000) <= errorMargin)
			isAtSpeed = true;
		else
			isAtSpeed = false;

		pOut = kP * shooterError;
		iOut += kI * shooterError;

		setShooter(pOut + iOut);

	}

	//Standard PID function
	float normPID(float myTar, float myPos, float myP, float myI) {
		PIDError = myTar + myPos;
		cumPIDError = PIDError;
		float PIDPout = PIDError * myP;
		float PIDIout = PIDError * myI;
		float PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDError) < 5) {
			PIDSpeed = 0;
		}
		return (PIDSpeed);
	}

	//Sets all wheels to drive at same speed
	void setSpeed(double speed) {
		left->Set(speed);
		right->Set(-speed);
	}

//Sets speeds of shooter motors
	void setShooter(double speed) {
		shooter1->Set(speed);
		shooter2->Set(speed);
	}

//Sets speed of intake motors
	void setIntake(double speed) {
		intakeInner->Set(-speed);
		intakeOuter->Set(speed);
	}

	//Nulls idle stick input at 0.08
	double deadband(double input) {

		if (absVal(input) < .08) {
			return 0;
		}
		return input;
	}

	//choosing position for auton
	void positionChoosing() {
		if (positionSelected == 1)
			position = 1;
		if (positionSelected == 2)
			position = 2;
		if (positionSelected == 3)
			position = 3;
		if (positionSelected == 4)
			position = 4;
		if (positionSelected == 5)
			position = 5;
	}

	//choosing defense to cross in auton
	void defenseChoosing() {
		if (defenseSelected == 1) {
			defense = 1;
			moatState = moatIntake;
		}
		if (defenseSelected == 2) {
			defense = 2;
		}
		if (defenseSelected == 3) {
			defense = 3;
			rockWallState = rockWallIntake;
		}
		if (defenseSelected == 4) {
			defense = 4;
			roughTerrainState = roughTerrainIntake;
		}
	}

	//C++ abs function sucks
	double absVal(float input) {
		if (input < 0)
			return -input;
		return input;
	}

};

START_ROBOT_CLASS(Khaleesi)
