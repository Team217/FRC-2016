#include "WPILib.h"
#include <cmath>

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

	//Spy Bot Select
	SendableChooser *spyBotChooser;
	const string yesSpyBot = "Yes";
	const string noSpyBot = "No";
	string spyBotSelected;

	//Defense Option Select
	SendableChooser *defenseChooser;
	const string portcullis = "Portcullis";
	const string chevalDeFrise = "Cheval De Frise";
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

	//Drive Motors
	CANTalon *leftCim, *leftMini, *rightCim, *rightMini;
	//Shooter Motors
	CANTalon *intake1, *intake2, *shooter, *intakeArm, *winch;

	float shooterError, shooterTarget, shooterSpeed, pOut, iOut, output;bool isAtSpeed = false;

	//Gyros
	AnalogGyro *horzGyro, *vertGyro;

	//Controllers
	Joystick *driver, *oper;

	//Solenoids for climber fire/lock
	Solenoid *gripA, *gripB;

	//Solenoids for Porky Claws
	Solenoid *claw1A, *claw1B, *claw2A, *claw2B;

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

	//Auton Backup Timers
	Timer *approachTimer;

	//One Ball Auton
	enum OneBallNoSpy{
		approach,
		cross,
		align,
		shoot
	}OneBallNoSpy;

	enum OneBallSpy{
		shootBall,
		driveToDefense,
		crossDefense
	}OneBallSpy;

	enum portcullis{
		driveUp,
		liftArm,
		porkyClaws,
		driveForward,
	}Portcullis;

	enum cheval{
		chevalDrive,
		chevalPush,
		chevalDriveAgain
	}ChevalDeFrise;

	enum moat{
		moatDriveOver
	}Moat;

	enum ramparts{
		rampartsDriveOver
	}Ramparts;

	enum sallyPort{
		sallyLiftArm,
		sallyDriveBack,
		sallyDriveThrough
	}SallyPort;

	enum drawbridge{
		drawLiftArm,
		drawBack,
		drawThrough
	}Drawbridge;

	enum rockWall{
		rockDriveForward
	}RockWall;

	enum roughTerrain{
		roughDriveForward
	}RoughTerrain;

	//Auton Variables
	float leftCimEncTar = 217;
	float leftMiniEncTar = 217;
	float rightCimEncTar = 217;
	float rightMiniEncTar = 217;
	float intakeArmEncTar = 217;

	float intakeArmEncPos = intakeArm -> GetEncPosition();
	float leftCimEncPos = leftCim -> GetEncPosition();
	float leftMiniEncPos = leftMini -> GetEncPosition();
    float rightCimEncPos = rightCim -> GetEncPosition();
    float rightMiniEncPos = rightMini -> GetEncPosition();

    int defense,position;

	void RobotInit()
	{

		//Initializing chooser and options
		driveChooser = new SendableChooser();
		driveChooser->AddDefault(arcadeDrive, (void*) &arcadeDrive);
		driveChooser->AddObject(tankDrive, (void*) &tankDrive);
		driveChooser->AddObject(climber, (void*) &climber);

		spyBotChooser = new SendableChooser();
		spyBotChooser->AddDefault(yesSpyBot, (void*) &yesSpyBot);
		spyBotChooser->AddObject(noSpyBot, (void*) &noSpyBot);

		defenseChooser = new SendableChooser();
		defenseChooser->AddDefault(portcullis, (void*) &portcullis);
		defenseChooser->AddObject(chevalDeFrise, (void*) &chevalDeFrise);
		defenseChooser->AddObject(moat, (void*) &moat);
		defenseChooser->AddObject(ramparts, (void*) &ramparts);
		defenseChooser->AddObject(sallyPort, (void*) &sallyPort);
		defenseChooser->AddObject(drawbridge, (void*) &drawbridge);
		defenseChooser->AddObject(rockWall, (void*) &rockWall);
		defenseChooser->AddObject(roughTerrain, (void*) &roughTerrain);

		positionChooser = new SendableChooser();
		positionChooser->AddDefault(firstPosition, (void*) &firstPosition);
		positionChooser->AddObject(secondPosition, (void*) &secondPosition);
		positionChooser->AddObject(thirdPosition, (void*) &thirdPosition);
		positionChooser->AddObject(fourthPosition, (void*) &fourthPosition);

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

		claw1A = new Solenoid(0);//set(1)=leftFire ???
		claw1B = new Solenoid(3);//set(1)=leftLock ????
		claw2A = new Solenoid(1);//set(1)=rightFire?????
		claw2B = new Solenoid(2);//set(1)=rightLock??????

		//Gyros
		horzGyro = new AnalogGyro(0);
		vertGyro = new AnalogGyro(1);

		//Timers
		approachTimer = new Timer();

		//NetworkTables
		table = NetworkTable::GetTable("SmartDashboard");

		banner = new DigitalInput(0);// 0 means go; 1 means stop
	}

	void AutonomousInit()
	{
		SmartDashboard::PutData("What defense are we crossing?", defenseChooser);
		defenseSelected = *((string*) defenseChooser->GetSelected());

		SmartDashboard::PutData("What position are we in?", positionChooser);
		positionSelected = *((string*) positionChooser->GetSelected());

		SmartDashboard::PutData("Are we the Spy Bot?", spyBotChooser);
		spyBotSelected = *((string*) spyBotChooser->GetSelected());

		leftCim -> SetEncPosition(0);
		leftMini -> SetEncPosition(0);
		rightCim -> SetEncPosition(0);
		rightMini -> SetEncPosition(0);

		defenseChoosing();
		positionChoosing();
		spyBotChoosing();

	}

	void AutonomousPeriodic()
	{
		//One Ball Auton when we are NOT the Spy Bot
		switch(OneBallNoSpy){
			case approach:
				approachTimer->Start();
				leftCim -> Set(normPID(leftCimEncTar,leftCimEncPos,0.001,0.0004));
				leftMini -> Set(normPID(leftMiniEncTar,leftMiniEncPos,0.001,0.0002));
				rightCim -> Set(normPID(rightCimEncTar,rightCimEncPos,1234,1234));
				rightMini -> Set(normPID(rightMiniEncTar,rightMiniEncPos,1234,1234));

				if(((leftCimEncPos == leftCimEncTar) && (leftMiniEncPos == leftMiniEncTar) && (rightCimEncPos == rightCimEncTar) && (rightMiniEncPos == rightMiniEncTar)) || (approachTimer->Get() > 4)){
					OneBallNoSpy = cross;
				}
				break;

			case cross:
				crossingDefense();
				OneBallNoSpy = align;

				break;

			case align:
				positionDefense();
				OneBallNoSpy = shoot;
				break;

			case shoot:
				shooterPID();
				break;
		}

		//One Ball Auton Where We ARE Spy Bot
		switch(OneBallSpy){
			case shootBall:
				shooterPID();
				break;

			case driveToDefense:
				break;

			case crossDefense:
				crossingDefense();
				break;
		}
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		printEverything();

		//x rotation alignment via vision
		if (driver->GetRawButton(14))//Center square board
		{
			visionXAlign();
		}
		else
		{

			Drivebase();
			Shooter();
			Gripper();

		}

	}

	void TestPeriodic()
	{

	}

	//Function for operating drivebase. Allows for tank/Arcade option select
	void Drivebase()
	{
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

		//reset winch when its all the way down
		if (oper->GetRawButton(13))
		{
			winch->SetEncPosition(0);
		}

		//load and shoot ball
		if (ballHit == 1)
		{
			if (isAtSpeed)
			{
				setIntake(intakeForward);
			}
			else
			{
				setIntake(intakeReverse);
			}
		}
		else
			if (ballHit == 0)
			{
				if (intakeForward != 0)
				{
					setIntake(intakeForward);
				}
				else
				{
					setIntake(intakeReverse);
				}
			}

		//reving the shooter to fire
		if (oper->GetRawButton(4))
		{
			shooterPID();
		}
		else
			if (shooterForward != 0)
			{
				shooter->Set(shooterForward);//.80 - .88 for parabolic

			}
			else//reverse shooter
			{
				shooter->Set(shooterReverse * .2);
			}

		//move winch up
		if (driver->GetRawButton(5))
		{
			visionYAlign();
		}
		else
			if (winchUp != 0)
			{
				winch->Set(winchUp * .75);
			}
			else//move winch down
			{
				winch->Set(winchDown * .75);
			}

		//move intake arm up or down
		intakeArm->Set(0.5 * intakeArmSpeed);

		//upload values to SmartDashboard for display
		SmartDashboard::PutNumber("Shooter Speed", shooter->GetSpeed());
		SmartDashboard::PutBoolean("banner: ", ballHit);
		SmartDashboard::PutNumber("Hood: ", winch->GetEncPosition());
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

	void defenseChoosing(){
		//sets the defense that the robot is crossing over
		if(defenseSelected == portcullis){
			defense = 1;
		}
		else{
			if(defenseSelected == chevalDeFrise){
				defense = 2;
			}
			else{
				if(defenseSelected == moat){
					defense = 3;
				}
				else{
					if(defenseSelected == ramparts){
						defense = 4;
					}
					else{
						if(defenseSelected == sallyPort){
							defense = 5;
						}
						else{
							if(defenseSelected == drawbridge){
								defense = 6;
							}
							else{
								if(defenseSelected == rockWall){
									defense = 7;
								}
								else{
									if(defenseSelected == roughTerrain){
										defense = 8;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	void crossingDefense(){
		switch(defense){
				case 1:
					approachTimer->Reset();
					switch(Portcullis){
						case driveUp:
							//drive up portcullis ramp
							approachTimer->Start();
							setSpeed(.3);
							if(vertGyro->GetAngle >= 14.5)//when the robot starts to go up the ramp, slow down
							{
								setSpeed(.2);
								if(vertGyro->GetAngle < 14.5)//once the robot reaches the top of the ramp slow down more
								{
									setSpeed(.15);
									if(approachTimer >= 1)//after a second from starting STOP
									{
										setSpeed(0);//stop the robot
										Portcullis = liftArm;
									}
								}
							}

							break;
						case liftArm:
							intakeArm->Set(normPID(intakeArmEncTar,intakeArmEncPos,0.001,0.0004)); //raise the portcullis with the arm
							if(intakeArmEncPos == intakeArmEncTar)
							{
								Portcullis = porkyClaws;
							}
							break;
						case porkyClaws:
							claw1A->Set(1); //Fire the claws to raise the portcullis
							claw1B->Set(0);
							claw2A->Set(1);
							claw2B->Set(0);
							Portcullis = driveForward;
							break;
						case driveForward:
							//Robot drives through the defense
							approachTimer->Reset();
							approachTimer->Start();
							setSpeed(.3);//Robot drives through portcullis
							if(vertGyro->GetAngle >= 14.5)//once the robot reaches the down ramp it slows down
							{
								setSpeed(.15);
								if(vertGyro->GetAngle < 10)//once the robot leaves the down ramp it stops
								{
									setSpeed(0);//End of Potcullis Crossing
								}
							}
							break;
					}
					break;
				case 2:
					switch(ChevalDeFrise){
						case chevalDrive:
							leftCimEncTar = 120;
							leftMiniEncTar = 120;
							rightCimEncTar = 120;
							rightMiniEncTar = 120;

							leftCim -> Set(normPID(leftCimEncTar,leftCimEncPos,0.001,0.0004));
							leftMini -> Set(normPID(leftMiniEncTar,leftMiniEncPos,0.001,0.0002));
							rightCim -> Set(normPID(rightCimEncTar,rightCimEncPos,1234,1234));
							rightMini -> Set(normPID(rightMiniEncTar,rightMiniEncPos,1234,1234));
							if(((leftCimEncPos == leftCimEncTar) && (leftMiniEncPos == leftMiniEncTar) && (rightCimEncPos == rightCimEncTar) && (rightMiniEncPos == rightMiniEncTar)) || (approachTimer->Get() > .1)){
								ChevalDeFrise = chevalPush;
							}
							break;

						case chevalPush:
							setSpeed(0); //stop the robot
							intakeArm->Set(normPID(intakeArmEncTar,intakeArmEncPos,0.001,0.0004)); //lower the thingy using the intake arm
							if(intakeArmEncPos == intakeArmEncTar)
							{
								ChevalDeFrise = chevalDriveAgain;
							}
							break;
						case chevalDriveAgain:
							approachTimer->Reset();
							leftCim -> Set(normPID(leftCimEncTar,leftCimEncPos,0.001,0.0004));
							leftMini -> Set(normPID(leftMiniEncTar,leftMiniEncPos,0.001,0.0002));
							rightCim -> Set(normPID(rightCimEncTar,rightCimEncPos,1234,1234));
							rightMini -> Set(normPID(rightMiniEncTar,rightMiniEncPos,1234,1234));
							if(((leftCimEncPos == leftCimEncTar) && (leftMiniEncPos == leftMiniEncTar) && (rightCimEncPos == rightCimEncTar) && (rightMiniEncPos == rightMiniEncTar)) || (approachTimer->Get() > .1)){
								setSpeed(0); //stop so we can align
							}
							break;
						}
						break;

					case 3:
						switch(Moat){
						case moatDriveOver:
							break;
						}
						break;
					case 4:
						switch(Ramparts){
						case rampartsDriveOver:
							break;
						}
						break;
					case 5:
						switch(SallyPort){
						case sallyLiftArm:
							break;
						case sallyDriveBack:
							break;
						case sallyDriveThrough:
							break;
						}
						break;
					case 6:
						switch(Drawbridge){
						case drawLiftArm:
							break;
						case drawBack:
							break;
						case drawThrough:
							break;
						}
						break;
					case 7:
						switch(RockWall){
						case rockDriveForward:
							break;
						}
						break;
					case 8:
						switch(RoughTerrain){
						case roughDriveForward:
							break;
						}
						break;
		}
	}

	void positionChoosing(){
		//sets the position that the robot is in
		if(positionSelected == firstPosition){
			position = 1;
		}
		else{
			if(positionSelected == secondPosition){
				position = 2;
			}
			else{
				if(positionSelected == thirdPosition){
					position = 3;
				}
				else{
					if(positionSelected == fourthPosition){
						position = 4;
					}
				}
			}
		}
	}

	//Aligns robot to shoot based on the position it is in
	void positionDefense(){
		switch(position){
			case 1:
				//use gyros to turn right
				visionXAlign();
				visionYAlign();
				break;
			case 2:
				visionXAlign();
				visionYAlign();
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
	void spyBotChoosing(){
		if(spyBotSelected == yesSpyBot){
			OneBallSpy = shootBall;
		}
		else{
			if(spyBotSelected == noSpyBot){
				OneBallNoSpy = approach;
			}
		}
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
