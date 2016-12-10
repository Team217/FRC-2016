#include "WPILib.h"

using std::string;

class Robot: public IterativeRobot
{
	private:

	//SmartDashboard chooser, option names
	SendableChooser *driveChooser;
	const string tankDrive = "Tank Drive";
	const string arcadeDrive = "Arcade Drive";
	string driveSelected;

	//Drive Motors
	CANTalon *leftCim, *leftMini, *rightCim, *rightMini;
	//Shooter Motors
	CANTalon *intake1, *intake2, *shooter, *intakeArm, *winch;

	//Controllers
	Joystick *driver, *oper;

	//Solenoids for climber fire/lock
	Solenoid *gripA, *gripB;

	void RobotInit()
	{

		//Initializing chooser and options
		driveChooser = new SendableChooser();
		driveChooser->AddDefault(arcadeDrive, (void*) &arcadeDrive);
		driveChooser->AddObject(tankDrive, (void*) &tankDrive);

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
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		Drivebase();
		Shooter();
		Gripper();
	}

	void TestPeriodic()
	{

	}

	//Function for operating drivebase. Allows for tank/Arcade option select
	void Drivebase()
	{
		//Places chooser onto SmartDashboard and records option select
		//SmartDashboard::PutData("Drive Modes", driveChooser);
		//driveSelected = *((string*) driveChooser->GetSelected());

		double Speed = deadband(-driver->GetY());
		double leftSpeed = deadband(-driver->GetY());
		double rightSpeed = deadband(driver->GetRawAxis(5));

		setSpeed(Speed);
		//setLeftspeed(leftSpeed)
		//setRightSpeed(rightSpeed);

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

		if (intakeForward != 0)
		{
			intake1->Set(intakeForward);
			intake2->Set(-intakeForward * 0.5);
		}
		else
		{
			intake1->Set(intakeReverse);
			intake2->Set(-intakeReverse * 0.5);
		}

		if (shooterForward != 0)
		{
			shooter->Set(shooterForward);
		}
		else
		{
			shooter->Set(shooterReverse * .2);
		}

		if (winchUp != 0)
		{
			winch->Set(winchUp);
		}
		else
			winch->Set(winchDown);

		intakeArm->Set(0.5 * intakeArmSpeed);

		SmartDashboard::PutNumber("Shooter Speed", shooter->Get());
		SmartDashboard::PutNumber("Intake Speed", intake1->Get());

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

	//Sets speeds of both drive motors
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

	//Removes idle stick input at 7%
	double deadband(double input)
	{
		double bounds = 0.07;
		if (absVal(input) < bounds)
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
};

START_ROBOT_CLASS(Robot)
