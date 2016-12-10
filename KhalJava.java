
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	/*
	 * public static final ExampleSubsystem exampleSubsystem = new
	 * ExampleSubsystem(); public static OI oi;
	 *
	 * Command autonomousCommand; SendableChooser chooser;
	 */
	// Defense Option Select
	int defenseSelected;

	// Position Option Select
	int positionSelected;

	// auton fire option select
	int autonFireSelected;

	// auton Timer.delay option select
	double autonWaitSelected;

	// Shooter PID
	double flyWheelSpeed;
	double flyWheelTarget = 3600;

	boolean isAtSpeed = false;

	double intakeArmSpeed = 0;
	double armKP, armKI;

	// Drive Motors
	CANTalon left, left2, right, right2;

	// Shooter Motors
	CANTalon intakeInner, intakeOuter, flyWheel, flyWheel2, arm, hood;

	// Controllers
	Joystick driver, oper;

	// Solenoids
	DoubleSolenoid rightPTO, leftPTO;
	Solenoid climberAngle;
	Solenoid climbLock;

	// Relay
	Relay ledSpike;

	// LED bool
	boolean ledBool = false;

	// Gyros
	AnalogGyro horzGyro;
	ADXRS450_Gyro vertGyro;
	double xAngle = 0, yAngle = 0;
	double target = 0;
	double gyroKP = 0.05;

	boolean climbIsReleased = false; // climber is locked
	boolean ptoEngaged = true;
	boolean angleSet = false;
	boolean pidArm = false;
	boolean angleReleased = true;
	boolean rampartsFlag = false;

	// Toggle variables for climber
	boolean stateAngle = true, releasedAngle = true;
	boolean stateLock = true, releasedLock = true;
	boolean statePTO = false, releasedPTO = true;

	// Toggle variables for anti-tip
	boolean stateGyro = true, releasedGyro = true, gyroDisable = false;

	// Banner Sensor for ball detection and Hood Limit Switch
	DigitalInput bannerInner, zeroHood;
	boolean ballHit = false;
	DigitalInput climberSwitch;

	// NetworkTables and Vision variables
	NetworkTable table;
	NetworkTable preferences;
	double cogx = 0;
	double cogy = 0;
	double cumcogPIDError = 0;
	double cogPIDError = 0;

	// x PID
	double cogxP = 0.005;
	double cogxI = 0.0009; // .05
	double cogxTar = 190; // TODO: get updated val

	// normPID/Y
	double PIDError = 0;
	double cumPIDError = 0;

	// shooting state machine
	enum shootState {
		intake, // Move around and accept a ball
		// Starts 2.5 sec after ball is fired. Ends if ball hits banner sensor
		reving, // Spin firing talon until its fast enough to throw a ball into
				// a goal
		// Starts after ball hits banner, ends if aborted or at 24000 speed
		firing // Ball is sent to the firing talon to fly through the air
		// Starts when reving is at full speed and ends after 2.5 seconds
	};

	shootState currShootState;

	// Timers
	Timer approachTimer; // auton
	Timer rampartsTimer;
	Timer resetFiringState; // teleop shooterPID
	Timer posTwoTimer; // ummmmm uhhhhh position 2 timer

	// Auton Enums
	enum moatEnum {
		moatIntake, moatFirstBump, moatBack, moatSecondBump, moatDrive, moatDrive2, moatRotate, moatShoot
	};

	moatEnum moatState;

	enum rockWallEnum {
		rockWallIntake, rockWallApproach, rockWallCross, rockWallDrive, rockWallDrive2, rockWallRotate, rockWallShoot
	};

	rockWallEnum rockWallState;

	enum roughTerrainEnum {
		roughTerrainIntake, roughTerrainDrive, roughTerrainRotate, roughTerrainShoot
	};

	roughTerrainEnum roughTerrainState;

	enum rampartsEnum {
		rampartsIntake, rampartsRush, rampartsApproach, rampartsCross, rampartsDrive, rampartsRotate, rampartsShoot
	};

	rampartsEnum rampartsState;

	enum spyEnum {
		spyIntake, spyRotate, spyShoot
	};

	spyEnum spyState;

	enum autonShootState {
		autonAligning, // Robot turns left or right in order to see goal
		autonAligning2, // Robot aligns according to X and Y vision
		autonReving, // Shooter begins reving up to fire
		autonFiring // Shooter fires
	};

	autonShootState autonShoot;

	enum posTwo {
		turn1, forward, turn2
	};

	posTwo posTwoState;

	// Auton SmartDash Variables
	int defense, position;
	boolean autonTimerFlag = true;
	double gyroTar = 0;
	double antiTipAngle = -28;
	boolean antiTipEngaged = false;
	double angleTar = 0;
	// Auton Variables
	double autonTime = 0; // chosen via pos
	boolean turnRight = false; // rotate until vision targets
	// Live Stream-Intermediate
	int session;
	Image frame;

	int myclimb = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);

		// Gyros
		horzGyro = new AnalogGyro(0); // need to init still
		vertGyro = new ADXRS450_Gyro();

		horzGyro.initGyro();

		// Joysticks
		driver = new Joystick(0);
		oper = new Joystick(1);

		// Drive motors
		left = new CANTalon(14);
		left2 = new CANTalon(15);
		left2.changeControlMode(TalonControlMode.Follower);
		left2.set(14);

		right = new CANTalon(12);
		right2 = new CANTalon(13);
		right2.changeControlMode(TalonControlMode.Follower);
		right2.set(12);

		// Intake, shooter motors
		intakeInner = new CANTalon(6);
		intakeOuter = new CANTalon(3);

		flyWheel = new CANTalon(0);
		flyWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		flyWheel.configEncoderCodesPerRev(1000);
		flyWheel.changeControlMode(TalonControlMode.Speed);
		flyWheel.setProfile(1);

		flyWheel2 = new CANTalon(1);
		flyWheel2.changeControlMode(TalonControlMode.Follower);
		flyWheel2.set(0);

		arm = new CANTalon(2);
		hood = new CANTalon(7);

		// Relay
		ledSpike = new Relay(0);

		// Solenoids
		rightPTO = new DoubleSolenoid(0, 3);
		leftPTO = new DoubleSolenoid(1, 2);
		climberAngle = new Solenoid(5);
		climbLock = new Solenoid(4);

		// NetworkTables
		table = NetworkTable.getTable("SmartDashboard");
		preferences = NetworkTable.getTable("Preferences");

		// Ball Recog Sensors
		bannerInner = new DigitalInput(0); // 0 means go; 1 means stop

		// Hood Limit Switch
		zeroHood = new DigitalInput(1); // 1 means not pressed; 0 means pressed

		// Climber drive stop
		climberSwitch = new DigitalInput(9);

		// Timer
		approachTimer = new Timer();
		resetFiringState = new Timer();
		rampartsTimer = new Timer();
		posTwoTimer = new Timer();

		// Default auton shoot state
		autonShoot = autonShootState.autonAligning;
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {
		ledSpike.set(Relay.Value.kOff);
		ledBool = false;
	}

	public void disabledPeriodic() {
		// Scheduler.getInstance().run();
		if (climberSwitch.get() == false) {
			vertGyro.calibrate();
			horzGyro.calibrate();
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	public void autonomousInit() {
		// autonomousCommand = (Command) chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		// if (autonomousCommand != null) autonomousCommand.start();

		ledBool = false;
		left.set(0);
		right.set(0);

		//Ramparts gyro target
		gyroTar = 0;

		//auton selection
		AutonSelect();

		SmartDash();

		//Auton Timer Reset
		approachTimer.reset();

		//Select Defense and Position
		leftPTO.set(DoubleSolenoid.Value.kForward);
		rightPTO.set(DoubleSolenoid.Value.kReverse);

		defenseChoosing();
		positionChoosing();

		autonShoot = autonShootState.autonAligning;
		posTwoState = posTwo.turn1;

		//Reset Gyros for Auton Use
		vertGyro.reset();
		horzGyro.reset();
		xAngle = 0;
		yAngle = 0;

		//Reset autonTime,rampartFlag, and gyroTar
		autonTime = 0;
		rampartsFlag = false;
		gyroTar = 0;

		//Wait a specified amount of time before auton
		Timer.delay(autonWaitSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		// Scheduler.getInstance().run();
		SmartDash();
		xAngle = horzGyro.getAngle();

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
		case 5:
			Ramparts(position);
			break;
		case 6:
			Spy();
		}
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// if (autonomousCommand != null) autonomousCommand.cancel();

		flyWheel.setEncPosition(0);

		SmartDash();
		approachTimer.stop();

		//Live Stream
		//IMAQdxStartAcquisition(session);

		//ensure solenoids are disengaged
		climberAngle.set(false);
		climbLock.set(false);

		leftPTO.set(DoubleSolenoid.Value.kForward);
		rightPTO.set(DoubleSolenoid.Value.kReverse);
		ptoEngaged = true;
		statePTO = false;
		gyroDisable = false;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		// Scheduler.getInstance().run();
		disableGyro();
		SmartDash();

		flyWheelSpeed = flyWheel.getSpeed();

		if (driver.getRawButton(7)) {
			visionXAlign();
		} else {
			if (vertGyro.getAngle() < antiTipAngle && ptoEngaged == true
					&& !gyroDisable)	//pto is not engaged
					{
				left.set(-.65);
				right.set(.65);
			} else {
				Drivebase();
			}
		}
		if (oper.getRawButton(10)) {
			flyWheel.changeControlMode(TalonControlMode.PercentVbus);
			flyWheel.set(1);
			if (oper.getRawButton(14)) {
				setIntake(1);
			} else if (!oper.getRawButton(14)) {
				setIntake(0);
			}
		} else {
			Shooter();
		}
		Gyro();
		Arm();
		//stops hood from going past point
		if (hood.getEncPosition() < -1650) {
			hood.set(normPID(-1300, hood.getEncPosition(), .0048, 0));
		} else
			Hood();
		Climber();
//		LiveStream();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// LiveWindow.run();
	}

	void Moat(int pos) {
		switch (pos) {
		case 1:
			autonTime = 1.7; // straight pos 2
			break;
		case 2:
			autonTime = 1.7; // curve pos 2
			break;
		case 3:
			autonTime = 1.15;
			angleTar = 22;
			break;
		case 4:
			autonTime = 1.0;
			angleTar = -3;
			break;
		case 5:
			autonTime = 1.8;
			angleTar = -25;
			break;
		}
		switch (moatState) {
		case moatIntake:
			ballHit = bannerInner.get();
			left.set(0);
			right.set(0);
			setIntake(1);
			armPID(0);
			if (ballHit == true) {
				setIntake(0);
				moatState = moatEnum.moatFirstBump;
				vertGyro.reset();
			}
			break;
		case moatFirstBump:
			armPID(0);
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(0.5 + gyroPID(0));
				right.set(-0.5 + gyroPID(0));
				antiTipEngaged = false;
			}
			if (vertGyro.getRate() >= 50 && (antiTipEngaged == false)) {
				left.set(0);
				right.set(0);
				Timer.delay(.5);
				approachTimer.reset();
				approachTimer.start();
				moatState = moatEnum.moatBack;
			}
			break;
		case moatBack:
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(-.15 + gyroPID(0));
				right.set(.15 + gyroPID(0));
				antiTipEngaged = false;
			}
			if (approachTimer.get() >= .75 && (antiTipEngaged == false)) {
				left.set(0);
				right.set(0);
				moatState = moatEnum.moatSecondBump;
				approachTimer.reset();
				Timer.delay(.5);
			}
			break;
		case moatSecondBump:
			armPID(0);
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(1);
				right.set(-1);
				antiTipEngaged = false;
			}
			if (vertGyro.getRate() < -95 && (antiTipEngaged == false)) {
				left.set(0);
				right.set(0);
				if (position == 2) {
					Timer.delay(.5);
					posTwoTimer.start();
					moatState = moatEnum.moatDrive2;
				} else
					moatState = moatEnum.moatDrive;
				approachTimer.reset();
			}
			break;
		case moatDrive:
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(.6 + gyroPID(0));
				right.set(-.6 + gyroPID(0));
				antiTipEngaged = false;
			}
			if (approachTimer.get() >= autonTime && (antiTipEngaged == false)) {
				left.set(0);
				right.set(0);
				moatState = moatEnum.moatRotate;
			}
			break;
		case moatDrive2:
			if (secondPos())
				moatState = moatEnum.moatRotate;
			break;

		case moatRotate:
			if (angleTar < 0) {
				left.set(-.5);
				right.set(-.5);
			} else {
				left.set(.5);
				right.set(.5);
			}
			if (absVal(horzGyro.getAngle() - angleTar) < 5) {
				left.set(0);
				right.set(0);
				moatState = moatEnum.moatShoot;
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
			autonTime = 1.7; // straight pos 2
			break;
		case 2:
			autonTime = 1.7; // curvy pos 2
			break;
		case 3:
			autonTime = 1.15;
			angleTar = 22;
			break;
		case 4:
			autonTime = 1.0;
			angleTar = -3;
			break;
		case 5:
			autonTime = 1.8;
			angleTar = -25;
			break;
		}

		SmartDash();

		switch (rockWallState) {
		case rockWallIntake:
			left.set(0);
			right.set(0);
			setIntake(1);
			ballHit = bannerInner.get();
			armPID(0);
			if (ballHit == true) {
				setIntake(0);
				rockWallState = rockWallEnum.rockWallApproach;
			}
			break;
		case rockWallApproach:
			armPID(0);
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(.65 + gyroPID(0));
				right.set(-.65 + gyroPID(0));
				antiTipEngaged = false;
			}
			if (vertGyro.getRate() >= 95 && (antiTipEngaged == false)) {
				Timer.delay(.3);
				vertGyro.getRate();
				rockWallState = rockWallEnum.rockWallCross;
			}
			break;
		case rockWallCross:
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(.8 + gyroPID(0));
				right.set(-.8 + gyroPID(0));
				antiTipEngaged = false;
			}
			armPID(0);
			if (vertGyro.getRate() <= -130 && (antiTipEngaged == false)) {
				left.set(0);
				right.set(0);
				Timer.delay(1);
				approachTimer.reset();
				approachTimer.start();
				if (position == 2) {
					rockWallState = rockWallEnum.rockWallDrive2;
					posTwoTimer.start();
				} else
					rockWallState = rockWallEnum.rockWallDrive;
			}
			break;
		case rockWallDrive:
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(.63 + gyroPID(0));
				right.set(-.63 + gyroPID(0));
				antiTipEngaged = false;
			}
			if (approachTimer.get() >= autonTime && (antiTipEngaged == false)) {
				setSpeed(0);
				rockWallState = rockWallEnum.rockWallRotate;
			}
			break;
		case rockWallDrive2:
			if (secondPos())
				rockWallState = rockWallEnum.rockWallRotate;
			break;
		case rockWallRotate:
			if (angleTar < 0) {
				left.set(-.5);
				right.set(-.5);
			} else {
				left.set(.5);
				right.set(.5);
			}
			if (absVal(horzGyro.getAngle() - angleTar) < 5) {
				left.set(0);
				right.set(0);

				rockWallState = rockWallEnum.rockWallShoot;
			}
			break;
		case rockWallShoot:
			autonShoot();
			break;
		}
	}

	void RoughTerrain(int pos) {
		switch (pos) {
		case 1:
			autonTime = 0;
			break;
		case 2:
			autonTime = 5.8; // 4.28=states
			angleTar = 40;
			break;
		case 3:
			autonTime = 4; // 4
			angleTar = 22;
			break;
		case 4:
			autonTime = 4.7; // 4
			angleTar = -3;
			break;
		case 5:
			autonTime = 5.7; // 4.15
			angleTar = -25;
			break;
		}

		switch (roughTerrainState) {
		case roughTerrainIntake:
			approachTimer.reset();
			approachTimer.start();
			setIntake(1);
			ballHit = bannerInner.get();
			armPID(0);
			left.set(0);
			right.set(0);
			if (ballHit == true) {
				setIntake(0);
				roughTerrainState = roughTerrainEnum.roughTerrainDrive;
			}
			break;
		case roughTerrainDrive:
			left.set(.55 + gyroPID(0));
			right.set(-.55 + gyroPID(0));
			if (approachTimer.get() >= autonTime) {
				setSpeed(0);
				roughTerrainState = roughTerrainEnum.roughTerrainRotate;
			}
			break;
		case roughTerrainRotate:
			if (angleTar < 0) {
				left.set(-.5);
				right.set(-.5);
			} else {
				left.set(.5);
				right.set(.5);
			}
			if (absVal(horzGyro.getAngle() - angleTar) < 5) {
				left.set(0);
				right.set(0);
				roughTerrainState = roughTerrainEnum.roughTerrainShoot;
			}
			break;
		case roughTerrainShoot:
			autonShoot();
			break;
		}
	}

	void Ramparts(int pos) {
		switch (pos) {
		case 1:
			autonTime = 1.6; // straight pos 2
			gyroTar = 0;
			break;
		case 2:
			autonTime = 2; // 1.6=state; curved pos 2
			gyroTar = 27;
			break;
		case 3:
			autonTime = 1.15; // 1.15
			break;
		case 4:
			autonTime = 1.0; // 1
			break;
		case 5:
			autonTime = 1.6;
			gyroTar = -65;
			gyroKP = .01;
			angleTar = -10;
			break;
		case 6:
			autonTime = 1.6;
			gyroTar = -20;
			gyroKP = .01;
			angleTar = -10;
			break;
		case 7:
			autonTime = 5.75;
			break;
		}

		switch (rampartsState) {
		case rampartsIntake:
			left.set(0);
			right.set(0);
			approachTimer.reset();
			approachTimer.start();
			setIntake(1);
			ballHit = bannerInner.get();
			armPID(0);
			if (ballHit == true && pos != 7) {
				setIntake(0);
				rampartsState = rampartsEnum.rampartsApproach;
			} else if (ballHit == true) {
				setIntake(0);
				rampartsState = rampartsEnum.rampartsRush;
			}
			break;
		case rampartsRush:
			if (approachTimer.get() < autonTime) {
				left.set(.7);
				right.set(-.7);
			} else {
				left.set(0);
				right.set(0);
				rampartsState = rampartsEnum.rampartsShoot;
			}
			break;
		case rampartsApproach:
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(.7);
				right.set(-.7);
				antiTipEngaged = false;
			}
			if (vertGyro.getRate() <= -75 && (antiTipEngaged == false)) {
				Timer.delay(.3);
				vertGyro.getRate();
				rampartsState = rampartsEnum.rampartsCross;
			}
			if (approachTimer.get() >= 5.5) {
				left.set(0);
				right.set(0);
				rampartsState = rampartsEnum.rampartsRotate;
			}
			break;
		case rampartsCross:
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(.55);
				right.set(-.75);
				antiTipEngaged = false;
			}
			if (vertGyro.getRate() >= 75 && (antiTipEngaged == false)) {
				rampartsFlag = true;
				vertGyro.getRate();
			}
			if (vertGyro.getRate() <= -75 && rampartsFlag && (antiTipEngaged == false)) {
				vertGyro.getRate();
				left.set(0);
				right.set(0);
				Timer.delay(.75);
				rampartsState = rampartsEnum.rampartsDrive;
				approachTimer.reset();
				approachTimer.start();
			}
			if (approachTimer.get() >= 4.25) {
				left.set(0);
				right.set(0);
				rampartsState = rampartsEnum.rampartsRotate;
			}
			break;
		case rampartsDrive:
			if (vertGyro.getAngle() < antiTipAngle) {
				left.set(-.4);
				right.set(.4);
				antiTipEngaged = true;
			} else {
				left.set(.7 + gyroPID(gyroTar));
				right.set(-.7 + gyroPID(gyroTar));
				antiTipEngaged = false;
			}
			if (approachTimer.get() > autonTime && (antiTipEngaged == false)) {
				left.set(0);
				right.set(0);
				if (pos == 5 || pos == 6) {
					rampartsState = rampartsEnum.rampartsRotate;
					rampartsTimer.reset();
					rampartsTimer.start();
				} else {
					rampartsState = rampartsEnum.rampartsShoot;
				}
			}
			break;
		case rampartsRotate:
			if (angleTar < 0 && pos != 5) {
				left.set(-.5);
				right.set(-.5);
			} else {
				left.set(.5);
				right.set(.5);
			}
			if (absVal(horzGyro.getAngle() - angleTar) < 5) {
				left.set(0);
				right.set(0);
				rampartsState = rampartsEnum.rampartsShoot;
			}
			break;
		case rampartsShoot:
			autonShoot();
			break;
		}
	}

	// linear turn auton on second position defense
	boolean secondPos() {
		switch (posTwoState) {
		case turn1:
			left.set(.5);
			right.set(.5);
			if (absVal(horzGyro.getAngle() - 45) < 5.0) {
				posTwoState = posTwo.forward;
				posTwoTimer.reset();
			}
			return false;

		case forward:
			left.set(.67 + gyroPID(45));
			right.set(-.67 + gyroPID(45));
			if (posTwoTimer.get() > autonTime)
				posTwoState = posTwo.turn2;
			return false;

		case turn2:
			left.set(-.33);
			right.set(-.33);
			if (absVal(horzGyro.getAngle()) < 5.0)
				return true;
			else
				return false;

		}
		return false;
	}

	void Spy() {
		cogx = table.getNumber("COG_X", 1000);
		cogy = table.getNumber("COG_Y", 1000);
		switch (spyState) {
		case spyIntake:
			approachTimer.reset();
			approachTimer.start();
			setIntake(1);
			ballHit = bannerInner.get();
			armPID(0);
			if (ballHit == true) {
				setIntake(0);
				spyState = spyEnum.spyRotate;
			}
			break;
		case spyRotate:
			// right.set(-.3);
			visionXAlign();
			if ((cogx < (cogxTar + 5) && cogx > (cogxTar - 5)) && visionPID(cogxTar, cogx) == 0)/// approachTimer.get()
																								/// >=
																								/// 1
			{

				setSpeed(0);
				spyState = spyEnum.spyShoot;
			}
			break;
		case spyShoot:
			autonShoot = autonShootState.autonReving;
			autonShoot();
			break;
		}
	}

	// State machine for aligning and firing in auton
	void autonShoot() {
		cogx = table.getNumber("COG_X", 1000);
		cogy = table.getNumber("COG_Y", 1000);

		if (autonFireSelected == 1) {
			switch (autonShoot) {
			case autonAligning:
				switch (position) {
				case 1:
				case 2:
				case 3:
					turnRight = true;
					break;
				case 4:
				case 6:
					turnRight = false;
					break;
				case 5:
					if (defense == 5)
						turnRight = true;
					else
						turnRight = false;
					break;
				default:
					break;
				}
				if (turnRight && cogx == 0) {
					left.set(.5);
					right.set(.5);
				} else if (!turnRight && cogx == 0) {
					left.set(-.5);
					right.set(-.5);
				} else {
					left.set(0);
					right.set(0);
					autonShoot = autonShootState.autonAligning2;
				}
				break;
			case autonAligning2:
				visionXAlign();
				// stops hood from going past point
				if (hood.getEncPosition() < -1650) {
					hood.set(normPID(-1300, hood.getEncPosition(), .0048, 0));
				} else
					visionYAlign();
				if (cogx < (cogxTar + 5) && cogx > (cogxTar - 5) && hood.getSpeed() == 0) {
					autonShoot = autonShootState.autonReving;
				}
				break;
			case autonReving:
				visionXAlign();
				// stops hood from going past point
				if (hood.getEncPosition() < -1650) {
					hood.set(normPID(-1300, hood.getEncPosition(), .0048, 0));
				} else
					visionYAlign();

				setFlyWheelRPM(flyWheelTarget);

				if (isAtSpeed) {
					// switch states because we are completely reved up
					resetFiringState.reset();
					resetFiringState.start();
					autonShoot = autonShootState.autonFiring;

				}
				break;
			case autonFiring:
				if (resetFiringState.get() < 1) {
					setIntake(1);
				} else {
					setIntake(0);
					stopFlyWheel();
				}
				// If the ball is still at the banner sensor, push it into the
				// shooter

				break;
			}
		} else {
			left.set(0);
			right.set(0);
		}
	}

	void disableGyro() {
		// manual disable antitip
		if (driver.getRawButton(13) && !stateGyro) // after second push
		{
			releasedGyro = false;
			gyroDisable = false; // Enable anti-tip
		} else if (driver.getRawButton(13) && stateGyro) // first push
		{
			releasedGyro = false;
			gyroDisable = true; // disable anti-tip
		} else if (!driver.getRawButton(13) && !releasedGyro) // happens first
		{
			stateGyro = !stateGyro;
			releasedGyro = true;
		}
	}

	// Function to display various data to SmartDashboard
	void SmartDash() {

		string cogxString = "COG_X " + std.Value.to_string(table.getNumber("COG_X", 1000));
		SmartDashboard.Value.PutString("DB/String 0", cogxString);

		string cogyString = "COG_Y " + std.Value.to_string(table.getNumber("COG_Y", 1000));
		SmartDashboard.Value.PutString("DB/String 1", cogyString);

		string pto;

		// Driving mode
		if (ptoEngaged == 1) {
			pto = "true";
			SmartDashboard.Value.PutBoolean("DB/LED 1", !ptoEngaged);
		}
		// Climbing mode
		if (ptoEngaged == 0) {
			pto = "false";
			SmartDashboard.Value.PutBoolean("DB/LED 1", !ptoEngaged);
		}
		// string PTOString = "PTO: " + pto;
		// SmartDashboard.Value.PutString("DB/String 2", PTOString);

		string bann;

		// Ball held
		if (bannerInner.get()) {
			bann = "true";
			SmartDashboard.Value.PutBoolean("DB/LED 0", ballHit);
		}
		// No Ball
		if (!bannerInner.get()) {
			SmartDashboard.Value.PutBoolean("DB/LED 0", ballHit);
			bann = "false";
		}
		// string ballHitString = "Ball Held: " + bann;
		// SmartDashboard.Value.PutString("DB/String 3", ballHitString);

		string HoodString = "Hood: " + std.Value.to_string(hood.getEncPosition());
		SmartDashboard.Value.PutString("DB/String 4", HoodString);

		// SmartDashboard.Value.PutString("DB/String 2",
		// std.Value.to_string(horzGyro.getAngle()));

		string armString = "Arm: " + std.Value.to_string(arm.getEncPosition());
		SmartDashboard.Value.PutString("DB/String 3", armString);
		// string shooterString = "flywheel: "
		// + std.Value.to_string(flyWheel.getSpeed());
		// SmartDashboard.Value.PutString("DB/String 2", shooterString);

		// string horzString = "horzgyro: " +
		// std.Value.to_string(horzGyro.getAngle());
		// SmartDashboard.Value.PutString("DB/String 2", horzString);

		string gyroString = "VertGyro: " + std.Value.to_string(vertGyro.getAngle());
		SmartDashboard.Value.PutString("DB/String 2", gyroString);

		// SmartDashboard.Value.PutString("DB/String 2",
		// (absVal(flyWheelSpeed) > 40
		// && (oper.getRawButton(10) || oper.getRawButton(4))) ?
		// "Shooting" : "Standby");

		// SmartDashboard.Value.PutString("DB/String 2", PTOString);

	}

	void AutonSelect() {
		string defString = SmartDashboard.Value.GetString("DB/String 5", "2");
		defenseSelected = std.Value.stoi(defString); // what defense

		string posString = SmartDashboard.Value.GetString("DB/String 6", "2");
		positionSelected = std.Value.stoi(posString); // what position

		string autonFireString = SmartDashboard.Value.GetString("DB/String 7", "1");
		autonFireSelected = std.Value.stoi(autonFireString);

		string autonWaitString = SmartDashboard.Value.GetString("DB/String 8", "0");
		autonWaitSelected = std.Value.stod(autonWaitString);
	}

	// Function to capture images and send to DashBoard
	void LiveStream() {
		 IMAQdxStartAcquisition(session);
		// Acquire Images
		 IMAQdxGrab(session, frame, true, NULL);

		// Send Image to DashBoard
		 CameraServer::GetInstance().setImage(frame);
	}

	// Function for operating drivebase. Allows for tank/Arcade option select
	// Includes gyro correction. Must be tested
	void Drivebase() {
		double speed = deadband(-driver.getY());
		double turn = deadband(driver.getZ());

		if (!driver.getRawButton(8)) {
			target = xAngle;
		}

		if (climberSwitch.get() == false && !ptoEngaged) {
			left.set(0);
			right.set(0);
		}

		else {
			left.set(speed + turn + gyroPID(target));
			right.set(-speed + turn + gyroPID(target));
		}
	}

	// Updates angle variables, resets x gyro when full rotation is completed
	void Gyro() {
		xAngle = horzGyro.getAngle();
		yAngle = vertGyro.getAngle();
		if (xAngle == 360 || xAngle == -360) {
			xAngle = 0;
		}
		if (driver.getRawButton(6)) {
			horzGyro.reset();
			vertGyro.reset();
		}
	}

	// Function for operating shooter
	void Hood() {

		// stops hood from going past point
		if (hood.getEncPosition() < -1650) {
			hood.set(normPID(-1300, hood.getEncPosition(), .0048, 0));
		} else {
			// reset hood when its all the way down
			if (zeroHood.get() == false) {

				if (oper.getRawButton(6) == true) {
					hood.set(0);
				}
				hood.setEncPosition(0);
				hood.set(normPID(50, hood.getEncPosition(), .0048, 0));
			} else if (driver.getRawButton(5)) // move hood up
			{
				visionYAlign();
			} else {
				if (oper.getRawButton(6)) {
					hood.set(.75);
				} else if (oper.getRawButton(8)) {
					hood.set(-.75);
				} else
					hood.set(0);
			}

			// Far far shot (position 2 outer works)
			if (oper.getRawButton(9)) {
				hood.set(normPID(640, hood.getEncPosition(), .0048, 0));// 610
				flyWheelTarget = 4050;
			}
			// lob shot, arm out
			if (oper.getPOV() == 0) {
				hood.set(normPID(815, hood.getEncPosition(), .0048, 0));
				flyWheelTarget = 3315;
			}
			// far shot
			if (oper.getPOV() == 90) {
				hood.set(normPID(624, hood.getEncPosition(), .0048, 0), 0);// 679
				flyWheelTarget = 3950;
			}
			// mid shot
			if (oper.getPOV() == 180) {
				hood.set(normPID(655, hood.getEncPosition(), .0048, 0), 0);// 660
				flyWheelTarget = 3600;
			}
			// close shot, edge of batter
			if (oper.getPOV() == 270) {
				hood.set(normPID(835, hood.getEncPosition(), .0048, 0), 0);// 855
				flyWheelTarget = 3600;
			}
			// Batter shot, fully up touching tower
			if (oper.getRawButton(14) && !oper.getRawButton(10)) {
				hood.set(normPID(1125, hood.getEncPosition(), .0048, .0025)); // 1320=hard,1340=soft
				flyWheelTarget = 3000; // 3600=hard,3000=soft
			}
		}
	}

	// state machine for operating shooter
	void Shooter() {
		/*
		 * 1. Operator uses intake to claim a ball and can spit out as well. Arm
		 * comes down automatically. 2. Banner sensor is hit to stop ball. Ball
		 * can be spit out. Arm comes all the way up. 3. Operator revs up
		 * shooter and holds intake, intake is not activated until shooter is up
		 * to speed.
		 */
		ballHit = bannerInner.get();
		if (ballHit || ledBool) {
			ledSpike.set(Relay.Value.kOn);
		} else {
			ledSpike.set(Relay.Value.kOff);
		}
		switch (currShootState) {

		case intake:
			stopFlyWheel();
			isAtSpeed = false;

			// In this mode, the robot is waiting to obtain a ball
			if (oper.getRawButton(5)) {
				setIntake(-1);

				// If the regurgitation button is pressed, pull a ball in
			} else if (oper.getRawButton(7)) {
				setIntake(1);
				if (!oper.getRawButton(3)) {
					pidArm = true;
					armPID(-1250);
				}
				// If the intake button is pressed, spit the ball out
			} else {
				setIntake(0);
				pidArm = false;
			}

			if (ballHit == true) {
				// switch states because we've fully loaded a ball
				setIntake(0);
				currShootState = shootState.reving;
			}

			break;
		case reving:
			// In this mode, the robot is waiting for the shooter to reach 24K
			// speed
			if (oper.getRawButton(7) && !oper.getRawButton(3)) {
				armPID(0);
				pidArm = true;
			} else {
				pidArm = false;
			}

			if (oper.getRawButton(5)) {
				setIntake(-1);
				currShootState = shootState.intake;
				// Regurgitate, and go to intake because we're spitting out the
				// ball

			} else if (oper.getRawButton(2)) {
				intakeOuter.set(-1);

			} else if (!oper.getRawButton(2)) {
				intakeOuter.set(0);
			}
			if (oper.getRawButton(4)) {
				setFlyWheelRPM(flyWheelTarget);
				// This runs the rev up process, PID'ing the shooter to 24000.
			} else {
				stopFlyWheel();
			}

			if (isAtSpeed && oper.getRawButton(7)) // isAtSpeed
			{
				// switch states because we are completely reved up
				currShootState = shootState.firing;

				resetFiringState.reset();
				resetFiringState.start();
			}
			break;
		case firing:
			if (ballHit == true) {
				// If the ball is still at the banner sensor, push it into the
				// shooter
				intakeInner.set(-1);
			}
			if (resetFiringState.get() >= 1.5) {
				currShootState = shootState.intake;
			}
			break;
		}

	}

	void setFlyWheelRPM(double rpm) {
		flyWheel.changeControlMode(TalonControlMode.Speed);
		flyWheel.set(rpm);
		flyWheelSpeed = flyWheel.getSpeed();
		double flyWheelError = absVal(rpm - flyWheelSpeed) - 360;
		if (flyWheelError <= 500) {
			isAtSpeed = true;
		} else {
			isAtSpeed = false;
		}
	}

	void stopFlyWheel() {
		flyWheel.changeControlMode(TalonControlMode.PercentVbus);
		flyWheel.set(0);
	}

	// function for operating arm
	void Arm() {
		/*
		 * Arm comes up full speed, down half speed (0.5)
		 *
		 * Square - arm comes up to 0 X - arm becomes level with floor
		 */

		// left stick
		intakeArmSpeed = deadband(oper.getY());
		if (arm.getEncPosition() > -650 && intakeArmSpeed < 0) {
			intakeArmSpeed = 0;
		}
		if (oper.getRawButton(13)) {
			arm.setEncPosition(0);

		}

		if ((deadband(oper.getRawAxis(5)) != 0) && (deadband(oper.getRawAxis(5)) < 0)) // move
																						// intake
																						// arm
																						// up
																						// or
																						// down
		{
			arm.set(deadband(oper.getRawAxis(5)) * .8);
		} else if (intakeArmSpeed < 0) {
			arm.set(intakeArmSpeed * .5);
			pidArm = false;
		} else if (intakeArmSpeed > 0) {
			arm.set(intakeArmSpeed * 0.3);
			pidArm = false;
		} else if (oper.getRawButton(3)) {
			armPID(0);
		} else if (oper.getRawButton(1)) {
			armPID(-1000);
		} else if (oper.getRawButton(2)) {
			armPID(-1250);
		} else if (!pidArm) {
			arm.set(0);
		}
	}

	// Sets speeds of both drive motors to for/back
	void Climber() {
		// string climbString = "climb: " + std.Value.to_string(myclimb);
		// SmartDashboard.Value.PutString("DB/String 2", climbString);
		/*
		 * Square - Toggle for angle X - Toggle for release Circle - Toggle for
		 * PTOs
		 */

		// climber angle Toggle
		if (driver.getRawButton(1) && !stateAngle) // after second push
		{
			releasedAngle = false;
			climberAngle.set(false); // becomes 0 degrees
			myclimb = 1;

		} else if (driver.getRawButton(1) && stateAngle) // first push
		{
			releasedAngle = false;
			climberAngle.set(true); // becomes 30 degrees
			myclimb = 2;

		} else if (!driver.getRawButton(1) && !releasedAngle) // happens first
		{
			stateAngle = !stateAngle;
			releasedAngle = true;
			myclimb = 3;
		}

		// climber release Toggle
		if (driver.getRawButton(2) && !stateLock) // after second push
		{
			releasedLock = false;
			// climbLock.set(0);//re-engages lock
			myclimb = 4;

		} else if (driver.getRawButton(2) && stateLock) // first push
		{
			releasedLock = false;
			climbLock.set(false); // releases lock
			ledBool = true;
			myclimb = 5;
		} else if (!driver.getRawButton(2) && !releasedLock) // happens first
		{
			stateLock = !stateLock;
			releasedLock = true;
			myclimb = 6;
		}

		// climberPTO Toggle
		if (driver.getRawButton(3) && !statePTO) // after second push
		{
			releasedPTO = false;
			ptoEngaged = false;
			leftPTO.set(DoubleSolenoid.Value.kReverse);
			rightPTO.set(DoubleSolenoid.Value.kForward); // climb mode
			Timer.delay(.25);
			releasedAngle = false;
			climberAngle.set(false); // becomes 0 degrees
			myclimb = 7;

		} else if (driver.getRawButton(3) && statePTO) // first push
		{
			releasedPTO = false;
			ptoEngaged = true;
			leftPTO.set(DoubleSolenoid.Value.kForward);
			rightPTO.set(DoubleSolenoid.Value.kReverse); // drive mode
			myclimb = 8;

		} else if (!driver.getRawButton(3) && !releasedPTO) // happens first
		{
			statePTO = !statePTO;
			releasedPTO = true;
			myclimb = 9;
		}

	}

	// PID for x-axis alignment
	void visionXAlign() {
		cogx = table.getNumber("COG_X", 1000);
		left.set(-visionPID(cogxTar, cogx));
		right.set(-visionPID(cogxTar, cogx));
	}

	// PID for hood alignment
	// TODO: find actual values for this
	void visionYAlign() {
		cogy = table.getNumber("COG_Y", 217);
		if (cogy < 28) // backed up to defenses at courtyard
		{
			hood.set(normPID(640, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 55) {
			hood.set(normPID(640, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 82) {
			hood.set(normPID(640, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 110) {
			hood.set(normPID(650, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 140) {
			hood.set(normPID(750, hood.getEncPosition(), .0048, 0));
		} else if (cogy < 165) {
			hood.set(normPID(830, hood.getEncPosition(), .0048, 0));
		} else
			hood.set(normPID(880, hood.getEncPosition(), .0048, 0));

	}

	double gyroPID(double target) {
		double gyroError;

		gyroError = target - xAngle;
		return gyroError * gyroKP;

	}

	// PID for x-axis vision alignment use
	// TODO: DELETE THIS ONE AND UNCOMMENT THE OTHER ONE FOR IRI
	// TODO: YES YOU GWELLY
	// TODO: IM NOT KIDDING
	// TODO: IF I COME TO PITS AND THIS ISNT DONE I S2G

	double visionPID(double cogTar, double cogPos) {
		cogPIDError = cogTar - cogPos;
		double cogPIDSpeed;

		if (cogPIDError > 40) {
			cogPIDSpeed = .38; // was .37
		} else if (cogPIDError > 5) {
			cogPIDSpeed = .31; // was .27
		} else if (cogPIDError < -40) {
			cogPIDSpeed = -.38; // was -.37
		} else if (cogPIDError < -5) {
			cogPIDSpeed = -.31; // -.27
		} else {
			cogPIDSpeed = 0;
		}
		return (cogPIDSpeed);
	}

	// double visionPID(double cogTar, double cogPos)
	// {
	// cogPIDError = cogTar - cogPos;
	// double cogPIDSpeed;
	//
	// if (cogPIDError > 40)
	// {
	// cogPIDSpeed = .38;//was .37
	// }
	// else if (cogPIDError > 5)
	// {
	// cogPIDSpeed = .34;//was .27
	// }
	// else if (cogPIDError < -40)
	// {
	// cogPIDSpeed = -.38;//was -.37
	// }
	// else if (cogPIDError < -5)
	// {
	// cogPIDSpeed = -.34;//-.27
	// }
	// else
	// {
	// cogPIDSpeed = 0;
	// }
	// return (cogPIDSpeed);
	// }

	// PID for arm
	void armPID(double armTar) {
		double armCur = arm.getEncPosition();
		double armSpeed = -normPID(armTar * 4, -armCur, .00015, 0);
		if (armSpeed > .5) {
			armSpeed = .5;
		}
		if (armSpeed < -.5) {
			armSpeed = -.5;
		}
		arm.set(armSpeed);

	}

	// Standard PID function
	double normPID(double myTar, double myPos, double myP, double myI) {
		PIDError = myTar + myPos;
		cumPIDError = PIDError;
		double PIDPout = PIDError * myP;
		double PIDIout = PIDError * myI;
		double PIDSpeed = (PIDPout + PIDIout);

		if (absVal(PIDError) < 5) {
			PIDSpeed = 0;
		}
		return (PIDSpeed);
	}

	// Sets all wheels to drive at same speed
	void setSpeed(double speed) {
		left.set(speed);
		right.set(-speed);
	}

	// Sets speed of intake motors
	void setIntake(double speed) {
		intakeInner.set(-speed);
		intakeOuter.set(speed);
	}

	// Nulls idle stick input at 0.08
	double deadband(double input) {

		if (absVal(input) < .08) {
			return 0;
		}
		return input;
	}

	// choosing position for auton
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
		if (positionSelected == 6)
			position = 6;
		if (positionSelected == 7)
			position = 7;
	}

	// choosing defense to cross in auton
	void defenseChoosing() {
		if (defenseSelected == 1) {
			defense = 1;
			moatState = moatEnum.moatIntake;
		}
		if (defenseSelected == 2) {
			defense = 2;
		}
		if (defenseSelected == 3) {
			defense = 3;
			rockWallState = rockWallEnum.rockWallIntake;
		}
		if (defenseSelected == 4) {
			defense = 4;
			roughTerrainState = roughTerrainEnum.roughTerrainIntake;
		}
		if (defenseSelected == 5) {
			defense = 5;
			rampartsState = rampartsEnum.rampartsIntake;
		}
		if (defenseSelected == 6) {
			defense = 6;
			spyState = spyEnum.spyIntake;
		}
	}

	// C++ abs function sucks
	double absVal(double input) {
		if (input < 0)
			return -input;
		return input;
	}
}
