
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.CANTalon.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.*;

/**
 * Khaleesi Demo Mode
 * 
 * @author evanj
 * @version 6/16/2016
 */
public class Robot extends IterativeRobot {

	AnalogGyro horzGyro;
	ADXRS450_Gyro vertGyro;

	CANTalon left, left2, right, right2;
	CANTalon flywheel, flywheel2, arm, hood;
	CANTalon intakeInner, intakeOuter;

	DigitalInput bannerInner;
	DigitalInput zeroHood;

	DoubleSolenoid leftPTO;
	DoubleSolenoid rightPTO;
	Solenoid climbLock;

	Joystick driver;

	Relay ledSpike;
	boolean ledOn = false;

	Timer resetFiringState;

	boolean ballHit = false;
	boolean hoodAdjusted = false;
	boolean isAtSpeed = false;
	boolean ledBool = false;
	boolean pidArm = false;
	boolean stateLock = true, releasedLock = true;
	boolean statePTO = false, releasedPTO = true;
	boolean ptoEngaged = true;

	double armKP, armKI;
	double cumPIDError = 0;
	double flywheelSpeed;
	double flywheelTarget = 2600;
	double hoodTarget;
	double intakeArmSpeed = 0;
	double PIDError = 0;
	double SPEEDCAP = 1;
	double xAngle = 0;

	enum shootState {
		intake, reving, firing
	};

	shootState currShootState;

	int session;
	Image frame;

	/**
	 * Method that is called once upon robot boot.
	 */
	public void robotInit() {
		left = new CANTalon(14);
		left2 = new CANTalon(15);
		left2.changeControlMode(TalonControlMode.Follower);
		left2.set(14);

		right = new CANTalon(12);
		right2 = new CANTalon(13);
		right2.changeControlMode(TalonControlMode.Follower);
		right2.set(12);

		arm = new CANTalon(2);
		hood = new CANTalon(7);

		flywheel = new CANTalon(0);
		flywheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		flywheel.configEncoderCodesPerRev(1000);
		flywheel.changeControlMode(TalonControlMode.Speed);
		flywheel.setProfile(1);

		flywheel2 = new CANTalon(1);
		flywheel2.changeControlMode(TalonControlMode.Follower);
		flywheel2.set(0);

		driver = new Joystick(0);

		intakeInner = new CANTalon(6);
		intakeOuter = new CANTalon(3);

		bannerInner = new DigitalInput(0);
		zeroHood = new DigitalInput(1);

		resetFiringState = new Timer();

		rightPTO = new DoubleSolenoid(0, 3);
		leftPTO = new DoubleSolenoid(1, 2);
		climbLock = new Solenoid(4);

		ledSpike = new Relay(0);

		horzGyro = new AnalogGyro(0);
		vertGyro = new ADXRS450_Gyro();

		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		session = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);

	}

	public void disabledInit() {
		leftPTO.set(Value.kForward);
		rightPTO.set(Value.kReverse);

		ledSpike.set(Relay.Value.kOff);
		ledBool = false;
	}

	/**
	 * Called once when teleop is enabled
	 */
	public void teleopInit() {
		flywheel.setEncPosition(0);
		leftPTO.set(Value.kForward);
		rightPTO.set(Value.kReverse);

		currShootState = shootState.intake;

		if (SmartDashboard.getBoolean("DB/Button 1"))
			SPEEDCAP = 0.4;
		else
			SPEEDCAP = 1;

		hoodTarget = hood.getPosition();

	}

	/**
	 * Called periodically during teleop mode
	 */
	public void teleopPeriodic() {

		NIVision.IMAQdxGrab(session, frame, 1);
		CameraServer.getInstance().setImage(frame);

		Arm();
		Drivebase();
		Hood();
		Shooter();
		Climber();
		// SmartDashboard.putNumber("flywheel Speed", flywheel.getSpeed());
		// SmartDashboard.putNumber("Hood Pos", hood.getPosition());
		// SmartDashboard.putBoolean("Zero Hood", zeroHood.get());

		Timer.delay(0.005);

	}

	/**
	 * Control for intake arm
	 */
	void Arm() {

		if (driver.getRawButton(13)) {
			arm.setEncPosition(0);

		}

		if (arm.getEncPosition() > 0 && driver.getRawButton(6)) {
			arm.set(0);
		} else if (driver.getRawButton(6)) {
			arm.set(-.5);
			pidArm = false;
		} else if (driver.getRawButton(8)) {
			arm.set(0.3);
			pidArm = false;
		} else if (driver.getRawButton(3)) {
			armPID(-1250);
		} else if (!pidArm) {
			arm.set(0);
		}
	}

	/**
	 * Moves arm to specified position
	 * 
	 * @param armTar
	 *            target in encoder ticks
	 */
	void armPID(double armTar) {
		double armCur = arm.getEncPosition();
		double armSpeed = -normPID(armTar * 4, -armCur, .0003, 0);
		if (armSpeed > .5) {
			armSpeed = .5;
		}
		if (armSpeed < -.5) {
			armSpeed = -.5;
		}
		arm.set(armSpeed);

	}

	public void Climber() {
		// climber release Toggle
		if (driver.getRawButton(10) && !stateLock)// after second push
		{
			releasedLock = false;
			climbLock.set(false);// re-engages lock

		} else if (driver.getRawButton(10) && stateLock)// first push
		{
			releasedLock = false;
			climbLock.set(true);// releases lock
			ledBool = true;

		} else if (!driver.getRawButton(10) && !releasedLock)// happens first
		{
			stateLock = !stateLock;
			releasedLock = true;

		}

		// climberPTO Toggle
		if (driver.getRawButton(9) && !statePTO)// after second push
		{
			releasedPTO = false;
			ptoEngaged = false;
			leftPTO.set(Value.kReverse);
			rightPTO.set(Value.kForward);// climb mode
			Timer.delay(.25);

		} else if (driver.getRawButton(9) && statePTO)// first push
		{
			releasedPTO = false;
			ptoEngaged = true;
			leftPTO.set(Value.kForward);
			rightPTO.set(Value.kReverse);// drive mode

		} else if (!driver.getRawButton(9) && !releasedPTO)// happens first
		{
			statePTO = !statePTO;
			releasedPTO = true;

		}
	}

	/**
	 * Control for drivebase and horizontal gyro
	 */
	void Drivebase() {
		double speed = deadband(-driver.getY());
		double turn = deadband(driver.getZ());

		left.set((speed + turn) * SPEEDCAP);
		right.set((-speed + turn) * SPEEDCAP);

		xAngle = horzGyro.getAngle();
		if (driver.getRawButton(12))
			horzGyro.reset();

	}

	/**
	 * Control for hood. Buttons for two specific positions and buttons for
	 * manual movement
	 */
	void Hood() {

		if (hood.getEncPosition() < -1650) {
			hood.set(normPID(-1300, hood.getEncPosition(), .0048, 0));
		} else {
			if (!zeroHood.get()) {

				if (driver.getRawButton(2)) {
					hood.set(0);
				}
				hood.setEncPosition(0);
				hood.set(normPID(50, hood.getEncPosition(), .0048, 0));
			} else {
				// if (driver.getRawButton(1)) {
				// hood.set(.75);
				// else if()
				// hood.set(-.75);
				// } else
				// hood.set(0);

				// far shot
				if (driver.getPOV() == 90) {
					hoodTarget = 624;
					hood.set(normPID(hoodTarget, hood.getEncPosition(), .0048, 0));
					flywheelTarget = 2600;
				}
				// close shot
				else if (driver.getPOV() == 270) {
					hoodTarget = 835;
					hood.set(normPID(hoodTarget, hood.getEncPosition(), .0048, 0));
					flywheelTarget = 2600;
				}

				else if (driver.getPOV() == 0) {
					// if (!hoodAdjusted) {
					// hoodAdjusted = true;
					// hoodTarget += 40;
					// }
					hood.set(1);
				} else if (driver.getPOV() == 180) {
					// if (!hoodAdjusted) {
					// hoodAdjusted = true;
					// hoodTarget -= 40;
					// }
					hood.set(-1);
				} else
					hood.set(0);

			}
		}
	}

	/**
	 * Control for shooter. State machine for smooth ball acquisition to shoot.
	 */
	void Shooter() {

		ballHit = bannerInner.get();

		if (ballHit || ledBool) {
			ledSpike.set(Relay.Value.kForward);
		} else {
			ledSpike.set(Relay.Value.kReverse);
		}

		switch (currShootState) {

		case intake:
			if (driver.getRawButton(1)) {
				setFlyWheelRPM(-900);
			} else
				stopFlyWheel();
			isAtSpeed = false;

			if (driver.getRawButton(5)) {
				setIntake(-1);

			} else if (driver.getRawButton(7)) {
				setIntake(1);
				if (!driver.getRawButton(5)) {
					pidArm = true;
					armPID(-1250);
				}
			} else {
				setIntake(0);
				pidArm = false;
			}

			if (ballHit) {
				setIntake(0);
				currShootState = shootState.reving;
			}

			break;
		case reving:
			if (driver.getRawButton(7)) {
				armPID(0);
				pidArm = true;
			} else {
				pidArm = false;
			}

			if (driver.getRawButton(5)) {
				setIntake(-1);
				currShootState = shootState.intake;
			}

			if (driver.getRawButton(4) && ballHit) {
				setFlyWheelRPM(flywheelTarget);
			} else {
				stopFlyWheel();
			}

			if (isAtSpeed && driver.getRawButton(4)) {
				currShootState = shootState.firing;

				resetFiringState.reset();
				resetFiringState.start();
			}
			break;
		case firing:
			if (ballHit) {
				intakeInner.set(-1);
			}
			if (resetFiringState.get() >= 1) {
				currShootState = shootState.intake;
			}
			break;

		}
	}

	/**
	 * sets flywheel to specified speed.
	 * 
	 * @param rpm
	 *            speed to set flywheel
	 */
	void setFlyWheelRPM(double rpm) {
		flywheel.changeControlMode(TalonControlMode.Speed);
		flywheel.set(rpm);
		flywheelSpeed = flywheel.getSpeed();
		double flywheelError = Math.abs(rpm - flywheelSpeed) - 360;
		if (flywheelError <= 100) {
			isAtSpeed = true;
		} else {
			isAtSpeed = false;
		}
	}

	/**
	 * Stops flywheel by cutting power to the motors.
	 */
	void stopFlyWheel() {
		flywheel.changeControlMode(TalonControlMode.PercentVbus);
		flywheel.set(0);
	}

	/**
	 * sets intake wheels to specified speed and direction
	 * 
	 * @param speed
	 *            voltage to set intake
	 */
	void setIntake(double speed) {
		intakeInner.set(-speed);
		intakeOuter.set(speed);
	}

	/**
	 * Standard PID controller with target, current position, P, and I constants
	 * 
	 * @param myTar
	 *            PID target
	 * @param myPos
	 *            sensor position
	 * @param myP
	 *            P constant
	 * @param myI
	 *            I constant
	 * @return voltage based on PID algorithm
	 */
	double normPID(double myTar, double myPos, double myP, double myI) {
		PIDError = myTar + myPos;
		cumPIDError = PIDError;
		double PIDPout = PIDError * myP;
		double PIDIout = PIDError * myI;
		double PIDSpeed = (PIDPout + PIDIout);

		if (Math.abs(PIDError) < 5) {
			PIDSpeed = 0;
		}
		return (PIDSpeed);
	}

	/**
	 * removes idle joystick input at 8%
	 * 
	 * @param input
	 *            joystick input to be checked
	 * @return trimmed input
	 */
	public double deadband(double input) {
		if (Math.abs(input) < 0.08)
			return 0;

		return input;
	}

	// double gyroPID(double target) {
	// double gyroError;
	//
	// gyroError = target - xAngle;
	// return gyroError * gyroKP;
	//
	// }

}
