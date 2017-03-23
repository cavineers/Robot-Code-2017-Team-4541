package org.usfirst.frc.team4541.robot;

//mecanum drive class
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.kauailabs.navx.frc.AHRS;

public class MecanumDrive implements PIDOutput {
	// drive speed modifiers
	private double[] speedScale = { .25, .5, .75, 1 };
	private int speedIndex;
	// talons must be talon SRX's and must be wired in CAN configuration
	private CANTalon t1, t2, t3, t4;
	public double rotatedAngle;

	public MecanumDrive(CANTalon tal1, CANTalon tal2, CANTalon tal3, CANTalon tal4) {
		t1 = tal1;
		t2 = tal2;
		t3 = tal3;
		t4 = tal4;
		speedIndex = 2;
		rotatedAngle = 0.0;
		SmartDashboard.putNumber("AUTOP", 0.045);
	}

	// drive method
	// to use with joystick control, pass the outupt of the modStickIn methods
	// for the desired axis
	// to use with autonomous control, pass the desired x, y, and rotation
	// values
	public void drive(double x, double y, double rotate) {
		double scale = updateSpeed();
//		mDrive(x * scale, y * scale, rotate * scale);
		if(!Robot.IS_TEST_CHASSIS) {
			x*=-1;
			y*=-1;
		}
		mDrive(x * scale, y * scale, rotate * scale);
		
	}

	public void straightDrive(double x, double r, double y, AHRS gyro) {
		double angle = gyro.getYaw();
		angle -= rotatedAngle;
		angle = AngleController.standardize(angle);
		//double scale = updateSpeed();
		double scale = 1;
		double kP = SmartDashboard.getNumber("AUTOP", 0);
		if(!Robot.IS_TEST_CHASSIS) {
			x*=-1;
			y*=-1;
			
		}

		if (r != 0) {
			drive(x, r, y);
			this.updateDefaultAngle(gyro);
			SmartDashboard.putString("AUTODRIVE", "TURNING");
		} else {
			SmartDashboard.putNumber("STRAIGHT DRIVE Y", y);
			drive(x*scale, angle * -kP, y*scale); // turn to correct heading
			SmartDashboard.putString("AUTODRIVE", "STRAIGHT DRIVING");
		}
		
	}
	
	//A basic straight drive style turn to angle
	public void approachAngle(int angle) {
		double kP = SmartDashboard.getNumber("AUTOP", 0);
		drive(0, angle * -kP, 0);
	}

	public void driveAtSpeed(double x, double y, double rotate, double speed) {
		mDrive(x * speed, y * speed, rotate * speed);
	}

	public void pidWrite(double output) {
		driveAtSpeed(0, 1, 0, output);
	}

	// to be used with directional pad control
	public void polDrive(int angle) {
		double scale = updateSpeed();
		if (angle == 0)
			mDrive(0, scale, 0);
		else if (angle == 45)
			mDrive(scale, scale, 0);
		else if (angle == 90)
			mDrive(scale, 0, 0);
		else if (angle == 135)
			mDrive(scale, -scale, 0);
		else if (angle == 180)
			mDrive(0, -scale, 0);
		else if (angle == 225)
			mDrive(-scale, -scale, 0);
		else if (angle == 270)
			mDrive(-scale, 0, 0);
		else if (angle == 315)
			mDrive(-scale, scale, 0);
		else
			mDrive(0, 0, 0);
	}

	// determines the value to set each motor at based on direction, called
	// within the drive method
	private void mDrive(double right, double forward, double clockwise) {
		double fLeftSpeed, fRightSpeed, rLeftSpeed, rRightSpeed, max;
		boolean norm = false;
		System.out.println(right + "\t" + forward + "\t" + clockwise);
		fLeftSpeed = forward + clockwise + right;
		fRightSpeed = forward - clockwise - right;
		rLeftSpeed = forward + clockwise - right;
		rRightSpeed = forward - clockwise + right;
		max = Math.abs(fLeftSpeed);
		if (Math.abs(fRightSpeed) > max)
			max = Math.abs(fRightSpeed);
		if (Math.abs(rLeftSpeed) > max)
			max = Math.abs(rLeftSpeed);
		if (Math.abs(rRightSpeed) > max)
			max = Math.abs(rRightSpeed);
		if (max > 1) {
			norm = true;
			fLeftSpeed /= max;
			fRightSpeed /= max;
			rLeftSpeed /= max;
			rRightSpeed /= max;
		}
		if (Robot.IS_TEST_CHASSIS) {
			// front right on test chaiss
			t1.set(-fLeftSpeed);
			// front left
			t2.set(-rLeftSpeed);
			// back left
			t3.set(fRightSpeed);
			// back right
			t4.set(rRightSpeed);
		} else {
			// front right on test chaiss
			t1.set(fLeftSpeed);
			t2.set(rLeftSpeed);
			t3.set(fRightSpeed);
			t4.set(rRightSpeed);
		}

	}

	public void rotateToAngle(double angle) {
		SmartDashboard.putString("Test", "It worked");
		double kP = 0.03;
		double kI = 0.00;
		double kD = 0.00;
		double kF = 0.00;
		double kToleranceDegrees = 1.0f;
		AHRS ahrs = new AHRS(SPI.Port.kMXP);
		PIDController turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setSetpoint(angle);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		while (ahrs.getAngle() <= angle && turnController.onTarget()) {
			driveAtSpeed(0, 0, 1, turnController.get());
		}
	}

	// updates the current speed value
	public double updateSpeed() {
		return speedScale[speedIndex];
	}

	// modifies the input of a joystick axis by adding dead zones and squaring
	// the inputs, intended to be used with XBOX controllers or other
	// controllers with many predefined axes
	public double modStickIn(Joystick j1, int num) {
		double joyIn = j1.getRawAxis(num);
		if (joyIn <= .05 && joyIn >= 0)
			joyIn = 0;
		else if (joyIn >= -.05 && joyIn <= 0)
			joyIn = 0;
		else if (joyIn < 0)
			joyIn = -Math.pow(joyIn, 2);
		else
			joyIn = Math.pow(joyIn, 2);
		return joyIn;
	}

	// sets the speed modifier to the next predefined value
	public void speedUp() {
		if (speedIndex != 3)
			speedIndex += 1;
//		Timer.delay(.1);
	}

	// sets the speed modifier to the previous predefined value
	public void slowDown() {
		if (speedIndex != 0)
			speedIndex -= 1;
//		Timer.delay(.1);
	}

	// sets the speed to the value of the speedScale array at the given index
	public void setSpeed(int index) {
		speedIndex = (index > 3 ? 3 : index < 0 ? 0 : index);
	}

	// returns the current speed
	public double getSpeed() {
		return speedScale[speedIndex];
	}

	public void jiggle(boolean jiggle) {
		if (jiggle) {
			t1.set(-0.4);
			t2.set(-0.4);
			t3.set(0.4);
			t4.set(0.4);
		} else {
			t1.set(0);
			t2.set(0);
			t3.set(0);
			t4.set(0);
		}

	}

	// returns the current speed index
	public double getIndex() {
		return speedIndex;
	}
	
	public void updateDefaultAngle(AHRS gyro) {
		rotatedAngle = gyro.getYaw();
	}
	
	public void mecanumDrive_Cartesian(double x, double y, double rotation, AHRS gyro, boolean straightDrive) {
	double gyroAngle = gyro.getAngle();
	double joystickAngle = Math.atan(x/y);
	if( x < 0 )
	{
		joystickAngle+=Math.PI;
	}
	double magnitude = Math.sqrt(x*x + y*y);
	double driveAngle = joystickAngle - (gyroAngle * Math.PI / 180);
	if (straightDrive) {
		this.straightDrive(magnitude * Math.sin(driveAngle), magnitude * Math.cos(driveAngle), rotation, gyro);
	} else {
		this.drive(magnitude * Math.sin(driveAngle), magnitude * Math.cos(driveAngle), rotation);
	}
}
	
}