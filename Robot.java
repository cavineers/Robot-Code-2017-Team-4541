package org.usfirst.frc.team4541.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogOutput;

import org.usfirst.frc.team4541.robot.AngleController;
import org.usfirst.frc.team4541.robot.ArduinoDueInterface;
import org.usfirst.frc.team4541.robot.MecanumDrive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CameraServer;

import com.kauailabs.navx.frc.AHRS;

public class Robot extends SampleRobot {
	Joystick stick, pad; /// changed
	final String otherside = "otherside";
	CANTalon W1, W2, W3, W4; // talons for the wheels
	CANTalon S1, S2; // talons for the shooter
	CANTalon gearOpen, gearTilt; // Gear Talons
	CANTalon agitator; // Talon for the agitator
	CANTalon climber; // Talon for the climber
	MecanumDrive drive;
	AHRS gyro;
	SendableChooser<String> chooser;
	CANTalon lifterTalon;
	NetworkTable table;
	AngleController controller;

	ShooterController shooter;
	UltrasonicHelper ultraHelper;
	GearMagic magic;

	final String driveForwardAUTO = "forward";
	final String placeGearStupidAUTO = "Stupid Gear";
	final String placeGearMiddleAUTO = "Middle Gear Place";
	String placeGearRightAUTO = "Right Gear Place";
	final String placeGearLeftAUTO = "Left Gear Place";

	public static final boolean IS_TEST_CHASSIS = true;

	public Robot() {
		stick = new Joystick(0);
		lifterTalon = new CANTalon(1);
		SmartDashboard.putNumber("AUTOP", 0.04);
		SmartDashboard.putBoolean("Drive Straight", false);
		chooser = new SendableChooser();
		chooser.addDefault(driveForwardAUTO, driveForwardAUTO);
		chooser.addObject(placeGearStupidAUTO, placeGearStupidAUTO);
		chooser.addObject("placeGearMiddleAUTO", placeGearMiddleAUTO);
		SmartDashboard.putData("Auto Modes", chooser);
	}

	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture(0);
		CameraServer.getInstance().startAutomaticCapture(1);


		W1 = new CANTalon(0);
		W2 = new CANTalon(1);
		W3 = new CANTalon(2);
		W4 = new CANTalon(3);
		climber = new CANTalon(4);
		S1 = new CANTalon(5);
		agitator = new CANTalon(6);
		S2 = new CANTalon(7);
		gearOpen = new CANTalon(8);
		gearTilt = new CANTalon(9);

		drive = new MecanumDrive(W1, W2, W3, W4);
		table = NetworkTable.getTable("GRIP/myContoursReport");
		// try {
		// gyro = new AHRS(SPI.Port.kMXP);
		// gyro.reset();
		// controller = new AngleController(ahrs, stick, drive, 0.01, 0.001,
		// 0.003, 0.001, 0.00, 0.0001);
		// } catch (RuntimeException ex) {
		// DriverStation.reportError("Error instantiating navX-MXP: " +
		// ex.getMessage(), true);
		// }

		gyro = new AHRS(SPI.Port.kMXP);
		gyro.reset();
		controller = new AngleController(gyro, stick, drive, 0.01, 0.001, 0.003, 0.001, 0.00, 0.0001);
		shooter = new ShooterController(S1, S2);
		ultraHelper = new UltrasonicHelper(drive, gyro);
		SmartDashboard.putNumber("Shooter Speed", 0);
		magic = new GearMagic(gyro, controller, table, drive, stick);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomous() { // TODO: uncomment angleController definition,
								// gyro definition, and gyro to smartdashbaord
		// UltrasonicHelper.goToUltrasonicDistance(10, 1);
		// drive.drive(0.5, 0, 0);
		// Timer.delay(0.5);
		// drive.drive(0, 0.5, 0);
		// Timer.delay(0.5);
		// drive.drive(0, 0, 0);
		//
		// gyro.reset();

		double kP = 0.2;
		double kPs = 0.2;
		gyro.reset();
		while (isAutonomous()) {
			kP = SmartDashboard.getNumber("AUTOP", 0);
			String autoSelected = (String) chooser.getSelected();
			if (autoSelected.equals(driveForwardAUTO)) { // Drives forwards for
															// 7 seconds
				// ultraHelper.goToUltrasonicDistance(50, SIDE.RIGHT);
				drive.drive(0.6, 0, 0);
				Timer.delay(2.4541);
				drive.drive(0, 0, 0);
				break;
			} else if (autoSelected.equals(placeGearStupidAUTO)) {
				drive.updateDefaultAngle(gyro);
				drive.straightDrive(0, -0.95, 0, gyro);
				Timer.delay(5); // change to how many seconds it takes to reach
								// the peg
				drive.straightDrive(0, 0, 0, gyro);
				gearOpen.set(-1);
				Timer.delay(0.5);
				drive.straightDrive(0.95, 0, 0, gyro);
				Timer.delay(1.2);
				drive.straightDrive(0, 0, 0, gyro);
				gearOpen.set(1);

			} else if (autoSelected.equals(placeGearMiddleAUTO)) {
				magic.placeGear();
				gearOpen.set(1);
				Timer.delay(0.2);
				gearOpen.set(-1);
				Timer.delay(0.2);
				drive.straightDrive(1, 0, 0, gyro);
				break;
			} else if (autoSelected.equals(placeGearLeftAUTO)) {
				drive.straightDrive(0.7, 0, 0, gyro);
				Timer.delay(3);
				gearOpen.set(1);
				Timer.delay(0.2);
				gearOpen.set(-1);
			}
			break;
		}
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	boolean isClimbing = false;
	boolean isReversed = false;
	boolean offFlap = true;
	boolean offDoors = true;
	boolean areDoorsOpen = false;
	boolean areFlapsOpen = false;
	boolean lastTiltButtonState = false;
	boolean isGearTiltIn = false;
	boolean lastDoorButtonState = false;
	boolean isGearDoorIn = false;
	boolean isGearDoorOpen = false;
	boolean lastStraightButtonState = false;

	public void operatorControl() {
		ArduinoDueInterface.init();
		while (isEnabled()) {
			double[] area = NetworkTable.getTable("GRIP/myContoursReport").getNumberArray("area", new double[] { 0, 0, 0 });
			if (magic.canSeeContours() && area.length == 2) {
			
				SmartDashboard.putNumber("Y AREA:", area[0] + area[1]);
			}
			magic.updateP();
			//ArduinoDueInterface.resetDue();
			this.updateStationToAttempt();
			SmartDashboard.putNumber("S1 Speed", S1.getEncVelocity());
			SmartDashboard.putNumber("S2 Speed", S2.getEncVelocity());

//			double distance1 = ArduinoDueInterface.getUltrasonic(1); // request
//																		// reading
//																		// from
//																		// ultrasonic
//																		// 1
//			double distance2 = ArduinoDueInterface.getUltrasonic(2); // request
//																		// reading
//																		// from
//																		// ultrasonic
//																		// 2
//			double distance3 = ArduinoDueInterface.getUltrasonic(3); // request
//																		// reading
//																		// from
//																		// ultrasonic
//																		// 3
//			double distance4 = ArduinoDueInterface.getUltrasonic(4); // request
//																		// reading
//																		// from
//																		// ultrasonic
//																		// 4
//			 SmartDashboard.putNumber("Distance 1:", distance1);
//			 SmartDashboard.putNumber("Distance 2:", distance2);
//			 SmartDashboard.putNumber("Distance 3:", distance3);
//			 SmartDashboard.putNumber("Distance 4:", distance4);
//			 SmartDashboard.putBoolean("DUE ONLINE", ArduinoDueInterface.isDueOnline());

			SmartDashboard.putNumber("angle", gyro.getYaw());

			if (SmartDashboard.getBoolean("Drive Straight", false)) {
				drive.straightDrive(-drive.modStickIn(stick, 1), drive.modStickIn(stick, 4), -drive.modStickIn(stick, 0),
						gyro);
				// drive.mecanumDrive_Cartesian(-drive.modStickIn(stick, 1),
				// drive.modStickIn(stick, 4), -drive.modStickIn(stick, 0),
				// gyro, false);
			} else {
				drive.updateDefaultAngle(gyro);
				// drive.mecanumDrive_Cartesian(-drive.modStickIn(stick, 1),
				// drive.modStickIn(stick, 4), -drive.modStickIn(stick, 0),
				// gyro, true);
				drive.drive(-drive.modStickIn(stick, 1), drive.modStickIn(stick, 4), -drive.modStickIn(stick, 0)); // This
			}

			if (drive.modStickIn(stick, 1) == 0 && drive.modStickIn(stick, 0) == 0) {
				drive.updateDefaultAngle(gyro);
			}

			// Shooter- set to test 2/20/17
			if (stick.getRawButton(5)) {
				shooter.setSpeed(0.76);
				agitator.set(-1);

			} else if (stick.getRawButton(7)) {
				double speed = SmartDashboard.getNumber("Shooter Speed", 0);
				shooter.setSpeedPID(speed);
				agitator.set(-1);
			} else {
				shooter.setSpeed(0);
				// this.setShooterSpeed(0);
				agitator.set(0);
			}

			if (stick.getRawButton(9)) {
				isReversed = !isReversed;
			}

			// Controls Climber
			if (stick.getRawButton(6)) {
				climber.set(.8);
			} else if (stick.getRawButton(8)) {
				climber.set(0);
			} else {
				climber.set(0);
			}

			shooter.updatePIDf();

			// Toggles Straight Drive
			boolean isStraightDriveEnabled = SmartDashboard.getBoolean("Drive Straight", true);
			if (lastStraightButtonState != stick.getRawButton(8) && stick.getRawButton(8)) { // It
																								// was
																								// just
																								// tapped
				if (isStraightDriveEnabled) {
					SmartDashboard.putBoolean("Drive Straight", false);
				} else {
					SmartDashboard.putBoolean("Drive Straight", true);
				}
				isStraightDriveEnabled = !isStraightDriveEnabled;
			}
			lastStraightButtonState = stick.getRawButton(8);

			// Controls toggling of Gear Flap
			if (!stick.getRawButton(2))
				offFlap = true;
			if (lastTiltButtonState != stick.getRawButton(2) && stick.getRawButton(2)) { // It
																							// was
																							// just
																							// tapped
				if (isGearTiltIn) {
					gearTilt.set(0.5);
				} else {
					gearTilt.set(-0.5);
					gearOpen.set(1);
					isGearDoorOpen = false;
				}
				isGearTiltIn = !isGearTiltIn;
			}
			lastTiltButtonState = stick.getRawButton(2);
			SmartDashboard.putBoolean("IS FLAP OPEN", isGearTiltIn);

			// Controls toggling of Gear Doors
			if (!stick.getRawButton(1))
				offFlap = true;

			if (lastDoorButtonState != stick.getRawButton(1) && stick.getRawButton(1)) { // It
																							// was
																							// just
																							// tapped
				if (isGearDoorOpen) {
					gearOpen.set(1);
				} else {
					gearOpen.set(-1);
				}
				isGearDoorOpen = !isGearDoorOpen;
			}
			lastDoorButtonState = stick.getRawButton(1);
			SmartDashboard.putBoolean("ARE DOORS OPEN", isGearDoorOpen);

			// SmartDashboard.putBoolean("FORWARD OPEN LIMIT SWITCH",
			// gearOpen.isFwdLimitSwitchClosed());
			// SmartDashboard.putBoolean("BACKWARD OPEN LIMIT SWITCH",
			// gearOpen.isRevLimitSwitchClosed());
			//
			// SmartDashboard.putBoolean("FORWARD TILT LIMIT SWITCH",
			// gearTilt.isFwdLimitSwitchClosed());
			// SmartDashboard.putBoolean("BACKWARD TILT LIMIT SWITCH",
			// gearTilt.isRevLimitSwitchClosed());

			// Code for auto gear placement
			if (stick.getRawButton(4)) {
				magic.placeGear();
				// gear.placeGear();
			}
		}
	}

	/**
	 * Runs during test mode
	 */
	public void test() {
		while (isEnabled()) {
			drive.drive(drive.modStickIn(stick, 1), drive.modStickIn(stick, 4), drive.modStickIn(stick, 0));
		}
	}
	
	public void updateStationToAttempt() {
		SmartDashboard.putBoolean("Satation 0", magic.getStationID() == 0 && magic.canSeeContours());
		SmartDashboard.putBoolean("Satation 1", magic.getStationID() == 1 && magic.canSeeContours());
		SmartDashboard.putBoolean("Satation 2", magic.getStationID() == 2 && magic.canSeeContours());
	}

}